#include "elrs_Remote.h"
#include "crsf_protocol.h"
#include "math.h"
#include "stdlib.h"

ELRS_Data_t ELRS_RC;
uint8_t rc_btn_pressed[6] = {0};  /* edge events: 1 = just pressed (consumer clears) */

/* Joystick axis calibration */
typedef struct {
    int16_t raw_min;
    int16_t raw_mid;
    int16_t raw_max;
    int16_t deadzone;
    float   scale_pos; /* positive side scale */
    float   scale_neg; /* negative side scale */
} Axis_Cfg_t;

Axis_Cfg_t Axis_Cfg[4];

/* Compute deadzone-removed normalized scale */
static void Calc_Axis_Scale(Axis_Cfg_t *axis) {
    float range_pos = (float)(axis->raw_max - axis->raw_mid - axis->deadzone);
    float range_neg = (float)(axis->raw_mid - axis->raw_min - axis->deadzone);
    axis->scale_pos = (range_pos > 1.0f) ? (1.0f / range_pos) : 1.0f;
    axis->scale_neg = (range_neg > 1.0f) ? (1.0f / range_neg) : 1.0f;
}

/* Map CRSF raw (192~1792) to 1000~2000 PWM */
static uint16_t Get_CH_1000_2000(uint8_t ch_idx) {
    int16_t raw = CRSF_Channels[ch_idx];
    if (raw < 192) raw = 192;
    if (raw > 1792) raw = 1792;
    /* raw * (1000/1600) + 1000 - (192*5/8) = raw*5/8 + 880 */
    return (uint16_t)((raw * 5) / 8 + 880);
}

/* Two 3-pos switches in one channel (9 states, step 125) */
static void Unpack_Two_3Pos(uint16_t ch_val, uint8_t *swA, uint8_t *swB) {
    uint8_t state = (ch_val - 1000 + 62) / 125;
    if (state > 8) state = 8;
    *swA = state / 3;
    *swB = state % 3;
}

/* 3-pos + 2-pos switch in one channel (6 states, step 200) */
static void Unpack_3Pos_2Pos(uint16_t ch_val, uint8_t *sw3, uint8_t *sw2) {
    uint8_t state = (ch_val - 1000 + 100) / 200;
    if (state > 5) state = 5;
    *sw3 = state / 2;
    *sw2 = state % 2;
}

/* Three buttons in one channel (8 states, step 143) */
static void Unpack_Three_Btns(uint16_t ch_val, bool *btnA, bool *btnB, bool *btnC) {
    uint8_t state = (ch_val - 1000 + 71) / 143;
    if (state > 7) state = 7;
    *btnA = (state >> 2) & 0x01;
    *btnB = (state >> 1) & 0x01;
    *btnC = state & 0x01;
}

/* Single 2-pos switch */
static uint8_t Unpack_Single_2Pos(uint16_t ch_val) {
    return (ch_val < 1500) ? 0 : 1;
}

/* Single 3-pos switch */
static uint8_t Unpack_Single_3Pos(uint16_t ch_val) {
    if (ch_val < 1333) return 0;
    else if (ch_val > 1666) return 2;
    else return 1;
}

void ELRS_Remote_Init(void) {
    /* --- default axis values --- */
    for (int i = 0; i < 4; i++) {
        Axis_Cfg[i].raw_min = 191;
        Axis_Cfg[i].raw_mid = 952;
        Axis_Cfg[i].raw_max = 1790;
        Axis_Cfg[i].deadzone = 50;
    }

    Axis_Cfg[0].raw_min = 215; Axis_Cfg[0].raw_mid = 1018; Axis_Cfg[0].raw_max = 1792;
    Axis_Cfg[1].raw_min = 228; Axis_Cfg[1].raw_mid = 1040; Axis_Cfg[1].raw_max = 1792;
    Axis_Cfg[2].raw_min = 196; Axis_Cfg[2].raw_mid = 994;  Axis_Cfg[2].raw_max = 1792;

    /* calc scale for all 4 channels */
    for (int i = 0; i < 4; i++) {
        Calc_Axis_Scale(&Axis_Cfg[i]);
    }
}

/* Joystick axis -> normalized [-1,1], deadzone + cubic curve */
static float Process_Joystick(Axis_Cfg_t *cfg, uint16_t raw_val) {
    int16_t offset = (int16_t)raw_val - cfg->raw_mid;
    if (abs(offset) < cfg->deadzone) return 0.0f;

    float norm = 0.0f;
    if (offset > 0) {
        norm = (offset - cfg->deadzone) * cfg->scale_pos;
    } else {
        norm = (offset + cfg->deadzone) * cfg->scale_neg;
    }
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm * norm * norm;
}

void ELRS_Remote_Update(void) {
    /* --- offline protection --- */
    if (xTaskGetTickCount() - CRSF_LastRxTime > pdMS_TO_TICKS(100)) {
        ELRS_RC.online = 0;
        ELRS_RC.Lx = ELRS_RC.Ly = ELRS_RC.Rx = ELRS_RC.Ry = 0.0f;
        return;
    }
    ELRS_RC.online = 1;

    /* --- [1] joystick (channels 0/1/2, Ly reserved) --- */
    ELRS_RC.Rx = Process_Joystick(&Axis_Cfg[0], CRSF_Channels[0]); /* CH1 */
    ELRS_RC.Ry = Process_Joystick(&Axis_Cfg[1], CRSF_Channels[1]); /* CH2 */
    ELRS_RC.Lx = Process_Joystick(&Axis_Cfg[2], CRSF_Channels[2]); /* CH3 */
    /* Ly: not parsed, reserved */

    /* --- [2] switches (channels 3~7) --- */
    ELRS_RC.sw[7] = Unpack_Single_3Pos(Get_CH_1000_2000(3));               /* CH4 */
    ELRS_RC.sw[0] = Unpack_Single_3Pos(Get_CH_1000_2000(4));               /* CH5 */
    Unpack_Two_3Pos(Get_CH_1000_2000(5), &ELRS_RC.sw[1], &ELRS_RC.sw[6]);  /* CH6 */
    Unpack_3Pos_2Pos(Get_CH_1000_2000(6), &ELRS_RC.sw[2], &ELRS_RC.sw[3]); /* CH7 */
    Unpack_Two_3Pos(Get_CH_1000_2000(7), &ELRS_RC.sw[5], &ELRS_RC.sw[4]);  /* CH8 */

    /* --- [3] knobs (channels 8/9) --- */
    {
        int16_t val9 = CRSF_Channels[8];
        if (val9 < 192) val9 = 192;
        if (val9 > 1792) val9 = 1792;
        ELRS_RC.knobs[0] = (val9 - 192) * 5 / 8;
    }
    {
        int16_t val10 = CRSF_Channels[9];
        if (val10 < 192) val10 = 192;
        if (val10 > 1792) val10 = 1792;
        ELRS_RC.knobs[1] = (val10 - 192) * 5 / 8;
    }

    /* --- [4] buttons (channels 10/11) --- */
    Unpack_Three_Btns(Get_CH_1000_2000(10), &ELRS_RC.btns[0], &ELRS_RC.btns[1], &ELRS_RC.btns[2]); /* CH11 */
    Unpack_Three_Btns(Get_CH_1000_2000(11), &ELRS_RC.btns[3], &ELRS_RC.btns[4], &ELRS_RC.btns[5]); /* CH12 */

    /* sw[3]==1: field-config mode, 6 channels -> field_info[12] (2 bits/cell) */
    if (ELRS_RC.sw[3] == 1) {
        ELRS_RC.Rx = 0; ELRS_RC.Ry = 0; ELRS_RC.Lx = 0;  /* CH1/CH2/CH3 repurposed */
        ELRS_RC.sw[1] = 0; ELRS_RC.sw[6] = 0;             /* CH6 repurposed */
        int8_t ch_idx[6] = {0, 1, 2, 5, 7, 11};           /* CH1,CH2,CH3,CH6,CH8,CH12 */
        for (int ch = 0; ch < 6; ch++) {
            int16_t raw = CRSF_Channels[ch_idx[ch]];
            if (raw < 192) raw = 192;
            uint16_t v = (raw - 192 + 10) / 20;           /* v=0~15 */
            if (v > 15) v = 15;
            for (int i = 1; i >= 0; i--) {
                ELRS_RC.field_info[ch * 2 + i] = v & 0x03;
                v >>= 2;
            }
        }
        ELRS_RC.field_info_updated = 1;
    }

    /* edge detection: suppress all buttons for first 14s after boot */
    static uint8_t prev_btns[6];
    static uint32_t boot_tick = 0;
    if (boot_tick == 0) boot_tick = xTaskGetTickCount();
    if (xTaskGetTickCount() - boot_tick < pdMS_TO_TICKS(14000)) {
        for (int i = 0; i < 6; i++) prev_btns[i] = ELRS_RC.btns[i];
    } else {
        for (int i = 0; i < 6; i++) {
            if (ELRS_RC.btns[i] != prev_btns[i]) {
                rc_btn_pressed[i] = 1;
                prev_btns[i] = ELRS_RC.btns[i];
            }
        }
    }
}
