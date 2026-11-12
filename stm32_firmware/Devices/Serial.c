#include "stm32f4xx.h"
#include "Serial.h"
#include "app_chassis.h"
#include "elrs_Remote.h"
#include "bsp_usart.h"
#include "imu.h"
#include "app_kin.h"
#include "app_tower.h"
#include "tjc_screen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  涓婁綅鏈(natsiie_gcs/ROS) <-> STM32 涓插彛閫氫俊
 *  甯: 0xFF | TYPE | LEN | PAYLOAD | CRC8 | 0xFE
 * ============================================================ */

uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN];   /* 寰呭彂閫佹暟鎹鍖 (8 瀛楄妭) */

/* 鎺ユ敹甯 + 瑙ｆ瀽缁撴灉: ISR 鍐 Serial_RxFrame 骞剁疆 Serial_RxFrameReady,
 * 浠诲姟閲岃皟 Serial_ProcessFrame() 娑堣垂骞舵寜 type 瑙ｆ瀽鍒 Serial_RxData銆 */
volatile uint8_t Serial_RxFrameReady = 0;
Serial_RxFrame_t  Serial_RxFrame;
Serial_RxData_t   Serial_RxData;
volatile uint8_t Serial_UnloadCoordReady = 0;  /* 0x0C 鍗歌揣鍧愭爣(3 缁 xy) 鏂板抚鏍囧織 */
volatile uint8_t Serial_ChassisTrimReady = 0;  /* 0x04/0x05 搴曠洏寰璋(corr) 鏂板抚鏍囧織 */
volatile uint8_t Serial_TurntableTrimReady = 0;/* 0x0E 杞鐩樼籂鍋(corr) 鏂板抚鏍囧織 */
volatile uint8_t Serial_ColorTargetReady = 0;  /* 0x08 棰滆壊鐩鏍囩偣 鏂板抚鏍囧織 */
volatile uint8_t Serial_QRReady = 0;           /* 0x06 浜岀淮鐮 鏂板抚鏍囧織 */
volatile uint8_t Serial_VisionCmd = 0;         /* 宸ヤ綔瑙嗚夋寚浠(0x04/05/08/09/0C)鏂板抚鏍囧織, 渚 mission 瑙嗚夎秴鏃剁湅闂ㄧ嫍 */
volatile uint8_t Serial_TrimFrameType = 0;     /* 最近一帧纠偏帧类型 (0x04/0x05/0x0E), 发 0x11 AlignDone 时定 stage */
uint8_t Serial_AlignDoneEn = 1;                /* 1=开(默认): 每进站到位回一条 0x11 AlignDone; 0=关 */

/* RX 鐘舵佹満: 0=绛夊抚澶 1=鏀禩YPE 2=鏀禠EN 3=鏀惰浇鑽 4=鏀禖RC 5=鏀跺抚灏 */
static uint8_t s_rx_state = 0;
static uint8_t s_rx_idx   = 0;
static uint8_t s_rx_crc   = 0;

static Serial_PlaceCommand_t s_place;
static uint8_t s_place_state = 0; /* 0=none, 1=queued, 2=executing, 3=success, 4=failed */
static char s_place_payload[SERIAL_RX_PAYLOAD_MAX + 1];

static void Serial_SendPlaceResult(const char *task_id, uint32_t command_id, uint8_t success)
{
    char payload[64];
    uint8_t len, crc, i;
    len = (uint8_t)sprintf(payload, "%s,%lu,%u", task_id,
                           (unsigned long)command_id, (unsigned)(success != 0));
    crc = SERIAL_TYPE_PLACE_RESULT ^ len;
    Serial_SendByte(SERIAL_HEAD);
    Serial_SendByte(SERIAL_TYPE_PLACE_RESULT);
    Serial_SendByte(len);
    for (i = 0; i < len; i++) { crc ^= (uint8_t)payload[i]; Serial_SendByte((uint8_t)payload[i]); }
    Serial_SendByte(crc);
    Serial_SendByte(SERIAL_TAIL);
}

/* 缁勫抚鍙戦: 0xFF TYPE LEN PAYLOAD CRC(寮傛垨) 0xFE */
static void Serial_SendFramed(uint8_t type, const char *payload, uint8_t len)
{
    uint8_t crc = type ^ len, i;
    Serial_SendByte(SERIAL_HEAD);
    Serial_SendByte(type);
    Serial_SendByte(len);
    for (i = 0; i < len; i++) { crc ^= (uint8_t)payload[i]; Serial_SendByte((uint8_t)payload[i]); }
    Serial_SendByte(crc);
    Serial_SendByte(SERIAL_TAIL);
}

/* 0x0D PlaceControl: 涓嬩綅鏈鸿Е鍙戜笂浣嶆満鏀剧疆闃舵 (start 鎴 retry) */
void Serial_SendPlaceControlStart(uint8_t stage, uint8_t batch)
{
    char payload[12];
    uint8_t len = (uint8_t)sprintf(payload, "start,%u,%u", (unsigned)stage, (unsigned)batch);
    Serial_SendFramed(SERIAL_TYPE_PLACE_CONTROL, payload, len);
}

void Serial_SendPlaceControlRetry(uint32_t command_id)
{
    char payload[20];
    uint8_t len = (uint8_t)sprintf(payload, "retry,%lu", (unsigned long)command_id);
    Serial_SendFramed(SERIAL_TYPE_PLACE_CONTROL, payload, len);
}

/* 0x0F PickDone: 鎶撳彇瀹屾垚搴忓彿 (1~6)銆備笂浣嶆満 pick_task.Pick.sequence = 鎵规*3 + 绗鍑犱釜 + 1,
 * 鍙鏈夊簭鍙峰拰瀹冨綋鍓嶆湡鏈涘间竴鑷存墠鎺ㄨ繘 (鍚﹀垯鍥 "蹇界暐鎶撳彇瀹屾垚搴忓彿")銆俻ayload 鍙鑳芥槸绾 ASCII 鏁板瓧銆 */
void Serial_SendPickDone(uint8_t seq)
{
    char payload[8];
    uint8_t len = (uint8_t)sprintf(payload, "%u", (unsigned)seq);
    Serial_SendFramed(SERIAL_TYPE_PICK_DONE, payload, len);
}

/* 0x13 PickColor: 硬件抓取颜色 (1~6), 从 0x08 ColorTarget 的 color_id 原样抄回来。
   0x11 现在是 AlignDone (区域矫正完成回传), 颜色挪到 0x13; 视觉暂不消费, 只在日志里显示。 */
void Serial_SendPickColor(uint8_t color)
{
    char payload[8];
    uint8_t len = (uint8_t)sprintf(payload, "%u", (unsigned)color);
    Serial_SendFramed(SERIAL_TYPE_PICK_COLOR, payload, len);
}

/* 0x11 AlignDone: 区域矫正完成 (下位机->上位机, 对齐视觉 race_supervisor_node)
   payload "stage,success": stage 1=物料转盘(0x0E) 2=粗加工三环(0x04) 3=暂存叠料(0x05),
   success 只发 1 (视觉收到 0 只会打日志继续等)。每进站最多一条。 */
void Serial_SendAlignDone(uint8_t stage, uint8_t success)
{
    char payload[8];
    uint8_t len = (uint8_t)sprintf(payload, "%u,%u", (unsigned)stage, (unsigned)(success != 0));
    Serial_SendFramed(SERIAL_TYPE_ALIGN_DONE, payload, len);
}

/* 0x12 RegionReady: arrived and gimbal action done; vision may start ring measurement and send 0x0C.
   payload "stage": 2=ring_station, 3=stacked_ring. Send once per unload chain. */
void Serial_SendRegionReady(uint8_t stage)
{
    char payload[8];
    uint8_t len = (uint8_t)sprintf(payload, "%u", (unsigned)stage);
    Serial_SendFramed(SERIAL_TYPE_REGION_READY, payload, len);
}


/* 0x10 NextBatch: 绗涓鎵(鍓 3 娆℃姄鍙)瀹屾垚銆佸洖瑁呭氨缁銆備笂浣嶆満 next_batch() 鍙鎺ュ彈 "1"锛
 * 鏀跺埌鍚庢墠浠 WAIT_NEXT_BATCH 鏀捐岀浜屾壒 (搴忓彿 4/5/6)銆 */
void Serial_SendNextBatch(uint8_t batch)
{
    char payload[8];
    uint8_t len = (uint8_t)sprintf(payload, "%u", (unsigned)batch);
    Serial_SendFramed(SERIAL_TYPE_NEXT_BATCH, payload, len);
}

uint8_t Serial_TakePlaceCommand(Serial_PlaceCommand_t *out)
{
    if (!out || s_place_state != 1) return 0;
    *out = s_place;
    s_place_state = 2;
    return 1;
}

void Serial_CompletePlaceCommand(const char *task_id, uint32_t command_id, uint8_t success)
{
    if (!task_id || s_place_state != 2 || command_id != s_place.command_id
        || strcmp(task_id, s_place.task_id) != 0) return;
    s_place_state = success ? 3 : 4;
    Serial_SendPlaceResult(s_place.task_id, s_place.command_id, success);
}

/* 0x0B 原始发送: 不受 s_place 队列约束。2026-11-09 视觉改成批量模型后, 0x0B 只表示
 * "本阶段整批放完" (race_supervisor 等一条成功帧就推进下一阶段, 识别源由 0x11
 * AlignDone 的 stage 决定)。到站时不要发, 只在整批卸/放完那一刻发。
 * 视觉只校验 task_id(32 位小写 hex)/command_id(1~2^31-1)/success(0/1)。 */
void Serial_SendPlaceResultRaw(const char *task_id, uint32_t command_id, uint8_t success)
{
    if (!task_id) return;
    Serial_SendPlaceResult(task_id, command_id, success);
}

/**
  * @brief  鍒濆嬪寲 USART1锛歅A9(TX)/PA10(RX)锛115200锛8N1锛孯XNE 鎺ユ敹涓鏂
  */
void Serial_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* NVIC_PriorityGroupConfig 鐢卞伐绋嬬粺涓璁剧疆 (FreeRTOS Group4), 杩欓噷涓嶅啀璋 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;   /* 浣庝紭鍏堢骇锛屽埆鎶 FreeRTOS */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  闃诲炲彂閫 1 瀛楄妭
  */
void Serial_SendByte(uint8_t byte)
{
    USART_SendData(USART1, byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

}

void Serial_SendArray(uint8_t *array, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
        Serial_SendByte(array[i]);
}

void Serial_SendString(char *str)
{
    uint8_t i;
    for (i = 0; str[i] != '\0'; i++)
        Serial_SendByte((uint8_t)str[i]);
}

/* float -> 瀹氱偣瀛楃︿覆 (鏈澶 1 浣嶅皬鏁), 涓嶄緷璧 %f (MicroLib) */
static uint8_t Serial_FormatAngle(float angle, char *buf)
{
    int32_t scaled = (int32_t)(angle * 10.0f + (angle >= 0.0f ? 0.5f : -0.5f));
    uint8_t pos = 0, neg = 0;
    char    tmp[8];
    uint8_t t = 0;
    int32_t ip, fp;

    if (scaled < 0) { neg = 1; scaled = -scaled; }

    ip = scaled / 10;
    fp = scaled % 10;

    do {
        tmp[t++] = (char)('0' + (ip % 10));
        ip /= 10;
    } while (ip > 0);

    if (neg) buf[pos++] = '-';
    while (t > 0) buf[pos++] = tmp[--t];
    if (fp != 0) { buf[pos++] = '.'; buf[pos++] = (char)('0' + fp); }
    buf[pos] = '\0';

    return pos;
}

/* 鍙戦佷簯鍙拌埖鏈鸿掑害缁欎笂浣嶆満 (0x07 GimbalYaw) */
void Serial_SendGimbalYaw(float angle_deg)
{
    char    payload[16];
    uint8_t len, crc, i;

    len = Serial_FormatAngle(angle_deg, payload);

    crc = SERIAL_TYPE_GIMBAL_YAW ^ len;
    for (i = 0; i < len; i++)
        crc ^= (uint8_t)payload[i];

    Serial_SendByte(SERIAL_HEAD);
    Serial_SendByte(SERIAL_TYPE_GIMBAL_YAW);
    Serial_SendByte(len);
    Serial_SendArray((uint8_t *)payload, len);
    Serial_SendByte(crc);
    Serial_SendByte(SERIAL_TAIL);
}

/* 鍙戦佷竴甯ф暟鎹鍖: 0xFF + 8 瀛楄妭 + 0xFE (鏃ф帴鍙, 淇濈暀) */
void Serial_SendPacket(void)
{
    Serial_SendByte(SERIAL_HEAD);
    Serial_SendArray(Serial_TxPacket, SERIAL_TX_DATA_LEN);
    Serial_SendByte(SERIAL_TAIL);
}

/* ---- 鎵嬪姩鏁板艰В鏋 (MicroLib 鏃 sscanf %f) ---- */

static uint8_t Serial_Expect(const char **pp, char c)
{
    if (**pp == c) { (*pp)++; return 1; }
    return 0;
}

static uint8_t Serial_ParseIntTok(const char **pp, int *out)
{
    const char *p = *pp;
    int sign = 1, val = 0;
    uint8_t seen = 0;
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') { p++; }
    while (*p >= '0' && *p <= '9') { val = val * 10 + (int)(*p - '0'); p++; seen = 1; }
    if (!seen) return 0;
    *out = sign * val;
    *pp = p;
    return 1;
}

static uint8_t Serial_ParseFloatTok(const char **pp, float *out)
{
    const char *p = *pp;
    float sign = 1.0f, val = 0.0f, frac = 0.0f, div = 1.0f;
    uint8_t seen = 0;
    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') { p++; }
    while (*p >= '0' && *p <= '9') { val = val * 10.0f + (float)(*p - '0'); p++; seen = 1; }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9') { frac = frac * 10.0f + (float)(*p - '0'); div *= 10.0f; p++; seen = 1; }
    }
    if (!seen) return 0;
    *out = sign * (val + frac / div);
    *pp = p;
    return 1;
}

static uint8_t Serial_ParsePlaceInt(const char **pp, uint32_t *out)
{
    const char *p = *pp;
    uint32_t value = 0, digit;
    if (*p < '0' || *p > '9') return 0;
    while (*p >= '0' && *p <= '9')
    {
        digit = (uint32_t)(*p - '0');
        if (value > (2147483647UL - digit) / 10UL) return 0;
        value = value * 10UL + digit;
        p++;
    }
    *pp = p;
    *out = value;
    return 1;
}

static void Serial_ReceivePlaceCommand(const char *payload)
{
    const char *p = payload;
    Serial_PlaceCommand_t command;
    uint32_t values[7];
    uint8_t i;
    memset(&command, 0, sizeof(command));
    for (i = 0; i < 32; i++)
    {
        if (!((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f'))) return;
        command.task_id[i] = *p++;
    }
    if (!Serial_Expect(&p, ',')) return;
    for (i = 0; i < 7; i++)
        if (!Serial_ParsePlaceInt(&p, &values[i]) || !Serial_Expect(&p, ',')) return;
    if (!Serial_ParseFloatTok(&p, &command.x_mm) || !Serial_Expect(&p, ',')
        || !Serial_ParseFloatTok(&p, &command.y_mm) || !Serial_Expect(&p, ',')
        || !Serial_ParseFloatTok(&p, &command.z_mm) || *p != '\0') return;
    if (values[0] == 0 || values[1] < 1 || values[1] > 2
        || values[2] < 1 || values[2] > 2 || values[3] < 1 || values[3] > 6
        || values[4] < 1 || values[4] > 6 || values[5] < 1 || values[5] > 3
        || values[6] < 1 || values[6] > 2
        || (values[3] - 1) / 3 + 1 != values[2]
        || !isfinite(command.x_mm) || !isfinite(command.y_mm) || !isfinite(command.z_mm)
        || fabsf(command.x_mm) > 10000.0f || fabsf(command.y_mm) > 10000.0f
        || fabsf(command.z_mm) > 10000.0f) return;
    command.command_id = values[0]; command.stage = (uint8_t)values[1];
    command.batch = (uint8_t)values[2]; command.sequence = (uint8_t)values[3];
    command.color = (uint8_t)values[4]; command.ring = (uint8_t)values[5];
    command.layer = (uint8_t)values[6];
    if (s_place_state && strcmp(command.task_id, s_place.task_id) == 0
        && command.command_id == s_place.command_id)
    {
        if (strcmp(payload, s_place_payload) != 0) return;
        if (s_place_state == 3) Serial_SendPlaceResult(s_place.task_id, s_place.command_id, 1);
        else if (s_place_state == 4) s_place_state = 1;
        return;
    }
    if (s_place_state == 1 || s_place_state == 2)
    {
        Serial_SendPlaceResult(command.task_id, command.command_id, 0);
        return;
    }
    if (s_place_state && strcmp(command.task_id, s_place.task_id) == 0
        && command.command_id < s_place.command_id) return;
    s_place = command;
    strcpy(s_place_payload, payload);
    s_place_state = 1;
}

/**
  * @brief  鎸 type 瑙ｆ瀽 Serial_RxFrame, 缁撴灉鍐 Serial_RxData銆
  *         0x08 鐐逛綅瑙ｆ瀽鎴愬姛鍚庤嚜鍔ㄨЕ鍙 kin_cmd=1 (GotoXY)銆
  *         鍦ㄤ换鍔￠噷璋冪敤 (姣 10ms, 鏀跺埌甯у悗)銆
  */

/* ================= PID 自动调参状态机 ================= */
typedef struct {
    uint8_t  active;
    uint8_t  is_yaw;       /* 1=Yaw阶跃, 0=Pos阶跃 */
    uint16_t t_cur_ms;
    uint16_t t_max_ms;
    uint8_t  decim;        /* 采样降频: 每 2 拍 (20ms) 发一次 */
    float    init_val;
    float    target_val;
} PidTuneSession_t;

static PidTuneSession_t s_pid_tune = {0};

void Serial_SendPidAck(uint8_t ok, const char *msg)
{
    char payload[64];
    uint8_t len = 0;
    const char *status = ok ? "OK" : "ERR";
    while (*status) payload[len++] = *status++;
    payload[len++] = ',';
    if (msg)
    {
        while (*msg && len < 60) payload[len++] = *msg++;
    }
    payload[len] = '\0';
    Serial_SendFramed(SERIAL_TYPE_PID_ACK, payload, len);
}

void Serial_PidTune_Tick(void)
{
    char buf[64];
    uint8_t len;
    float cur_val, cur_out;
    uint16_t t;

    if (!s_pid_tune.active) return;

    /* ================= 遥控器安全互锁: 遥控器拥有一票否决权 ================= */
    if (ELRS_RC.online)
    {
        /* 1. 遥控器未解锁或拨杆锁底盘 (sw[0] != 2) -> 强制终止阶跃测试 */
        if (chassis_flag.need_sw_unlock && ELRS_RC.sw[0] != 2)
        {
            s_pid_tune.active = 0;
            chassis_flag.rotate_active = 0;
            chassis_flag.pos_active = 0;
            Serial_SendPidAck(0, "ABORT_REMOTE_LOCKED");
            return;
        }
        /* 2. 遥控推杆人工介入 (摇杆偏离 > 0.2) -> 立即将底盘控制权归还人工 */
        if (fabsf(ELRS_RC.Lx) > 0.2f || fabsf(ELRS_RC.Rx) > 0.2f || fabsf(ELRS_RC.Ry) > 0.2f)
        {
            s_pid_tune.active = 0;
            chassis_flag.rotate_active = 0;
            chassis_flag.pos_active = 0;
            Serial_SendPidAck(0, "ABORT_RC_OVERRIDE");
            return;
        }
    }


    s_pid_tune.t_cur_ms += 10;
    s_pid_tune.decim++;

    if (s_pid_tune.is_yaw)
    {
        cur_val = Imu_GetYaw();
        cur_out = chassis_val.yaw_out;
        /* 异常保护: 如果偏转发散超过 90 度，立即急停 */
        if (fabsf(cur_val - s_pid_tune.init_val) > 90.0f)
        {
            s_pid_tune.active = 0;
            chassis_flag.rotate_active = 0;
            chassis_val.yaw_target = cur_val;
            Serial_SendPidAck(0, "ABORT_YAW_DIVERGE");
            return;
        }
    }
    else
    {
        cur_val = chassis_val.odom_x_mm;
        cur_out = chassis_val.odom_vx_mmps;
        /* 异常保护: 如果平移超程超过 2000mm，立即急停 */
        if (fabsf(cur_val - s_pid_tune.init_val) > 2000.0f)
        {
            s_pid_tune.active = 0;
            chassis_flag.pos_active = 0;
            Serial_SendPidAck(0, "ABORT_POS_OVERRUN");
            return;
        }
    }

    /* 每 20ms 发送一次打点数据 */
    if (s_pid_tune.decim >= 2)
    {
        s_pid_tune.decim = 0;
        t = s_pid_tune.t_cur_ms;

        len = (uint8_t)sprintf(buf, "%u,", (unsigned)t);
        len += Serial_FormatAngle(s_pid_tune.target_val, buf + len);
        buf[len++] = ',';
        len += Serial_FormatAngle(cur_val, buf + len);
        buf[len++] = ',';
        len += Serial_FormatAngle(cur_out, buf + len);
        buf[len] = '\0';

        Serial_SendFramed(SERIAL_TYPE_PID_DATA, buf, len);
    }

    /* 测试时长到达 */
    if (s_pid_tune.t_cur_ms >= s_pid_tune.t_max_ms)
    {
        s_pid_tune.active = 0;
        if (s_pid_tune.is_yaw)
        {
            chassis_flag.rotate_active = 0;
            chassis_val.yaw_target = cur_val;
        }
        else
        {
            chassis_flag.pos_active = 0;
        }
        Serial_SendFramed(SERIAL_TYPE_PID_DATA, "DONE", 4);
    }
}

void Serial_ProcessFrame(void)
{
    const char *p;
    uint8_t n = Serial_RxFrame.len;
    if (n > SERIAL_RX_PAYLOAD_MAX) return;
    Serial_RxFrame.payload[n] = '\0';
    p = (const char *)Serial_RxFrame.payload;

    Serial_RxData.type = Serial_RxFrame.type;

    switch (Serial_RxFrame.type)
    {
    case SERIAL_TYPE_PID_SET: /* 0x20 */
    {
        if (strncmp(p, "STOP", 4) == 0)
        {
            s_pid_tune.active = 0;
            chassis_flag.rotate_active = 0;
            chassis_flag.pos_active = 0;
            Serial_SendPidAck(1, "STOPPED");
        }
        else if (strncmp(p, "YAW,", 4) == 0)
        {
            float kp, kd_rate, ki;
            p += 4;
            if (Serial_ParseFloatTok(&p, &kp) && Serial_Expect(&p, ',') &&
                Serial_ParseFloatTok(&p, &kd_rate) && Serial_Expect(&p, ',') &&
                Serial_ParseFloatTok(&p, &ki))
            {
                if (kp < 0.0f) kp = 0.0f;
                if (kp > 10.0f) kp = 10.0f;
                if (kd_rate > 0.0f) kd_rate = 0.0f;
                if (kd_rate < -5.0f) kd_rate = -5.0f;
                if (ki < 0.0f) ki = 0.0f;
                if (ki > 1.0f) ki = 1.0f;

                chassis_cfg.yaw_kp = kp;
                chassis_cfg.yaw_kd_rate = kd_rate;
                chassis_cfg.yaw_ki = ki;
                Serial_SendPidAck(1, "YAW_PID_UPDATED");
            }
            else
            {
                Serial_SendPidAck(0, "PARSE_ERR");
            }
        }
        else if (strncmp(p, "POS,", 4) == 0)
        {
            float kp, dz, spd;
            p += 4;
            if (Serial_ParseFloatTok(&p, &kp) && Serial_Expect(&p, ',') &&
                Serial_ParseFloatTok(&p, &dz) && Serial_Expect(&p, ',') &&
                Serial_ParseFloatTok(&p, &spd))
            {
                if (kp < 0.0f) kp = 0.0f;
                if (kp > 3.0f) kp = 3.0f;
                if (dz < 0.5f) dz = 0.5f;
                if (dz > 100.0f) dz = 100.0f;
                if (spd < 5.0f) spd = 5.0f;
                if (spd > 150.0f) spd = 150.0f;

                chassis_cfg.pos_kp = kp;
                chassis_cfg.pos_deadzone_mm = dz;
                chassis_cfg.pos_max_speed = spd;
                Serial_SendPidAck(1, "POS_PID_UPDATED");
            }
            else
            {
                Serial_SendPidAck(0, "PARSE_ERR");
            }
        }
        break;
    }

    case SERIAL_TYPE_PID_STEP: /* 0x21 */
    {
        if (strncmp(p, "YAW,", 4) == 0)
        {
            float delta;
            int dur;
            p += 4;
            if (Serial_ParseFloatTok(&p, &delta) && Serial_Expect(&p, ',') &&
                Serial_ParseIntTok(&p, &dur))
            {
                if (dur < 200) dur = 200;
                if (dur > 5000) dur = 5000;
                if (delta > 90.0f) delta = 90.0f;
                if (delta < -90.0f) delta = -90.0f;

                s_pid_tune.active = 1;
                s_pid_tune.is_yaw = 1;
                s_pid_tune.t_cur_ms = 0;
                s_pid_tune.t_max_ms = (uint16_t)dur;
                s_pid_tune.decim = 0;
                s_pid_tune.init_val = Imu_GetYaw();
                s_pid_tune.target_val = s_pid_tune.init_val + delta;

                Chassis_RotateTo(s_pid_tune.target_val);
                Serial_SendPidAck(1, "STEP_YAW_STARTED");
            }
            else
            {
                Serial_SendPidAck(0, "PARSE_ERR");
            }
        }
        else if (strncmp(p, "POS,", 4) == 0)
        {
            float delta;
            int dur;
            p += 4;
            if (Serial_ParseFloatTok(&p, &delta) && Serial_Expect(&p, ',') &&
                Serial_ParseIntTok(&p, &dur))
            {
                if (dur < 200) dur = 200;
                if (dur > 10000) dur = 10000;
                if (delta > 2000.0f) delta = 2000.0f;
                if (delta < -2000.0f) delta = -2000.0f;

                s_pid_tune.active = 1;
                s_pid_tune.is_yaw = 0;
                s_pid_tune.t_cur_ms = 0;
                s_pid_tune.t_max_ms = (uint16_t)dur;
                s_pid_tune.decim = 0;
                s_pid_tune.init_val = chassis_val.odom_x_mm;
                s_pid_tune.target_val = s_pid_tune.init_val + delta;

                chassis_val.pos_target_x_mm = s_pid_tune.target_val;
                chassis_val.pos_target_y_mm = chassis_val.odom_y_mm;
                chassis_flag.pos_active = 1;
                Serial_SendPidAck(1, "STEP_POS_STARTED");
            }
            else
            {
                Serial_SendPidAck(0, "PARSE_ERR");
            }
        }
        break;
    }

    case SERIAL_TYPE_PLACE_COMMAND:
        Serial_ReceivePlaceCommand(p);
        break;
    case SERIAL_TYPE_COLOR:   /* 0x01: id:cx,cy/id:cx,cy/... */
    {
        uint8_t cnt = 0;
        while (cnt < 6)
        {
            int id, cx, cy;
            if (!Serial_ParseIntTok(&p, &id)) break;
            if (!Serial_Expect(&p, ':')) break;
            if (!Serial_ParseIntTok(&p, &cx)) break;
            if (!Serial_Expect(&p, ',')) break;
            if (!Serial_ParseIntTok(&p, &cy)) break;
            Serial_RxData.color_id[cnt] = (uint8_t)id;
            Serial_RxData.color_cx[cnt] = (int16_t)cx;
            Serial_RxData.color_cy[cnt] = (int16_t)cy;
            cnt++;
            if (*p == '/') { p++; continue; }
            break;
        }
        Serial_RxData.color_count = cnt;
        break;
    }

    case SERIAL_TYPE_COLOR_MOTION:   /* 0x02: state:speed */
    {
        int st; float sp;
        if (Serial_ParseIntTok(&p, &st) && Serial_Expect(&p, ':') &&
            Serial_ParseFloatTok(&p, &sp))
        {
            Serial_RxData.motion_state = (uint8_t)st;
            Serial_RxData.motion_speed = sp;
        }
        break;
    }

    case SERIAL_TYPE_TURNTABLE:   /* 0x03: cx,cy,radius,stable */
    {
        float cx, cy, r; int stable;
        if (Serial_ParseFloatTok(&p, &cx) && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &cy) && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &r)  && Serial_Expect(&p, ',') &&
            Serial_ParseIntTok(&p, &stable))
        {
            Serial_RxData.turntable_cx = cx;
            Serial_RxData.turntable_cy = cy;
            Serial_RxData.turntable_r  = r;
            Serial_RxData.turntable_stable = (uint8_t)stable;
        }
        break;
    }

    case SERIAL_TYPE_RING_STATION:   /* 0x04: err_x,err_y,ang,corr_x,corr_y,corr_yaw,within */
    case SERIAL_TYPE_STACKED_RING:   /* 0x05: 鍚 0x04 */
    {
        float ex, ey, ang, cx, cy, cyaw; int w;
        if (Serial_ParseFloatTok(&p, &ex)   && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &ey)   && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &ang)  && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &cx)   && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &cy)   && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &cyaw) && Serial_Expect(&p, ',') &&
            Serial_ParseIntTok(&p, &w))
        {
            Serial_RxData.ring_err_x    = ex;
            Serial_RxData.ring_err_y    = ey;
            Serial_RxData.ring_ang_err  = ang;
            Serial_RxData.ring_corr_x   = cx;
            Serial_RxData.ring_corr_y   = cy;
            Serial_RxData.ring_corr_yaw = cyaw;
            Serial_RxData.ring_within   = (uint8_t)w;

            /* 閫氱煡涓氬姟灞: corr=搴曠洏寰璋(搴曠洏); err 鏄鍋忕Щ鍧愭爣, 鐢变笂灞傛寜闇浣跨敤 */
            Serial_TrimFrameType = Serial_RxFrame.type;
            Serial_ChassisTrimReady = 1;
            Serial_VisionCmd = 1;
        }
        break;
    }

    case SERIAL_TYPE_TURNTABLE_ALIGN:   /* 0x0E: corr_x,corr_y,within (鐗╂枡鍖鸿浆鐩, 鏃 err/鏃 yaw) */
    {
        float cx, cy; int w;
        if (Serial_ParseFloatTok(&p, &cx) && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &cy) && Serial_Expect(&p, ',') &&
            Serial_ParseIntTok(&p, &w))
        {
            /* 澶嶇敤 ring_* 妲戒綅: 涓嬫父 Chassis_ApplyVisionTrim 鍙璁よ繖涓缁 corr銆
               杞鐩樺抚娌℃湁 err 鍜 yaw, 蹇呴』鏄惧紡娓 0, 鍚﹀垯浼氱暀鐫涓婁竴鏉 0x04/0x05 鐨勬棫鍊
               (yaw 娈嬬暀浼氳╁簳鐩樻寜鏃х殑 corr_yaw 杞杩囧ご)銆 */
            Serial_RxData.ring_err_x    = 0.0f;
            Serial_RxData.ring_err_y    = 0.0f;
            Serial_RxData.ring_ang_err  = 0.0f;
            Serial_RxData.ring_corr_x   = cx;
            Serial_RxData.ring_corr_y   = cy;
            Serial_RxData.ring_corr_yaw = 0.0f;
            Serial_RxData.ring_within   = (uint8_t)w;

            /* 鍗曠嫭鏍囧織: 杞鐩樿妭鐐瑰父寮, 鐢卞簳鐩樹晶閰嶅悎 chassis_flag.trim_armed 鎵嶆敹 */
            Serial_TrimFrameType = SERIAL_TYPE_TURNTABLE_ALIGN;
            Serial_TurntableTrimReady = 1;
            Serial_VisionCmd = 1;
        }
        break;
    }

    case SERIAL_TYPE_UNLOAD_COORD:   /* 0x0C: 涓夌粍 x,y (3 涓鍗歌揣钀界偣, 濉斿悐绯荤粷瀵 mm) */
    {
        float v[6];
        uint8_t k = 0, i;

        /* 瀹瑰繊鍒嗛殧绗 (, ; / 鍜岀┖鐧), 椤哄簭璇 6 涓鏁 -> 3 缁 (x,y) */
        while (k < 6)
        {
            while (*p == ',' || *p == ';' || *p == '/' ||
                   *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
            if (!Serial_ParseFloatTok(&p, &v[k])) break;
            k++;
        }

        Serial_RxData.unload_count = k / 2;
        if (Serial_RxData.unload_count > 3) Serial_RxData.unload_count = 3;
        for (i = 0; i < Serial_RxData.unload_count; i++)
        {
            Serial_RxData.unload_x[i] = v[i * 2];
            Serial_RxData.unload_y[i] = v[i * 2 + 1];
        }

        if (Serial_RxData.unload_count > 0)
        {
            Serial_UnloadCoordReady = 1;
            Serial_VisionCmd = 1;
        }
        break;
    }

    case SERIAL_TYPE_QR_DECODE:   /* 0x06: 浠诲姟鐮佸瓧绗︿覆 -> 瑙ｆ瀽 4 缁勬樉绀哄埌灞 t0/t2 */
    {
        uint8_t i;
        for (i = 0; i < n; i++) Serial_RxData.str[i] = (char)Serial_RxFrame.payload[i];
        Serial_RxData.str[n] = '\0';
        Screen_ShowQRCode(Serial_RxData.str);   /* t0=鍓嶄袱缁 t2=鍚庝袱缁, 瑕嗙洊鍗犱綅 */
        Serial_QRReady = 1;                     /* 閫氱煡 mission 瑙ｆ瀽鐜搴 */
        break;
    }

    case SERIAL_TYPE_COLOR_TARGET:   /* 0x08: color_id,x_mm,y_mm,z_mm */
    {
        int id; float x, y, z;
        if (Serial_ParseIntTok(&p, &id)  && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &x) && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &y) && Serial_Expect(&p, ',') &&
            Serial_ParseFloatTok(&p, &z))
        {
            Serial_RxData.target_color_id = (uint8_t)id;
            Serial_RxData.target_x_mm = x;
            Serial_RxData.target_y_mm = y;
            Serial_RxData.target_z_mm = z;

            /* 鍙瀛樺潗鏍, 涓嶅姩: 涓婂眰鍙缁 (x,y), z 鐢 kin_cal.grab_z_mm 瀹氭; 绛 Watch 鍐 kin_flag.cmd=1 鎵嶈蛋銆
               鎶撳彇鎵ц屼腑(kin_flag.active)鎷掔粷鏂 0x08, 闃茶嗚夎璇嗗埆瑕嗙洊姝ｅ湪鐢ㄧ殑鐩鏍囧潗鏍 */
            if (!kin_flag.active)
            {
                kin_tgt.target_x = x;
                kin_tgt.target_y = y;
                Serial_ColorTargetReady = 1;   /* 閫氱煡 mission: 鏂伴滆壊鐩鏍囩偣宸查攣瀛 */
            }
            Serial_VisionCmd = 1;
        }
        break;
    }

    case SERIAL_TYPE_GRAB_CMD:   /* 0x09: "1"=瑙嗚夋巿鏉冩姄鍙 (鐗╂枡鍋滅ǔ/澶圭埅鍒颁綅) */
    {
        int v = 0;
        if (Serial_ParseIntTok(&p, &v) && v != 0)
        {
            Serial_RxData.grab_cmd = 1;   /* 寰呮秷璐规爣蹇: app_kin 闂鐖鏃舵竻 0, 涓嶄細涓鐩存寕 1 */
            if (Serial_RxData.grab_cmd_cnt < 60000) Serial_RxData.grab_cmd_cnt++;   /* 鍒拌揪璁℃暟, 鍙澧炰笉娓 */
            grab_cmd = 1;   /* 瑁呰揣娴佺▼: 鐢垫満5 浠550闄嶅埌447 涓嬫帰鎶撳彇 */
            Serial_VisionCmd = 1;
        }
        break;
    }

    default:
        break;
    }
}

/**
  * @brief  USART1 鎺ユ敹涓鏂: 甯х粨鏋勭姸鎬佹満
  *         0=绛夊抚澶 1=TYPE 2=LEN 3=杞借嵎 4=CRC 5=甯у熬
  */

/* 通用字节接收状态机 (供 USART1 和 USART6 共同调用) */
void Serial_FeedByte(uint8_t b)
{
    switch (s_rx_state)
    {
    case 0:
        if (b == SERIAL_HEAD) s_rx_state = 1;
        break;
    case 1:
        Serial_RxFrame.type = b;
        s_rx_crc = b;
        s_rx_state = 2;
        break;
    case 2:
        Serial_RxFrame.len = b;
        s_rx_crc ^= b;
        s_rx_idx = 0;
        if (b == 0) s_rx_state = 4;
        else if (b <= SERIAL_RX_PAYLOAD_MAX) s_rx_state = 3;
        else s_rx_state = 0;
        break;
    case 3:
        Serial_RxFrame.payload[s_rx_idx++] = b;
        s_rx_crc ^= b;
        if (s_rx_idx >= Serial_RxFrame.len) s_rx_state = 4;
        break;
    case 4:
        if (b == s_rx_crc) s_rx_state = 5;
        else s_rx_state = 0;
        break;
    case 5:
        if (b == SERIAL_TAIL) Serial_RxFrameReady = 1;
        s_rx_state = 0;
        break;
    default:
        s_rx_state = 0;
        break;
    }
}

void USART1_IRQHandler(void)
{
    uint8_t d;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        d = USART_ReceiveData(USART1);

        switch (s_rx_state)
        {
        case 0:   /* 绛夊抚澶 */
            if (d == SERIAL_HEAD) s_rx_state = 1;
            break;

        case 1:   /* TYPE */
            Serial_RxFrame.type = d;
            s_rx_crc = d;
            s_rx_state = 2;
            break;

        case 2:   /* LEN */
            Serial_RxFrame.len = d;
            s_rx_crc ^= d;
            s_rx_idx = 0;
            s_rx_state = (d == 0) ? 4 : 3;   /* 鏃犺浇鑽风洿鎺ョ瓑 CRC */
            break;

        case 3:   /* PAYLOAD */
            s_rx_crc ^= d;
            if (s_rx_idx < Serial_RxFrame.len)
            {
                if (s_rx_idx < SERIAL_RX_PAYLOAD_MAX)
                    Serial_RxFrame.payload[s_rx_idx] = d;
                s_rx_idx++;
            }
            if (s_rx_idx >= Serial_RxFrame.len) s_rx_state = 4;
            break;

        case 4:   /* CRC */
            if (d == s_rx_crc) s_rx_state = 5;
            else s_rx_state = 0;   /* CRC 閿, 閲嶆柊鍚屾 */
            break;

        case 5:   /* 甯у熬 */
            if (d == SERIAL_TAIL) Serial_RxFrameReady = 1;
            s_rx_state = 0;
            break;

        default:
            s_rx_state = 0;
            break;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }

    /* 鏈鍚庢竻 ORE(婧㈠嚭): 蹇呴』鍦 RXNE 涔嬪悗娓, 鍚﹀垯浼氭妸寰呰诲瓧鑺備涪鎺 (璇 SR 鍐嶈 DR 鎵嶆竻 ORE) */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        volatile uint32_t tmp = USART1->SR; (void)tmp;
        tmp = USART1->DR; (void)tmp;
    }
}
