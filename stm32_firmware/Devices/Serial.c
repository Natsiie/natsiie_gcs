#include "stm32f4xx.h"
#include "Serial.h"
#include "app_kin.h"
#include "app_tower.h"
#include "tjc_screen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 *  上位机(natsiie_gcs/ROS) <-> STM32 串口通信
 *  帧: 0xFF | TYPE | LEN | PAYLOAD | CRC8 | 0xFE
 * ============================================================ */

uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN];   /* 待发送数据包 (8 字节) */

/* 接收帧 + 解析结果: ISR 写 Serial_RxFrame 并置 Serial_RxFrameReady,
 * 任务里调 Serial_ProcessFrame() 消费并按 type 解析到 Serial_RxData。 */
volatile uint8_t Serial_RxFrameReady = 0;
Serial_RxFrame_t  Serial_RxFrame;
Serial_RxData_t   Serial_RxData;

/* RX 状态机: 0=等帧头 1=收TYPE 2=收LEN 3=收载荷 4=收CRC 5=收帧尾 */
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

/**
  * @brief  初始化 USART1：PA9(TX)/PA10(RX)，115200，8N1，RXNE 接收中断
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

    /* NVIC_PriorityGroupConfig 由工程统一设置 (FreeRTOS Group4), 这里不再调 */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;   /* 低优先级，别抢 FreeRTOS */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  阻塞发送 1 字节
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

/* float -> 定点字符串 (最多 1 位小数), 不依赖 %f (MicroLib) */
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

/* 发送云台舵机角度给上位机 (0x07 GimbalYaw) */
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

/* 发送一帧数据包: 0xFF + 8 字节 + 0xFE (旧接口, 保留) */
void Serial_SendPacket(void)
{
    Serial_SendByte(SERIAL_HEAD);
    Serial_SendArray(Serial_TxPacket, SERIAL_TX_DATA_LEN);
    Serial_SendByte(SERIAL_TAIL);
}

/* ---- 手动数值解析 (MicroLib 无 sscanf %f) ---- */

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
  * @brief  按 type 解析 Serial_RxFrame, 结果写 Serial_RxData。
  *         0x08 点位解析成功后自动触发 kin_cmd=1 (GotoXY)。
  *         在任务里调用 (每 10ms, 收到帧后)。
  */
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
    case SERIAL_TYPE_STACKED_RING:   /* 0x05: 同 0x04 */
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
        }
        break;
    }

    case SERIAL_TYPE_QR_DECODE:   /* 0x06: 任务码字符串 -> 解析 4 组显示到屏 t0/t2 */
    {
        uint8_t i;
        for (i = 0; i < n; i++) Serial_RxData.str[i] = (char)Serial_RxFrame.payload[i];
        Serial_RxData.str[n] = '\0';
        Screen_ShowQRCode(Serial_RxData.str);   /* t0=前两组 t2=后两组, 覆盖占位 */
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

            /* 只存坐标, 不动: 上层只给 (x,y), z 由 kin_grab_z_mm 定死; 等 Watch 写 kin_cmd=1 才走 */
            kin_target_x = x;
            kin_target_y = y;
        }
        break;
    }

    case SERIAL_TYPE_GRAB_CMD:   /* 0x09: "1"=授权下降抓取 (夹爪到位) */
    {
        int v = 0;
        if (Serial_ParseIntTok(&p, &v) && v != 0)
        {
            Serial_RxData.grab_cmd = 1;
            grab_cmd = 1;   /* 装货流程: 电机5 从550降到447 下探抓取 */
        }
        break;
    }

    default:
        break;
    }
}

/**
  * @brief  USART1 接收中断: 帧结构状态机
  *         0=等帧头 1=TYPE 2=LEN 3=载荷 4=CRC 5=帧尾
  */
void USART1_IRQHandler(void)
{
    uint8_t d;

    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        volatile uint32_t tmp = USART1->SR; (void)tmp;
        tmp = USART1->DR; (void)tmp;
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        d = USART_ReceiveData(USART1);

        switch (s_rx_state)
        {
        case 0:   /* 等帧头 */
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
            s_rx_state = (d == 0) ? 4 : 3;   /* 无载荷直接等 CRC */
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
            else s_rx_state = 0;   /* CRC 错, 重新同步 */
            break;

        case 5:   /* 帧尾 */
            if (d == SERIAL_TAIL) Serial_RxFrameReady = 1;
            s_rx_state = 0;
            break;

        default:
            s_rx_state = 0;
            break;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
