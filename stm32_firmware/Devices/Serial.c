#include "stm32f4xx.h"
#include "Serial.h"

/* ============================================================
 *  上位机 <-> STM32 串口通信（复刻自 yyb_stm32 / SYSTEM/Serial.c）
 *  帧：0xFF 帧头 + 数据 + 0xFE 帧尾
 * ============================================================ */

uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN];   /* 待发送数据（8 字节） */
uint8_t Serial_RxPacket[SERIAL_RX_DATA_LEN];   /* 接收数据（8 字节）   */
uint8_t Serial_RxFlag;                         /* 完整帧标志位         */

static uint8_t RxState = 0;  /* 接收状态机：0=等帧头 1=收数据 2=等帧尾 */

/**
  * @brief  初始化 USART1：PA9(TX)/PA10(RX)，115200，8N1，RXNE 接收中断
  */
void Serial_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* 开时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);

    /* PA9 = USART1_TX, PA10 = USART1_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 配置 */
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* 开 RXNE 接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* NVIC 配置。注意：NVIC_PriorityGroupConfig 由工程统一设置
       （FreeRTOS 用 Group4），这里不要再调，否则会打乱 FreeRTOS 优先级 */
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

/**
  * @brief  阻塞发送一个数组
  */
void Serial_SendArray(uint8_t *array, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
        Serial_SendByte(array[i]);
}

/**
  * @brief  阻塞发送一个字符串（到 '\0' 为止）
  */
void Serial_SendString(char *str)
{
    uint8_t i;
    for (i = 0; str[i] != '\0'; i++)
        Serial_SendByte((uint8_t)str[i]);
}

/**
  * @brief  float -> 定点字符串 (最多 1 位小数, 整数时省略小数点)
  *         不依赖 printf 的 %f (MicroLib 下 %f 可能不生效), 返回字符串长度
  */
static uint8_t Serial_FormatAngle(float angle, char *buf)
{
    int32_t scaled = (int32_t)(angle * 10.0f + (angle >= 0.0f ? 0.5f : -0.5f));
    uint8_t pos = 0, neg = 0;
    char    tmp[8];
    uint8_t t = 0;
    int32_t ip, fp;

    if (scaled < 0) { neg = 1; scaled = -scaled; }

    ip = scaled / 10;   /* 整数部分 */
    fp = scaled % 10;   /* 小数部分 (0~9) */

    /* 整数部分逆序拆位 (do-while 保证 0 也输出 "0") */
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

/**
  * @brief  发送云台舵机角度给上位机 (natsiie GCS 协议)
  *         帧: 0xFF | 0x07(GimbalYaw) | LEN | ASCII角度(度) | CRC8 | 0xFE
  *         CRC8 = TYPE ^ LEN ^ PAYLOAD 逐字节异或 (与上位机 protocol.py 一致)
  *         上位机 serial_bridge 收到后发布 /gimbal/joint_states
  */
void Serial_SendGimbalYaw(float angle_deg)
{
    char    payload[16];
    uint8_t len, crc, i;

    len = Serial_FormatAngle(angle_deg, payload);

    crc = SERIAL_TYPE_GIMBAL_YAW ^ len;
    for (i = 0; i < len; i++)
        crc ^= (uint8_t)payload[i];

    Serial_SendByte(0xFF);                       /* 帧头 */
    Serial_SendByte(SERIAL_TYPE_GIMBAL_YAW);     /* TYPE = GimbalYaw */
    Serial_SendByte(len);                        /* LEN */
    Serial_SendArray((uint8_t *)payload, len);   /* PAYLOAD: ASCII 角度 */
    Serial_SendByte(crc);                        /* CRC8 */
    Serial_SendByte(0xFE);                       /* 帧尾 */
}

/**
  * @brief  发送一帧数据包：0xFF + Serial_TxPacket(8字节) + 0xFE
  */
void Serial_SendPacket(void)
{
    Serial_SendByte(0xFF);
    Serial_SendArray(Serial_TxPacket, SERIAL_TX_DATA_LEN);
    Serial_SendByte(0xFE);
}

/**
  * @brief  读取接收标志位（收到完整一帧后返回 1，读完自动清 0）
  */
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

/**
  * @brief  手动复位接收状态机（出错重同步用，可选）
  */
void Serial_SetRxState(uint8_t state)
{
    RxState = state;
}

/**
  * @brief  USART1 接收中断：三状态状态机
  *         状态0 等帧头 0xFF -> 状态1
  *         状态1 收满 8 字节数据 -> 状态2
  *         状态2 等帧尾 0xFE -> 置 Serial_RxFlag，回状态0
  */
void USART1_IRQHandler(void)
{
    static uint8_t pRxPacket = 0;   /* 当前收到的数据字节序号 */

    /* 过载错误(ORE)处理: 读 SR+DR 清标志, 防 115200 下过载后接收卡死 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
    {
        volatile uint32_t tmp = USART1->SR; (void)tmp;
        tmp = USART1->DR; (void)tmp;
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

        if (RxState == 0)                    /* 等帧头 */
        {
            if (RxData == 0xFF)
            {
                RxState    = 1;
                pRxPacket  = 0;
            }
        }
        else if (RxState == 1)               /* 收数据 */
        {
            Serial_RxPacket[pRxPacket] = RxData;
            pRxPacket++;
            if (pRxPacket >= SERIAL_RX_DATA_LEN)
            {
                RxState = 2;
            }
        }
        else if (RxState == 2)               /* 等帧尾 */
        {
            if (RxData == 0xFE)
            {
                RxState      = 0;
                Serial_RxFlag = 1;           /* 成功接收完整一帧 */
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
