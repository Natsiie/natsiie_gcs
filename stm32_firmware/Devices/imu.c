#include "stm32f4xx.h"
#include "wit_c_sdk.h"
#include "delay.h"
#include "imu.h"

/**
  * @brief  维特智能 JY901 陀螺仪驱动 (UART, 自带姿态融合)
  *         注意: Imu_Init 内部用 delay_ms 自动扫描波特率,
  *         必须在 FreeRTOS 调度器启动前调用。
  */

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void Usart2Init(uint32_t uiBaud);
static void AutoScanSensor(void);

static const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static volatile uint8_t s_cDataUpdate = 0;

void Imu_Init(void)
{
    Usart2Init(115200);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
    AutoScanSensor();

    /* 让陀螺仪连续回传数据, 否则它只在上电/被查询时回一帧, yaw 不会随转动刷新 */
    delay_ms(100);
    WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG);  /* 回传内容: 加速度+角速度+角度+磁场 */
    delay_ms(50);
    WitSetOutputRate(RRATE_100HZ);                            /* 100Hz 连续回传 */
    delay_ms(50);
}

float Imu_GetYaw(void)
{
    return sReg[Yaw] / 32768.0f * 180.0f;
}

float Imu_GetYawRate(void)
{
    /* 陀螺仪 Z 轴角速度 (Gz): 16位有符号, 默认量程 ±2000°/s -> °/s */
    return sReg[GZ] / 32768.0f * 2000.0f;
}

static void Usart2Init(uint32_t uiBaud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PD5 - TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* PD6 - RX */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    USART_InitStructure.USART_BaudRate            = uiBaud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t ucTemp = (uint8_t)USART_ReceiveData(USART2);
        WitSerialDataIn(ucTemp);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    uint32_t i;
    for (i = 0; i < uiSize; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, p_data[i]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

static void Delayms(uint16_t ucMs)
{
    delay_ms(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    (void)uiReg;
    (void)uiRegNum;
    s_cDataUpdate = 1;
}

static void AutoScanSensor(void)
{
    int i, iRetry;

    for (i = 1; i < 10; i++)
    {
        Usart2Init(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            delay_ms(100);
            if (s_cDataUpdate != 0)
                return;   /* 找到传感器, 保持当前波特率 */
            iRetry--;
        } while (iRetry);
    }
}
