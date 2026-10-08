/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/
/**
  ******************************************************************************
  * @file      bsp_usart.c
  * @author    SWUST_ROBOCON_2026_by_����
  * @brief     ����3����ʵ�֣�����DMA�����߼�
  ******************************************************************************
  */
#include "bsp_usart.h"
#include "crsf_protocol.h"  // ����Э���ͷ�ļ������ڽ�������

/* ���ջ����� */
uint8_t usart3_rx_buf[128];
uint8_t usart5_rx_buf[128];
uint8_t usart6_rx_buf[128];

Chassis_Feedback_t1 Chassis_Feedback1;
Usart6_Feedback_t  Usart6_Feedback;

/* USART6 ���ͻ����� (ȫ��) */
Usart6_Protocol_t usart6_tx_packet = {
    .header1 = 0x55,
    .header2 = 0xAA,
    .flag    = 0,
    .crc8    = 0
};


/**
  * @brief  �ڲ���̬�������������� DMA ���ղ�������־λ
  */
static uint16_t Restart_DMA_Rx(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG_Mask)
{
    DMA_Cmd(DMAy_Streamx, DISABLE);
    while (DMA_GetCmdStatus(DMAy_Streamx) != DISABLE);
    
    uint16_t rx_len = 128 - DMA_GetCurrDataCounter(DMAy_Streamx);
    
    DMA_ClearFlag(DMAy_Streamx, DMA_FLAG_Mask);
    DMA_SetCurrDataCounter(DMAy_Streamx, 128);
    DMA_Cmd(DMAy_Streamx, ENABLE);
    
    return rx_len;
}

/**
  * @brief  ͳһ��ʼ��
  */
void BSP_USART_Init(BSP_UART_ID_e uart_id, uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_TypeDef* USARTx = NULL;
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    switch (uart_id)
    {
        case UART_3:
            USARTx = USART3;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            break;
        case UART_5:
            USARTx = UART5;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);   /* PC12 = UART5_TX */
            GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);   /* PD2  = UART5_RX */
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;
        case UART_6:
            USARTx = USART6;
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
            GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            break;
    }

    USART_InitStructure.USART_BaudRate   = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    if (uart_id == UART_3)
    {
        DMA_DeInit(DMA1_Stream3);
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_Memory0BaseAddr = 0;
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_Init(DMA1_Stream3, &DMA_InitStructure);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

        DMA_DeInit(DMA1_Stream1);
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart3_rx_buf;
        DMA_InitStructure.DMA_BufferSize = 128;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_Init(DMA1_Stream1, &DMA_InitStructure);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    }
    else if (uart_id == UART_5)
    {
        DMA_DeInit(DMA1_Stream7);
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_Memory0BaseAddr = 0;
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_Init(DMA1_Stream7, &DMA_InitStructure);
        USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);

        DMA_DeInit(DMA1_Stream0);
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart5_rx_buf;
        DMA_InitStructure.DMA_BufferSize = 128;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_Init(DMA1_Stream0, &DMA_InitStructure);
        USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA1_Stream0, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    }
    else if (uart_id == UART_6)
    {
        DMA_DeInit(DMA2_Stream6);
        DMA_InitStructure.DMA_Channel = DMA_Channel_5;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_Memory0BaseAddr = 0;
        DMA_InitStructure.DMA_BufferSize = 0;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_Init(DMA2_Stream6, &DMA_InitStructure);
        USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

        DMA_DeInit(DMA2_Stream1);
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6_rx_buf;
        DMA_InitStructure.DMA_BufferSize = 128;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_Init(DMA2_Stream1, &DMA_InitStructure);
        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA2_Stream1, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    }

    volatile uint32_t temp = USARTx->DR; (void)temp;
    USART_ClearITPendingBit(USARTx, USART_IT_RXNE);

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE); 
    USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);
    USART_Cmd(USARTx, ENABLE);
}

/**
  * @brief  ͳһ�� DMA ���ͺ���
  */
void BSP_USART_DMA_Send(BSP_UART_ID_e uart_id, uint8_t *data, uint16_t len)
{
    DMA_Stream_TypeDef* DMAy_Streamx = NULL;
    uint32_t ALL_FLAGS = 0; // ����һ����������б�־λ

    if (uart_id == UART_3) { 
        DMAy_Streamx = DMA1_Stream3; 
        ALL_FLAGS = DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3; 
    }
    else if (uart_id == UART_5) {
        DMAy_Streamx = DMA1_Stream7;
        ALL_FLAGS = DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7;
    }
    else if (uart_id == UART_6) {
        DMAy_Streamx = DMA2_Stream6;
        ALL_FLAGS = DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6;
    }
    else return;

    // ��� DMA ����æ��ֱ�ӷ��� (����ϲ� app_chassis_comm.c �ı���)
    if(DMA_GetCmdStatus(DMAy_Streamx) != DISABLE) return;
    
    // �������������������ر�־λ����ֹ TE �����µ�����������
    DMA_ClearFlag(DMAy_Streamx, ALL_FLAGS);      
    
    DMAy_Streamx->M0AR = (uint32_t)data;              
    DMAy_Streamx->NDTR = len;                         
    DMA_Cmd(DMAy_Streamx, ENABLE);                    
}

/**
  * @brief  USART6 ��־λ���� (�Զ��������� CRC8 ����)
  */
void USART6_Send_Command(uint8_t flag)
{
    uint32_t timeout = 5000; 
    while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE && timeout--) {
        if (timeout == 0) return; 
    }

    usart6_tx_packet.flag = flag;
    // ʹ�� crsf_protocol.c �Ĳ����������У�� Payload (����2�ֽ�֡ͷ)
    usart6_tx_packet.crc8 = CRC8_Calc(&usart6_tx_packet.flag, sizeof(Usart6_Protocol_t) - 3);
    
    BSP_USART_DMA_Send(UART_6, (uint8_t *)&usart6_tx_packet, sizeof(Usart6_Protocol_t));
}

void USART6_Send_VOFA_JustFloat1(void)
{
    // ���徲̬���棬���ڳ�ʼ��ʱ���� JustFloat �ı�׼֡β
    static VOFA_JustFloat_t1 vofa_packet = {
        .tail = {0x00, 0x00, 0x80, 0x7F}
    };
    
    // ��� DMA ���ڰ�����һ֡���ݣ�ֱ�ӷ��أ�������ǰ����֤ʵʱ��
    if (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE) {
        return; 
    }
    
    // �������µĵ��̷�������
    vofa_packet.err_yaw = Chassis_Feedback1.err_yaw;
    vofa_packet.yaw_out = Chassis_Feedback1.yaw_out;
    vofa_packet.yaw     = Chassis_Feedback1.yaw;
    vofa_packet.aim_yaw = Chassis_Feedback1.aim_yaw;
     
    // �����ֳɵ�ͨ�� DMA ���ͽӿ� (���ͳ���Ϊ 16�ֽ����� + 4�ֽ�֡β = 20�ֽ�)
    BSP_USART_DMA_Send(UART_6, (uint8_t *)&vofa_packet, sizeof(VOFA_JustFloat_t1));
}


// ======================= �жϷ����� =======================


void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t temp = USART3->SR; temp = USART3->DR; (void)temp;
        uint16_t rx_len = Restart_DMA_Rx(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1);

        for(uint16_t i = 0; i < rx_len; i++)
        {
            CRSF_ProcessByte(usart3_rx_buf[i]);
        }
    }
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) { volatile uint32_t temp = USART3->SR; temp = USART3->DR; (void)temp; }
}

void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t temp = UART5->SR; temp = UART5->DR; (void)temp;
        uint16_t rx_len = Restart_DMA_Rx(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0);

        for(uint16_t i = 0; i < rx_len; i++)
        {
            CRSF_ProcessByte(usart5_rx_buf[i]);
        }
    }
    if(USART_GetFlagStatus(UART5, USART_FLAG_ORE) != RESET) { volatile uint32_t temp = UART5->SR; temp = UART5->DR; (void)temp; }
}

void USART6_IRQHandler(void)
{
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t temp = USART6->SR; temp = USART6->DR; (void)temp;
        uint16_t rx_len = Restart_DMA_Rx(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1);

        if (rx_len >= sizeof(Usart6_Protocol_t))
        {
            for (int i = 0; i <= rx_len - sizeof(Usart6_Protocol_t); i++)
            {
                if (usart6_rx_buf[i] == 0x55 && usart6_rx_buf[i+1] == 0xAA)
                {
                    Usart6_Protocol_t *rx_p = (Usart6_Protocol_t *)&usart6_rx_buf[i];
                    uint8_t p_len = sizeof(Usart6_Protocol_t) - 3; 
                    
                    if (CRC8_Calc(&rx_p->flag, p_len) == rx_p->crc8)
                    {
                        Usart6_Feedback.flag = rx_p->flag;
                        Usart6_Feedback.online = 1; 
                        Usart6_Feedback.last_update_tick = xTaskGetTickCountFromISR();
                        break; 
                    }
                }
            }
        }
    }
    if(USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET) { volatile uint32_t temp = USART6->SR; temp = USART6->DR; (void)temp; }
}
