#ifndef TEST_STM32F4XX_H
#define TEST_STM32F4XX_H
#include <stdint.h>
typedef struct { int GPIO_Pin, GPIO_Mode, GPIO_Speed, GPIO_OType, GPIO_PuPd; } GPIO_InitTypeDef;
typedef struct { int USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity,
    USART_HardwareFlowControl, USART_Mode; } USART_InitTypeDef;
typedef struct { int NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority,
    NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
typedef struct { int SR, DR; } TestUSART;
extern TestUSART test_usart;
#define USART1 (&test_usart)
#define GPIOA 0
#define RCC_APB2Periph_USART1 1
#define RCC_AHB1Periph_GPIOA 1
#define ENABLE 1
#define RESET 0
#define SET 1
#define GPIO_PinSource9 9
#define GPIO_PinSource10 10
#define GPIO_AF_USART1 1
#define GPIO_Pin_9 9
#define GPIO_Pin_10 10
#define GPIO_Mode_AF 1
#define GPIO_Speed_50MHz 1
#define GPIO_OType_PP 1
#define GPIO_PuPd_UP 1
#define USART_WordLength_8b 1
#define USART_StopBits_1 1
#define USART_Parity_No 1
#define USART_HardwareFlowControl_None 1
#define USART_Mode_Rx 1
#define USART_Mode_Tx 2
#define USART_IT_RXNE 1
#define USART_FLAG_TXE 1
#define USART_FLAG_ORE 2
#define USART1_IRQn 1
#define RCC_APB2PeriphClockCmd(a,b) ((void)0)
#define RCC_AHB1PeriphClockCmd(a,b) ((void)0)
#define GPIO_PinAFConfig(a,b,c) ((void)0)
#define GPIO_Init(a,b) ((void)0)
#define USART_Init(a,b) ((void)0)
#define USART_ITConfig(a,b,c) ((void)0)
#define NVIC_Init(a) ((void)0)
#define USART_Cmd(a,b) ((void)0)
#define USART_ClearITPendingBit(a,b) ((void)0)
#define USART_GetFlagStatus(a,b) ((b) == USART_FLAG_TXE)
#define USART_GetITStatus(a,b) 0
#define USART_ReceiveData(a) 0
void test_send_byte(uint8_t byte);
#define USART_SendData(a,b) test_send_byte(b)
#endif
