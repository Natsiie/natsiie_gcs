#ifndef __INTERRUPT_PRIORITY_H
#define __INTERRUPT_PRIORITY_H

/* ======================================================================== */
/* 全局中断优先级配置总表                                                   */
/* 优先级分组：NVIC_PriorityGroup_2 (2位抢占优先级，2位响应优先级)          */
/* 抢占优先级范围: 0~3, 响应优先级范围: 0~3                                 */
/* ======================================================================== */

/* --- DMA 中断 --- */
//#define DMA1_STREAM1_PRE_PRIO    0  // 抢占优先级最高
//#define DMA1_STREAM1_SUB_PRIO    1

/* --- CAN 中断 --- */
#define CAN1_RX0_PRE_PRIO        1
#define CAN1_RX0_SUB_PRIO        0

#define CAN2_RX1_PRE_PRIO        1
#define CAN2_RX1_SUB_PRIO        2

/* --- USART 中断 --- */
//#define USART3_PRE_PRIO          1
//#define USART3_SUB_PRIO          1

//#define USART1_PRE_PRIO          1
//#define USART1_SUB_PRIO          2

// 如果后续需要加 TIM 中断，直接在这里追加宏定义即可
// #define TIM2_PRE_PRIO         1
// #define TIM2_SUB_PRIO         2

#endif /* __INTERRUPT_PRIORITY_H */
