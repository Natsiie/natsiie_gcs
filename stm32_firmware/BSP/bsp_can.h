#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdbool.h>
#include "stm32f4xx.h"

/**
  * @brief  ZDT X_V2 闭环步进电机 CAN 层 (500 kbps, 扩展帧)
  *         CAN1 @ PA11(RX) / PA12(TX)
  */

typedef struct {
    __IO CanRxMsg CAN_RxMsg;
    __IO CanTxMsg CAN_TxMsg;
    __IO bool   rxFrameFlag;
} CAN_t;

extern __IO CAN_t can;

/* EMM 定时主动上报位置快照表 (CAN 中断按电机地址填入, 任务侧只读):
   can_pos_raw[addr] = 32位位置计数 (65536=360度), can_pos_sign[addr] = 符号,
   can_pos_cnt[addr]  = 收到帧计数 (里程计据此判断"本周期有新数据")。
   索引 0 不用, 1~MOTOR_ADDR_MAX 对应电机地址; 中断里只存整数, 浮点换算放任务侧。 */
#define MOTOR_ADDR_MAX  6
extern __IO uint32_t can_pos_raw[7];
extern __IO uint8_t  can_pos_sign[7];
extern __IO uint16_t can_pos_cnt[7];
extern __IO uint32_t can_rx_drop;   /* 丢弃计数: 位置帧地址越界 */
extern __IO uint32_t can_tx_drop;   /* 丢弃计数: 发送邮箱超时放弃的帧 (Watch 看是否为 0) */
extern __IO uint32_t can_rx_ovr;    /* 丢弃计数: RX FIFO0 硬件溢出 (Watch 应恒 0; 非 0 = 中断被拖太久) */

/* 电机母线电压回包专用槽位: 与位置帧分流, 不再共用 rxFrameFlag, 消除撕裂读竞态。
   按电机地址索引, 中断里"先存数据、最后置 can_vbus_flag[addr]"。
   支持两种回包: 0x24 单读总线电压(Data[1..2]) / 0x43 批量读系统状态第0包(Data[3..4]),
   均为 16 位 mV 大端。索引 0 不用, 1~MOTOR_ADDR_MAX 对应电机地址 (底盘1 + 塔吊5 都走这里)。 */
extern __IO uint16_t can_vbus_mv[7];    /* 各电机地址电压原始值 (mV) */
extern __IO uint8_t  can_vbus_flag[7];  /* 1 = 收到该地址一帧新的 0x43 第0包 */
extern __IO uint32_t can_vbus_ok;       /* 成功接收计数 (验收用) */

void CAN1_Init(void);
void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

#endif /* __BSP_CAN_H */
