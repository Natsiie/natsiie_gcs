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

void CAN1_Init(void);
void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

#endif /* __BSP_CAN_H */
