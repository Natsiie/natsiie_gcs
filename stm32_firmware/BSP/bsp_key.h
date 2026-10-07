#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "stm32f4xx.h"

/**
  * @brief  一键启动开关 (PC9)
  *         默认接法: 开关一端接 PC9, 另一端接 GND, 内部上拉; 接通 = 低电平
  */
#define KEY_GPIO_PORT    GPIOC
#define KEY_GPIO_PIN     GPIO_Pin_9

#define KEY_ACTIVE_LOW   1   /* 1=低电平有效(接GND, 上拉); 若开关接3.3V改成 0 */

void    Key_Init(void);       /* 配置 PC9 为带上下拉的输入 */
uint8_t Key_Read(void);       /* 返回 1=按下/接通, 0=松开/断开 */

#endif /* __BSP_KEY_H */
