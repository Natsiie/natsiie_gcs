#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"

typedef enum {
    SERVO_GRIPPER = 0,   /* PA1 / TIM2_CH2 夹爪 */
    SERVO_PLATFORM,      /* PA2 / TIM2_CH3 物料平台 */
    SERVO_GIMBAL         /* PA3 / TIM2_CH4 云台 */
} Servo_ID_t;

void Servo_Init(void);
void Servo_SetAngle(Servo_ID_t id, float angle);
float Servo_GetAngle(Servo_ID_t id);   /* 返回最近一次设定的角度 (云台无回读, 供运动学用) */

#endif /* __SERVO_H */
