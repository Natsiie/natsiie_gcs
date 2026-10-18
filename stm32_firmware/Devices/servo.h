#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"

typedef enum {
    SERVO_GRIPPER = 0,   /* PA1 / TIM2_CH2 夹爪 */
    SERVO_PLATFORM,      /* PA2 / TIM2_CH3 物料平台 */
    SERVO_GIMBAL         /* PA3 / TIM2_CH4 云台 */
} Servo_ID_t;

void Servo_Init(void);
void Servo_SetAngle(Servo_ID_t id, float angle);          /* 立即到位 (低层接口, 受上电门控) */
void Servo_MoveTo(Servo_ID_t id, float target, float max_deg_per_tick);  /* 斜坡限速: 每10ms最多N度 */
void Servo_Tick(void);                                     /* 上电门控计数, 每10ms调一次 */
float Servo_GetAngle(Servo_ID_t id);   /* 返回最近一次设定的角度 (云台无回读, 供运动学用) */

#endif /* __SERVO_H */
