#include "servo.h"
#include "car_config.h"

/**
  * @brief  舵机驱动: TIM2 CH2/CH3/CH4 = PA1/PA2/PA3, 50 Hz
  *         CCR = angle / range * 2000 + 500
  */
static float servo_cur[3] = {0.0f, 0.0f, 0.0f};  /* 各舵机最近一次设定角 (云台无回读, 运动学从这读) */

void Servo_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2); /* PA1 -> CH2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2); /* PA2 -> CH3 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2); /* PA3 -> CH4 */

    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = 20000 - 1;  /* 50 Hz */
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 84 - 1;     /* 1 MHz */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

void Servo_SetAngle(Servo_ID_t id, float angle)
{
    float    range;
    uint16_t ccr;

    switch (id)
    {
        case SERVO_GRIPPER:  range = SERVO_GRIPPER_RANGE;  break;
        case SERVO_PLATFORM: range = SERVO_PLATFORM_RANGE; break;
        case SERVO_GIMBAL:   range = SERVO_GIMBAL_RANGE;   break;
        default:             return;
    }

    if (angle < 0.0f)  angle = 0.0f;
    if (angle > range) angle = range;

    servo_cur[id] = angle;

    ccr = (uint16_t)(angle / range * 2000.0f + 500.0f);

    switch (id)
    {
        case SERVO_GRIPPER:  TIM_SetCompare2(TIM2, ccr); break;
        case SERVO_PLATFORM: TIM_SetCompare3(TIM2, ccr); break;
        case SERVO_GIMBAL:   TIM_SetCompare4(TIM2, ccr); break;
        default: break;
    }
}

float Servo_GetAngle(Servo_ID_t id)
{
    if (id > SERVO_GIMBAL) return 0.0f;
    return servo_cur[id];
}
