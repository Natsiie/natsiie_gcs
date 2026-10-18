#include "servo.h"
#include "car_config.h"

/**
  * @brief  舵机驱动: TIM2 CH2/CH3/CH4 = PA1/PA2/PA3, 50 Hz
  *         CCR = angle / range * 2000 + 500
  */
static float servo_cur[3] = {0.0f, 0.0f, 0.0f};  /* 各舵机最近一次设定角 (云台无回读, 运动学从这读) */

/* 上电门控: Servo_Init 写停放位初值后, 挡 SERVO_ARM_TICKS 个 10ms 拍再放行任何舵机动作,
   避开舵机上电自检, 也给机械一个稳定的初始姿态 */
static volatile uint8_t servo_armed    = 0;
static uint16_t         servo_arm_tick = 0;

/* 角度 -> CCR (50Hz, 0.5~2.5ms 脉宽对应 0~range 度) */
static uint16_t servo_ccr(float angle, float range)
{
    if (range <= 0.0f) return 500;
    return (uint16_t)(angle / range * 2000.0f + 500.0f);
}

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

    /* 上电直接给"机械实际停放位"CCR: 第一条有效脉冲就是停放位, 不是 0 全速跳变 */
    TIM_OCInitStructure.TIM_Pulse = servo_ccr(GRIPPER_OPEN_ANGLE, SERVO_GRIPPER_RANGE);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = servo_ccr(PLATFORM_ANGLE_A, SERVO_PLATFORM_RANGE);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = servo_ccr(GIMBAL_PARK_ANGLE, SERVO_GIMBAL_RANGE);
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);

    /* servo_cur 一并初始化成停放角: 上电 Servo_GetAngle() 即真值 (不再误报 0 / -42°) */
    servo_cur[SERVO_GRIPPER]  = GRIPPER_OPEN_ANGLE;
    servo_cur[SERVO_PLATFORM] = PLATFORM_ANGLE_A;
    servo_cur[SERVO_GIMBAL]   = GIMBAL_PARK_ANGLE;

    servo_armed    = 0;
    servo_arm_tick = 0;
}

/* 上电门控计数: 每 10ms 调一次, 满 SERVO_ARM_TICKS 拍后放行舵机动作 */
void Servo_Tick(void)
{
    if (!servo_armed && ++servo_arm_tick >= SERVO_ARM_TICKS)
        servo_armed = 1;
}

void Servo_SetAngle(Servo_ID_t id, float angle)
{
    float    range;
    uint16_t ccr;

    if (!servo_armed) return;   /* 上电未就绪: 拒绝动作 */

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
    ccr = servo_ccr(angle, range);

    switch (id)
    {
        case SERVO_GRIPPER:  TIM_SetCompare2(TIM2, ccr); break;
        case SERVO_PLATFORM: TIM_SetCompare3(TIM2, ccr); break;
        case SERVO_GIMBAL:   TIM_SetCompare4(TIM2, ccr); break;
        default: break;
    }
}

/* 斜坡限速到目标角: 每 10ms 最多变化 max_deg_per_tick 度。云台 0-360 连续, 抄近路跨 0/360 */
void Servo_MoveTo(Servo_ID_t id, float target, float max_deg_per_tick)
{
    float cur, d;

    if (id > SERVO_GIMBAL) return;

    cur = servo_cur[id];
    d = target - cur;

    if (id == SERVO_GIMBAL)
    {
        if (d >  180.0f) d -= 360.0f;
        if (d < -180.0f) d += 360.0f;
    }

    if (d >  max_deg_per_tick) d =  max_deg_per_tick;
    if (d < -max_deg_per_tick) d = -max_deg_per_tick;

    cur += d;

    if (id == SERVO_GIMBAL)
    {
        while (cur >= 360.0f) cur -= 360.0f;
        while (cur <   0.0f)  cur += 360.0f;
    }

    Servo_SetAngle(id, cur);
}

float Servo_GetAngle(Servo_ID_t id)
{
    if (id > SERVO_GIMBAL) return 0.0f;
    return servo_cur[id];
}
