/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      math_utils.c
  * @author    SWUST_ROBOCON_2026_by_李焱
  * @version   V1.0.0
  * @date      2026-3-14
  * @brief     用户数学计算函数集
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 SWUST_ROBOCON.
  * All rights reserved.</center></h2>
  * 
  ******************************************************************************
  */
#include "math_utils.h"

/* ================== 基础数学与几何工具 ================== */
float Math_Limit(float value, float min, float max) {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

// 提取浮点数的符号 (保留了你的高精度 eps 保护)
float Math_Sign(float val) {
    if (val > 1e-6f) return 1.0f;
    if (val < -1e-6f) return -1.0f;
    return 0.0f; 
}

float Math_Map(float x, float in_min, float in_max, float out_min, float out_max) {
    return ((x - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min;
}

// 角度规约至 -180 到 180 (使用了安全的 fmodf)
float Math_NormalizeAngle(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
}

// 线性插值
float Math_Lerp(float start, float end, float t) {
    t = Math_Limit(t, 0.0f, 1.0f); 
    return start + t * (end - start);
}

// 计算二维空间两点距离
float Math_CalcDistance(float x1, float y1, float x2, float y2) {
    float dis;
    arm_sqrt_f32((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2), &dis);
    return dis;
}

// 二次函数求解
float Math_CalcQuadraticY(float x, float a, float b, float c) {
    return a * x * x + b * x + c;
}

// 雷神之锤快速平方根倒数 (大疆移植版，速度极快)
float Math_InvSqrt(float num) {
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1); 
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

// 角度与弧度互转
float Math_DegToRad(float angle) {
    return angle * MATH_PI / 180.0f; 
}
float Math_RadToDeg(float rad) {
    return rad / MATH_PI * 180.0f;
}


/* ==================  CAN通信与数据转换 ================== */
uint16_t Math_FloatToUint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Math_UintToFloat(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


/* ==================  遥控器与死区 ================== */
float Math_SmoothDeadZone(float value, float deadzone, float max_val) {
    if (fabsf(value) <= deadzone) return 0.0f;
    float sign = Math_Sign(value);
    return sign * ((fabsf(value) - deadzone) / (max_val - deadzone) * max_val);
}


/* ==================  滤波器与平滑器 ================== */
// 对象化 LPF
void Math_LPF_Init(Math_LPF_t *lpf, float alpha) {
    lpf->input = 0.0f; lpf->out = 0.0f;
    lpf->alpha = Math_Limit(alpha, 0.0f, 1.0f);
}
float Math_LPF_Calc(Math_LPF_t *lpf, float input) {
    lpf->out = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->out;
    return lpf->out;
}

// 对象化 Ramp
void Math_Ramp_Init(Math_Ramp_t *ramp, float frame_period, float max, float min) {
    ramp->frame_period = frame_period;
    ramp->max_value = max; ramp->min_value = min;
    ramp->input = 0.0f; ramp->out = 0.0f;
}
float Math_Ramp_Calc(Math_Ramp_t *ramp, float input) {
    ramp->out += input * ramp->frame_period; 
    ramp->out = Math_Limit(ramp->out, ramp->min_value, ramp->max_value);
    return ramp->out;
}


/* ================== 舵轮底盘 (Swerve) 专属 ================== */
void Math_VectorRotate2D(float *x, float *y, float angle_rad) {
    float cos_a = Math_Cos(angle_rad);
    float sin_a = Math_Sin(angle_rad);
    float new_x = (*x) * cos_a - (*y) * sin_a;
    float new_y = (*x) * sin_a + (*y) * cos_a;
    *x = new_x; *y = new_y;
}

void Math_Swerve_Optimize(float *target_angle, float *target_speed, float current_angle) {
    float delta = Math_NormalizeAngle(*target_angle - current_angle);
    if (delta > 90.0f) {
        *target_angle -= 180.0f;
        *target_speed = -(*target_speed);
    } else if (delta < -90.0f) {
        *target_angle += 180.0f;
        *target_speed = -(*target_speed);
    }
    *target_angle = Math_NormalizeAngle(*target_angle);
}

void Math_Power_Allocation(float out_currents[4], float power_limit, float battery_voltage) {
    float total_power = 0.0f;
    float k_i2p = 1.0f / 1000.0f; 
    for (int i = 0; i < 4; i++) {
        total_power += fabsf(out_currents[i]) * battery_voltage * k_i2p; 
    }
    float safe_power_limit = power_limit * 0.95f; 
    if (total_power > safe_power_limit) {
        float power_scale = safe_power_limit / total_power;
        for (int i = 0; i < 4; i++) out_currents[i] *= power_scale;
    }
}


/* ================== 机械臂/塔吊 专属 ================== */
float Math_Friction_Comp(float target_vel, float friction_const, float deadband) {
    if (fabsf(target_vel) <= deadband) return 0.0f;
    return Math_Sign(target_vel) * friction_const; 
}

void Math_Quintic_Init(Math_Quintic_t *qp, float p0, float p1, float T) {
    qp->p0 = p0; qp->p1 = p1; qp->T = T;
    
    float T3 = T * T * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    
    qp->c[0] = p0;
    qp->c[1] = 0.0f;
    qp->c[2] = 0.0f;
    qp->c[3] =  10.0f * (p1 - p0) / T3;
    qp->c[4] = -15.0f * (p1 - p0) / T4;
    qp->c[5] =   6.0f * (p1 - p0) / T5;
}

float Math_Quintic_Calc(Math_Quintic_t *qp, float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > qp->T) t = qp->T; 
    
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    
    return qp->c[0] + qp->c[3]*t3 + qp->c[4]*t4 + qp->c[5]*t5;
}
/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 
