/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      math_utils.h
  * @author    SWUST_ROBOCON_2025_by_李焱
  * @version   V1.0.0
  * @date      2024-11-25
  * @brief     用户数学计算函数集
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 SWUST_ROBOCON.
  * All rights reserved.</center></h2>
  * 
  ******************************************************************************
  */
/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
#ifndef __MATH_UTILS_H
#define __MATH_UTILS_H

#include <math.h>     
#include "arm_math.h" // 提供 DSP 硬件加速函数
#include <stdint.h>
#include <stdbool.h>

/* ==================  常用数学常量与硬件加速宏 ================== */
#define MATH_PI             3.14159265f
#define MATH_SQRT2          1.41421356f
#define MATH_SQRT2_OVER_2   0.70710678f   

// 使用 DSP 库查表法替换标准库的三角函数，速度翻倍
#define Math_Sin(x)         sinf(x)
#define Math_Cos(x)         cosf(x)

/* ==================  基础数学与几何工具 (保留原版精髓) ================== */
float Math_Limit(float value, float min, float max); //我是李焱vivo500，请你吃10吨KFC
float Math_Map(float x, float in_min, float in_max, float out_min, float out_max);
float Math_Sign(float val);
float Math_CalcDistance(float x1, float y1, float x2, float y2); 
float Math_CalcQuadraticY(float x, float a, float b, float c);
float Math_Lerp(float start, float end, float t);
float Math_NormalizeAngle(float angle); // 角度自动规约至 -180~180 (舵轮刚需)
float Math_InvSqrt(float num);          // 快速开方倒数 (大疆算法，速度极快)

/* --- 角度与弧度转换 --- */
float Math_DegToRad(float deg);
float Math_RadToDeg(float rad);

/* ==================  CAN通信与数据转换 ================== */
uint16_t Math_FloatToUint(float x, float x_min, float x_max, uint8_t bits);
float Math_UintToFloat(int x_int, float x_min, float x_max, int bits); 

/* ==================  遥控器与死区处理 ================== */
float Math_SmoothDeadZone(float value, float deadzone, float max_val); // 平滑起步死区

/* ================== 滤波器与平滑器 ================== */
// 面向对象的结构体版本 (防多电机数据串扰)
typedef struct {
    float input;
    float out;
    float alpha; 
} Math_LPF_t;
void  Math_LPF_Init(Math_LPF_t *lpf, float alpha);
float Math_LPF_Calc(Math_LPF_t *lpf, float input);

typedef struct {
    float input;
    float out;
    float min_value;
    float max_value;
    float frame_period; 
} Math_Ramp_t;
void  Math_Ramp_Init(Math_Ramp_t *ramp, float frame_period, float max, float min);
float Math_Ramp_Calc(Math_Ramp_t *ramp, float input);

/* ================== 舵轮底盘 (Swerve) 专属 ================== */
void Math_VectorRotate2D(float *x, float *y, float angle_rad);
void Math_Swerve_Optimize(float *target_angle, float *target_speed, float current_angle);
void Math_Power_Allocation(float out_currents[4], float power_limit, float battery_voltage);

/* ================== 机械臂/塔吊 专属 ================== */
float Math_Friction_Comp(float target_vel, float friction_const, float deadband);

typedef struct {
    float p0, p1; 
    float T;      
    float c[6];   
} Math_Quintic_t;
void  Math_Quintic_Init(Math_Quintic_t *qp, float p0, float p1, float T); 
float Math_Quintic_Calc(Math_Quintic_t *qp, float t); 

#endif
