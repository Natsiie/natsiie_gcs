#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "math_utils.h" // 引入你的究极数学库

/* --- 进阶版 PID 结构体 --- */
typedef struct {
    // 基础参数
    float Kp;
    float Ki;
    float Kd;
    float max_out;      // 输出限幅
    float max_i;        // 积分限幅
    
    // 高级优化参数
    float deadzone;     // 误差死区 (低于此误差则不响应，防齿轮抖动)
    float i_band;       // 积分分离阈值 (误差大于此值时停止积分，防大步长超调)
    Math_LPF_t d_filter;// ?? 升级：直接内嵌新版的低通滤波器结构体，处理D项高频噪声
    float feedforward;  // 前馈偏置 (用于机械臂抗重力悬停，可外部动态修改)
	
    uint8_t is_angle_loop; // 1: 启用最短角度路径规划(舵轮转向专用), 0: 普通线性模式
    
    // 动态状态
    float err;
    float last_actual;  // 用于微分先行，记录上一次的实际值
    float sum_err;
    float out;
} PID_Controller_t;

/* --- 串级 PID 结构体 --- */
typedef struct {
    PID_Controller_t pos_loop;  // 位置外环
    PID_Controller_t spd_loop;  // 速度内环
} CascadePID_t;

/* --- 核心算法接口 --- */
void PID_Init(PID_Controller_t *pid, float p, float i, float d, float max_out, float max_i, 
              float deadzone, float i_band, float d_lpf_alpha, uint8_t is_angle_loop);

float PID_Calc(PID_Controller_t *pid, float target, float actual);
float CascadePID_Calc(CascadePID_t *cascade, float target_pos, float actual_pos, float actual_spd);
void  PID_Clear(PID_Controller_t *pid);

#endif
