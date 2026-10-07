#include "PID.h"

// 初始化 
void PID_Init(PID_Controller_t *pid, float p, float i, float d, float max_out, float max_i, 
              float deadzone, float i_band, float d_lpf_alpha, uint8_t is_angle_loop) 
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_i = max_i;
    
    pid->deadzone = deadzone;
    pid->i_band = i_band;
    
    // ?? 升级：调用新版数学库，初始化内部的D项低通滤波器
    Math_LPF_Init(&pid->d_filter, d_lpf_alpha); 
    
    pid->feedforward = 0.0f; 
    pid->is_angle_loop = is_angle_loop;
    
    PID_Clear(pid);
}

// 状态清空 
void PID_Clear(PID_Controller_t *pid) 
{
    pid->err = 0.0f;
    pid->last_actual = 0.0f;
    pid->sum_err = 0.0f;
    pid->out = 0.0f;
    pid->d_filter.out = 0.0f; // 同步清空滤波器的历史状态
}

// 单环 PID 核心计算
float PID_Calc(PID_Controller_t *pid, float target, float actual) 
{
    // 误差计算与角度规约
    pid->err = target - actual;
    if (pid->is_angle_loop == 1) {
        pid->err = Math_NormalizeAngle(pid->err); 
    }
    
    // 误差死区过滤
    if (fabsf(pid->err) < pid->deadzone) {
        pid->err = 0.0f;
    }
    
    // P项计算
    float p_term = pid->Kp * pid->err;
    
    // 动态抗积分饱和 (Clamping Anti-Windup)
    if (fabsf(pid->err) < pid->i_band) 
    {
        // 核心逻辑：如果输出已经满载，且误差的方向还在试图增加输出，则冻结积分！
        if ((pid->out >= pid->max_out && pid->err > 0) || 
            (pid->out <= -pid->max_out && pid->err < 0)) 
        {
            // 积分冻结，什么都不做
        } 
        else 
        {
            pid->sum_err += pid->err;
            pid->sum_err = Math_Limit(pid->sum_err, -pid->max_i, pid->max_i); 
        }
    }
    float i_term = pid->Ki * pid->sum_err;
    
    // 微分先行 (规避目标值突变的剧烈抖动)
    float delta_actual = actual - pid->last_actual;
    if (pid->is_angle_loop == 1) {
        delta_actual = Math_NormalizeAngle(delta_actual);
    }
    float d_raw = -pid->Kd * delta_actual; 
    
    // 使用面向对象的方法计算滤波后的 D 项
    float d_term = Math_LPF_Calc(&pid->d_filter, d_raw);
    pid->last_actual = actual;    
    
    // 总输出计算及限幅
    pid->out = p_term + i_term + d_term + pid->feedforward;
    pid->out = Math_Limit(pid->out, -pid->max_out, pid->max_out); 
    
    return pid->out;
}

// 串级 PID 计算封装
float CascadePID_Calc(CascadePID_t *cascade, float target_pos, float actual_pos, float actual_spd) 
{
    // 先算外环 (位置环)，其输出作为内环的目标速度
    float target_spd = PID_Calc(&cascade->pos_loop, target_pos, actual_pos);
    
    // 再算内环 (速度环)，其输出为最终的电流/力矩
    float final_out = PID_Calc(&cascade->spd_loop, target_spd, actual_spd);
    
    return final_out;
}
