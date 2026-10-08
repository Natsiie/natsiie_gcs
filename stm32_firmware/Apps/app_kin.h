#ifndef __APP_KIN_H
#define __APP_KIN_H

#include "stm32f4xx.h"

/**
  * @brief  塔吊抓取机构正/逆运动学 + 点到点运动 (圆柱坐标: 云台θ + 伸出r + 升降z)
  *         塔吊系: 原点 = 云台转轴在地面的投影, +X = 前, +Y = 左, +Z = 上(地面起算)
  *         执行器单位: 云台 = 角度(度); 电机5(升降)/电机6(伸出) 对外一律用 mm。
  *
  *         上层只发 (x, y), z 由下位机定死 (kin_grab_z_mm)。
  *
  *         正解 Kin_FK:      (θ, r, z) -> (x, y, z)    供上层"铆钉"/回传当前位姿
  *         逆解 Kin_IK:      (x, y, z) -> (θ, r, z)    供需要完整 3D 目标时用
  *         运动 Kin_GotoXY:  (x, y) -> θ, r; z 固定; 按下面安全顺序走
  *
  *         安全运动顺序 (防止转云台时扫到东西):
  *           1) 电机6 -> kin_safe_extend_mm (先)
  *           2) 电机5 -> kin_safe_hoist_mm  (后)
  *           3) 两者都到位 -> 云台 -> 目标角
  *           4) 云台到位 -> 电机6 -> 目标 r (先)
  *           5) 电机6到位 -> 电机5 -> 抓取 z = kin_grab_z_mm (后)
  *
  *         注意: 云台是普通 PWM 舵机, 无位置回读; 当前角从 Servo_GetAngle(SERVO_GIMBAL)
  *         读(servo.c 记录最近一次设定角)。启用 Kin 时请把 Gimbal_Test_Task 关掉
  *         (gimbal_test_enable=0), 避免抢云台。
  */

/* ================= 标定参数 (Keil Watch 现场标定; 单位 mm / 度) ================= */

/* 云台: 物理方位角 θ(度, 0=+X, 逆时针为正) <-> 舵机角 */
extern float  kin_gimbal_theta_offset;   /* θ=0(+X) 时的舵机值 */
extern int8_t kin_gimbal_dir;            /* +1: 舵机值增大->θ增大; -1 反向 */
extern float  kin_gimbal_gain;           /* 舵机值 = offset + dir*gain*θ (斜率, 默认 93/90) */

/* 伸出(电机6): 水平距离 r(mm) <-> 电机码值(度) */
extern float  kin_extend_mm_per_deg;     /* 每电机"度"对应多少 mm 水平伸出 */
extern float  kin_extend_zero_deg;       /* r=kin_extend_r_min_mm 时的电机码值(度) */
extern float  kin_extend_r_min_mm;       /* 最短时夹爪中心到转轴的水平距离(mm) */
extern int8_t kin_extend_sign;           /* +1: 电机码值增大->r增大; -1 反向 */
extern float  kin_extend_target_offset_mm; /* 视觉点在伸出方向偏近的补偿(mm), 逆解命令加在 r 上 */

/* 升降(电机5): 高度 z(mm) <-> 电机码值(度) */
extern float  kin_hoist_mm_per_deg;
extern float  kin_hoist_zero_deg;        /* z=kin_hoist_z_min_mm 时的电机码值(度) */
extern float  kin_hoist_z_min_mm;        /* 最低时夹爪中心距地面高度(mm) */
extern int8_t kin_hoist_sign;

/* 工作空间 (逆解可达性判断) */
extern float  kin_extend_r_max_mm;
extern float  kin_hoist_z_max_mm;

/* 抓取高度(下位机定死) + 转云台前的安全位姿 */
extern float  kin_grab_z_mm;             /* 抓取时夹爪距地面高度(mm), 现场定死 */
extern float  kin_safe_extend_mm;        /* 转云台前: 电机6 安全位(mm) */
extern float  kin_safe_hoist_mm;         /* 转云台前: 电机5 安全位(mm) */

/* 运动参数 */
extern float    kin_gimbal_step_deg;     /* 云台每10ms最大转角(度) */
extern float    kin_gimbal_tol_deg;      /* 云台到位容差(度) */
extern float    kin_pos_tol_mm;          /* 电机5/6 到位容差(mm) */
extern uint8_t  kin_hoist_cmd_dir;       /* 电机5 位置命令 dir (沿用实测 0) */
extern uint8_t  kin_extend_cmd_dir;      /* 电机6 位置命令 dir (沿用实测 1) */
extern uint16_t kin_hoist_acc, kin_hoist_dec;
extern float    kin_hoist_vel;
extern uint16_t kin_extend_acc, kin_extend_dec;
extern float    kin_extend_vel;

/* ================= 状态 ================= */
extern uint8_t kin_cmd;        /* Watch: 写 1=GotoXY(kin_target_x/y), 2=停止 */
extern uint8_t kin_active;     /* 只读: 1=运动中 */
extern uint8_t kin_done;       /* 只读: 1=刚到位(一次, 用后清0) */
extern uint8_t kin_reachable;  /* 只读: 上次逆解 r 是否越界(0=被截断) */
extern uint8_t kin_hoist_enable;    /* 1=走电机5升降轴; 0=跳过升降步(只测云台+电机6) */
extern uint8_t kin_handshake_enable;/* 1=分步模式: 到点上方后等 ok(0x09) 再降到抓取高度; 0=全自动直接降 */

/* 调试用 (Watch 只读): 运动内部状态 */
extern uint8_t  kin_step;         /* 当前步骤 0=空闲 1=缩回 2=升 3=云台 4=伸 5=降 6=等ok */
extern uint32_t kin_motion_count; /* 触发次数, 确认重触发有没有进来 */
extern float    kin_tgt_servo, kin_tgt_m6_deg, kin_tgt_m5_deg; /* 内部目标(舵机/电机6/电机5 码值度) */

extern float kin_target_x, kin_target_y;                  /* 目标 (mm, 塔吊系) */
extern float kin_target_theta, kin_target_r, kin_target_z_mm; /* 内部目标(执行器位姿) */

extern float kin_pose_x, kin_pose_y, kin_pose_z;         /* 当前夹爪位姿 FK 结果 (ReportPose) */
extern float kin_pose_theta, kin_pose_r, kin_pose_z_mm;

/* ================= API ================= */
void Kin_FK(float theta_deg, float r_mm, float z_mm, float *x, float *y, float *z);
uint8_t Kin_IK(float x, float y, float z, float *theta_deg, float *r_mm, float *z_mm);

float Kin_ThetaToServo(float theta_deg);
float Kin_ServoToTheta(float servo_deg);
float Kin_ExtendMmToDeg(float r_mm);
float Kin_ExtendDegToMm(float deg);
float Kin_HoistMmToDeg(float z_mm);
float Kin_HoistDegToMm(float deg);

void Kin_GotoXY(float x, float y);   /* 上层给 (x,y), z 用 kin_grab_z_mm; 按安全顺序走 */
void Kin_Stop(void);
void Kin_Update(void);      /* 10ms, 由塔吊任务调用 */
void Kin_ReportPose(void);  /* 读当前电机位置 + 云台角 -> 算当前夹爪 (x,y,z) */

/* ================= 只读: 夹爪当前位置解算 ================= */
extern uint8_t kin_read_trig;   /* Watch 写 1 触发读一次, 读完清 0 */
void Kin_ReadGripper(void);     /* 读电机5/6 + 云台角 -> 正解到 kin_pose_* */

#endif /* __APP_KIN_H */
