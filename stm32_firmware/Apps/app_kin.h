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
  *         运动 Kin_GotoXY:  (x, y) -> θ, r; z 固定; 按下面顺序走
  *
  *         约定位姿(装货测试): 默认位=云台42 / 电机5 50 / 电机6 0;
  *                             抓取位=云台(视觉θ) / 电机5 447 / 电机6(视觉r);
  *                             放置位=云台220 / 电机5 480 / 电机6 500。
  *         抓取顺序(默认位 -> 抓取位, 不先升600):
  *           1) 电机5 -> kin_grab_approach_deg (先, 550 预降)
  *           2) 云台   -> 目标角 (再)
  *           3) 电机6 -> 目标 r (后)
  *           4) 到抓取位 -> 等抓取授权(视觉0x09 或手动 kin_descend_cmd)
  *           5) 收到ok -> 电机5 下探到 kin_grab_z_mm(447)
  *           6) 等夹爪闭合(手动 kin_grip_cmd) -> 闭夹爪
  *         抓取后的装货放置(自动, 单位同 Tower: 电机5/6=电机度, 云台=舵机度;
  *         抓取->放置 和 放置->回默认 都先升600清障; 默认->抓取 不升600):
  *           7) 电机5 -> kin_place_hoist_up_deg(先升600)   8) 电机6 -> kin_place_extend_deg(后600)
  *           9) 云台   -> kin_place_gimbal_deg(220)  10) 电机6 -> kin_place_extend2_deg(500 先)
  *          11) 电机5 -> kin_place_hoist_deg(480 后) 12) 夹爪开 -> 旋转一次物料平台
  *         放完后自动回默认位(再等下一次视觉坐标 kin_cmd=1; 同样先升600清障):
  *          13) 电机5 -> kin_place_hoist_up_deg(先升600) 14) 电机6 -> kin_place_extend_deg(后伸600)
  *          15) 云台   -> kin_home_gimbal_deg(42)   16) 电机6 -> kin_home_extend_deg(收0, 电机5保持600)
  *          -> 完成, 夹爪保持默认张开待命
 *
 *         卸货(kin_cmd=3 触发, 与装货方向相反; 抓平台 -> 放前面视觉点;
 *         220与42互相转动前 电机5/6先停600清障, 舵机到位才动5/6; 夹取自动, 放开手动):
 *          18) 电机5 -> kin_place_hoist_up_deg(600 park 先升) 19) 电机6 -> kin_place_extend_deg(600 park)
 *          20) 云台   -> kin_place_gimbal_deg(220 平台)
 *          21) 电机6 -> kin_place_extend2_deg(500 平台槽) 22) 电机5 -> kin_unload_grab_hoist_deg(447 抓取高)
 *          23) 夹爪自动闭合 -> 延时
 *          24) 电机5 -> kin_place_hoist_up_deg(600 清障) 25) 电机6 -> kin_place_extend_deg(600)
 *          26) 云台   -> kin_home_gimbal_deg(42 回前)
 *          27) (跳过: 电机5 保持600 高位, 不再先降50) 28) 电机6 -> kin_home_extend_deg(0 收回)
 *          29) 等 0x0C 卸货坐标就绪(坐标后台持续更新, 回去夹东西时已锁定, 直接用)
 *          30) 云台   -> 落点θ  31) 电机6 -> 落点r (m5 保持600 高位)
 *          32) 电机5 -> kin_home_hoist_deg(50 到落点上方后再降)
 *          33) 等放开(手动 kin_release_cmd) -> 松夹爪 34) 延时 -> 平台转下一档(C->B->A)
 *          -> 还有落点则回 18) 抓下一个环, 依次放满 3 个 -> 完成待命
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
extern float  kin_unload_x_offset_mm;     /* 卸货落点: 视觉坐标 x 偏移(mm) */
extern float  kin_unload_y_offset_mm;     /* 卸货落点: 视觉坐标 y 偏移(mm) */

/* 升降(电机5): 高度 z(mm) <-> 电机码值(度) */
extern float  kin_hoist_mm_per_deg;
extern float  kin_hoist_zero_deg;        /* z=kin_hoist_z_min_mm 时的电机码值(度) */
extern float  kin_hoist_z_min_mm;        /* 最低时夹爪中心距地面高度(mm) */
extern int8_t kin_hoist_sign;

/* 工作空间 (逆解可达性判断) */
extern float  kin_extend_r_max_mm;
extern float  kin_hoist_z_max_mm;

/* 抓取高度(下位机定死) + 转云台前的安全位姿 */
extern float  kin_grab_z_mm;             /* 收到ok后下探的抓取高度(mm==度, 447) */
extern float  kin_grab_approach_deg;     /* 抓取前预降(视觉对准)高度(电机度, 550) */

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
extern uint8_t kin_cmd;        /* Watch: 写 1=装货抓取 GotoXY; 卸货等视觉时 1=到位等放开; 2=停止; 3=卸货(平台抓取->50默认->等视觉) */
extern uint8_t kin_active;     /* 只读: 1=运动中 */
extern uint8_t kin_done;       /* 只读: 1=刚到位(一次, 用后清0) */
extern uint8_t kin_reachable;  /* 只读: 上次逆解 r 是否越界(0=被截断) */
extern uint8_t kin_hoist_enable;    /* 1=走电机5升降轴; 0=跳过升降步(只测云台+电机6) */
extern uint8_t kin_handshake_enable;/* 1=分步模式: 到点上方后等 ok(0x09) 再降到抓取高度; 0=全自动直接降 */

/* 装货放置 (抓取后自动执行): 单位与 Tower 一致 —— 电机5/6=电机度(正数), 云台=舵机度 */
extern float kin_place_hoist_up_deg;   /* 电机5 转云台前停靠600(抓取->放置 / 放置->回默认 共用) */
extern float kin_place_extend_deg;     /* 电机6 转云台前停靠600(同上) */
extern float kin_place_gimbal_deg;     /* 云台放置角(舵机度) */
extern float kin_home_gimbal_deg;      /* 云台回抓取位角(舵机度, 42) */
extern float kin_home_extend_deg;      /* 电机6 回抓取位(收回, 最短 0) */
extern float kin_home_hoist_deg;       /* 电机5 默认位(低, 50) */
extern float kin_place_hoist_deg;      /* 电机5 放置高度 */
extern float kin_place_extend2_deg;    /* 电机6 放置伸出 */
extern float kin_unload_grab_hoist_deg;/* 卸货: 平台抓取时电机5高度 (装货放置480 改成447) */

/* 卸货: 0x0C 帧一次给 3 个落点(3 组 xy), 状态机依次放 */
extern float   kin_unload_x[3];        /* 3 个落点 X (塔吊系绝对 mm) */
extern float   kin_unload_y[3];        /* 3 个落点 Y */
extern uint8_t kin_unload_idx;         /* 当前放第几个 (0/1/2) */
extern uint8_t kin_unload_have;        /* 1=已收到过 0x0C 坐标(至少一次), 可去放 */
extern uint8_t kin_unload_lock;        /* 1=坐标已锁定(回去夹东西时停止更新), 整轮放完清零恢复更新 */
extern float kin_gripper_close_angle;  /* 抓取闭合角 */
extern float kin_gripper_open_angle;   /* 放置张开角 */
extern uint16_t kin_gripper_delay_10ms;/* 夹爪动作后等待(10ms 单位) */

/* 手动触发命令 (锁存: Watch 写1, 状态机消费后清0) */
extern uint8_t kin_descend_cmd;   /* 手动下降抓取授权, 与视觉 grab_cmd(0x09) 等价 */
extern uint8_t kin_grip_cmd;      /* 手动夹爪闭合抓取(装货用) */
extern uint8_t kin_release_cmd;   /* 手动松开夹爪(卸货用) */
extern uint8_t kin_platform_sel;  /* 物料平台槽位 0/1/2 -> A/B/C, 每次放置完成后旋转一次 */

/* 调试用 (Watch 只读): 运动内部状态 */
extern uint8_t  kin_step;         /* 当前步骤 0=空闲 1=下探447 2=旧停靠(废弃) 3=云台 4=伸6 5=升5(550预降) 6=等授权 7=等夹爪 8~13=放置 14~17=回默认 18~34=卸货 */
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
void Kin_UnloadStart(void);          /* 卸货启动: 平台抓取(220/447/500, 自动夹取)->50默认(42/50/0)->等视觉 */
void Kin_UnloadDrop(void);           /* 卸货放置: 用 kin_target_x/y 解 θ/r(m5保持600高位), 到落点后降50, 到位等放开(kin_release_cmd) */
void Kin_Stop(void);
void Kin_Update(void);      /* 10ms, 由塔吊任务调用 */
void Kin_ReportPose(void);  /* 读当前电机位置 + 云台角 -> 算当前夹爪 (x,y,z) */

/* ================= 只读: 夹爪当前位置解算 ================= */
extern uint8_t kin_read_trig;   /* Watch 写 1 触发读一次, 读完清 0 */
void Kin_ReadGripper(void);     /* 读电机5/6 + 云台角 -> 正解到 kin_pose_* */

#endif /* __APP_KIN_H */
