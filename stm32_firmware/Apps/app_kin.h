#ifndef __APP_KIN_H
#define __APP_KIN_H

#include "stm32f4xx.h"

/**
  * @brief  塔吊抓取机构正/逆运动学 + 点到点运动 (圆柱坐标: 云台θ + 伸出r + 升降z)
  *         塔吊系: 原点 = 云台转轴在地面的投影, +X = 前, +Y = 左, +Z = 上(地面起算)
  *         执行器单位: 云台 = 角度(度); 电机5(升降)/电机6(伸出) 对外一律用 mm。
  *
  *         上层只发 (x, y), z 由下位机定死 (kin_cal.grab_z_mm)。
  *
  *         正解 Kin_FK:      (θ, r, z) -> (x, y, z)    供上层"铆钉"/回传当前位姿
  *         逆解 Kin_IK:      (x, y, z) -> (θ, r, z)    供需要完整 3D 目标时用
  *         运动 Kin_GotoXY:  (x, y) -> θ, r; z 固定; 按下面顺序走
  *
  *         姿态 (值在 car_config.h):
  *           初始化(上电) = 云台135 / 电机5 0 / 电机6 0
  *           运动姿态     = 云台310 / 电机5 600 / 电机6 500 (行驶/换工作点)
  *           探查姿态     = 云台135 / 电机5 600 / 电机6 0 (到工作点给视觉看)
  *           物料区微调前 = 云台135 / 电机5 460 / 电机6 0   (只在物料区用; 电机5 已在抓取高,
  *                          电机6 收着别挡转盘/物料视场, 等底盘微调 within=1)
  *           物料区预备位 = 云台135 / 电机5 460 / 电机6 260 (微调完再伸出去, 夹爪张开等授权)
  *         高度: 回转平台460(物料区抓) / 物料平台600(放+卸货抓) / 平地100(放+回装抓) /
  *               暂存码垛400(暂存2nd放) / 清障640(转云台/伸缩前先升)
  *
  *         物料区固定位装货 (kin_flag.cmd=10, 手动/测试; 主流程仍走视觉 0x08/0x09):
  *           摆物料区预备位 134/460/260 (夹爪张开, 电机5 就站在抓取高, 无下降步)
  *           -> 等授权 (kin_flag.mat_grab_cmd 或 视觉 0x09) -> 直接闭夹爪 -> 升640清障
  *           -> 云台放置角+电机6 放置伸出 -> 电机5 降到600 -> 松夹爪 -> 平台转下一档 -> 回物料区预备位
  *           cmd=9  只摆物料区预备位 134/460/260, 不抓, 可单步验证姿态
  *           cmd=13 只摆物料区微调前姿态 134/460/0 (电机6 收着), 不动底盘微调
  *           cmd=14 只把电机6 从 0 伸到 260 (云台/电机5 一律不动), 微调完之后那一步
  *           cmd=11 = 完整两段: 先摆 134/460/0, 姿态期间挂起视觉纠偏(chassis_flag.trim_hold=1),
  *                    姿态到位后放开, 等视觉给出新的 within=1 (0x0E) -> 只伸电机6 到 260 -> kin_flag.done=1
  *                    (和 mission MS_TRIM 同一套 Chassis_TrimArm/Release/Done 接口)
  *
  *         粗加工站整链测试 (kin_flag.cmd=15, 台架验证"纠偏 + 按扫码顺序卸货 + 回装"):
  *           摆探查姿态 134/600/0 (期间挂起纠偏) -> 放开纠偏等视觉 within=1 (0x04/0x05)
  *           -> Chassis_TrimBlock() 封死本站 -> 按二维码环序填 kin_tgt.drop_ring
  *           -> 卸货 3 个 (同 cmd=3) -> 自动回装 3 个 (同 cmd=7) -> 回探查姿态 -> kin_flag.done=1
  *           环序取哪一轮看 app_kin.c 的 kin_test_qr_round (0=第1轮 1=第2轮, Watch 可改);
  *           没扫到二维码时 drop_ring 用 1/2/3。整理口在 Mission_ApplyQRDropRing()。
  *
  *         暂存区整链测试 (kin_flag.cmd=16 平地 / cmd=17 堆垛, 都不回抓):
  *           和 cmd=15 同一条链 (探查姿态 -> 底盘微调 -> TrimBlock -> 按QR环序放 3 个 -> 回探查姿态),
  *           只差落高: 16 = HOIST_HEIGHT_DROP_GROUND (平地), 17 = HOIST_HEIGHT_PALLET (码垛),
  *           两个都直接取 car_config.h 的值 (全自动暂存区在 mission 里按轮次取同一对宏)。
  *           环序同样按二维码: 16 固定第1轮(平地), 17 固定第2轮(码垛), 不用写 kin_test_qr_round;
  *           没扫码回 1/2/3。
  *           等待上限 kin_trim_wait_10ms (默认 1000=10s, 写 0 取消), 15/16/17 共用。
  *           (2026-10-31 临时把超时放行注释掉了: 现在只认 Chassis_TrimDone())
  *
  *         单步"探查姿态 + 底盘微调" (kin_flag.cmd=18, 粗加工/暂存):
  *           摆 134/600/0 (期间挂起纠偏) -> 姿态到位放开纠偏 -> 等一次 within=1
  *           -> TrimBlock 封死本站 -> kin_flag.done=1。不伸电机6、不卸货。
  *           等价于 cmd=11 去掉"伸到 260"那一步; 想继续用就接着发 cmd=14 伸电机6。
  *
  *         装货(kin_flag.cmd=1, 物料区: 回转平台460 抓 -> 物料平台600 放; 不预降):
  *           云台+电机6 一起到(θ,r) -> 电机5 -> 460(已在抓取高, 秒过) -> 张夹爪 -> 等ok(0x09) -> 闭夹爪
  *           -> 电机5 升640清障 -> 云台310+电机6 500 一起 -> 电机5 降到600 -> 松夹爪
  *           -> 回探查姿态, 等下一个物料坐标
  *
  *         卸货(kin_flag.cmd=3 触发, 与装货方向相反; 物料平台600 抓 -> 放前面视觉点;
  *         转云台/伸缩前 电机5 先升640清障, 云台与电机6 可一起动; 夹取自动, 放开手动):
  *           电机5 升600 -> 云台310+电机6 500 一起 -> 稳停 -> 闭夹爪抓
  *           -> 电机5 升640清障 -> 云台+电机6 一起到落点(θ,r) (落点已知就直接过去,
  *              省掉"甩回正前135 + 电机6 收到0再伸出"两折; 没坐标才回 135/0 安全位) -> 等 0x0C 视觉坐标就绪
  *           -> 云台落点θ+电机6 落点r(停在640) -> 电机5 降到落点高(kin_cal.drop_hoist_deg, car_config 70/370) -> 松夹爪
  *          -> 还有落点则回抓下一个环, 依次放满 3 个 -> 完成待命
  *
  *         注意: 云台是普通 PWM 舵机, 无位置回读; 当前角从 Servo_GetAngle(SERVO_GIMBAL)
  *         读(servo.c 记录最近一次设定角)。启用 Kin 时请注释掉 MyTasks.c 里的
  *         Test_Task(), 避免 Kin 和手动测试抢同一批云台/电机。
  */

/* ================= 标志位 (位域打包: 共 33 位, 前 32 位在 1 个 uint32 里, end_boot 落到第 2 个字;
   Keil Watch 只看 kin_flag) ================= */
typedef struct {
    uint32_t cmd           : 5;   /* 0..31: 1=装货抓取 GotoXY / 2=停止 / 3=卸货 / 4=探查姿态 / 5=运动姿态 / 6=前往坐标 / 7=一键回装(反序3->2->1) / 8=一键堆垛(码垛高放0x0C三落点) / 9=物料区预备位(134/460/260) / 10=物料区固定位完整装货 / 11=物料区微调前姿态(134/460/0)+等底盘微调+只伸电机6到260 / 12=物料区连抓3件 / 13=物料区微调前姿态(134/460/0)只摆姿态 / 14=只伸电机6到260(云台/电机5不动) / 15=粗加工站整链测试(探查姿态+底盘微调+卸3按QR环序+回装3) / 16=暂存区平地卸货测试(微调+按QR环序放3于平地, 不回抓) / 17=暂存区堆垛测试(微调+按QR环序放3于码垛高, 不回抓) / 18=粗加工/暂存探查姿态(134/600/0)+底盘微调单步(cmd=11 去掉伸电机6, 到位即完成) */
    uint32_t active        : 1;   /* 只读: 1=运动中 */
    uint32_t done          : 1;   /* 只读: 1=刚到位(一次, 用后清0) */
    uint32_t reachable     : 1;   /* 只读: 上次逆解 r 是否越界(0=被截断) */
    uint32_t hoist_en      : 1;   /* 1=走电机5升降轴; 0=跳过升降步 */
    uint32_t handshake_en  : 1;   /* 1=分步模式: 到点上方等 0x09 ok 再降; 0=全自动直接降 */
    uint32_t auto_grip     : 1;   /* 1=装货/回装到位后自动闭夹爪 */
    uint32_t auto_release  : 1;   /* 1=卸货/堆叠到位后自动松夹爪 */
    uint32_t descend_cmd   : 1;   /* 手动下降抓取授权 (与 0x09 等价, 锁存) */
    uint32_t grip_cmd      : 1;   /* 手动闭夹爪 (锁存) */
    uint32_t release_cmd   : 1;   /* 手动松夹爪 (锁存) */
    uint32_t mat_grab_cmd  : 1;   /* 物料区抓取授权 (锁存): 已在抓取高 -> 置1 直接闭夹爪升640; 视觉0x09 也算 */
    uint32_t platform_sel  : 2;   /* 物料平台槽位 0/1/2 -> A/B/C */
    uint32_t unload_idx    : 2;   /* 当前放第几个 (0/1/2) */
    uint32_t unload_have   : 1;   /* 1=已收到过 0x0C 坐标(至少一次) */
    uint32_t unload_lock   : 1;   /* 1=坐标已锁定(回去夹东西时停止更新) */
    uint32_t read_trig     : 1;   /* 写1 读一次夹爪位姿(读完清0) */
    uint32_t step          : 6;   /* 当前步骤 0..55 (看卡在哪一步) */
    uint32_t phase         : 1;   /* 0/1 交错: 发命令 / 读位置 (防 CAN 背靠背冲突) */
    uint32_t need_send     : 1;   /* 1=当前步需要发一次位置命令 */
    uint32_t hoist_cmd_dir : 1;   /* 电机5 位置命令 dir (沿用实测 0) */
    uint32_t extend_cmd_dir: 1;   /* 电机6 位置命令 dir (沿用实测 1) */
    uint32_t end_boot      : 1;   /* 1=本条链是全站最后一件: 收尾直接回运动姿态 310/600/500,
                                     不再先摆探查姿态 (134/600/0 或物料 134/460/260)。
                                     mission 在发最后一件的命令前置1, kin 回位时消费清0;
                                     Kin_Stop/Kin_ResetRun 也会清, 停过之后不会留在下一场 */
    uint32_t mid_hold      : 1;   /* 1=本件不是最后一件, 但下一件马上就来: 收尾只把电机5 抬到清障高
                                     (place_hoist_up_deg) 就报完成, 不回探查姿态 134/600/0。
                                     用在粗加工"卸完 3 个 -> 回装 3 个"和回装第 1/2 件:
                                     三个落点是一次性从 0x0C 拿到的 (回装期间坐标还锁死),
                                     每件之间回探查位等于"收回 -> 甩到正前 134 -> 再甩回落点", 白走两段;
                                     抬到清障高再交班, 下一件从清障高直接转向下一落点, 臂也不在平台高横扫。
                                     mission 在发这一件的命令前置1, kin 收尾时消费清0;
                                     和 end_boot 互斥 (end_boot 优先), Kin_Stop/Kin_ResetRun 也会清 */
} KinFlag_t;

/* ================= 手动测试标志 (位域打包: 11 位叠进 uint8, Watch 只看 kin_test) ================= */
typedef struct {
    uint8_t mode           : 3;   /* 1=装货 2=卸货平地 3=回装 4=暂存 */
    uint8_t trig           : 1;   /* 写1 触发, 消费后清0 */
    uint8_t coord_mode     : 1;   /* 0=(x,y)mm逆解; 1=直接角度 */
    uint8_t handshake      : 1;   /* 0=全自动(不等视觉) 1=分步等视觉(0x08/0x09/0x0C) */
    uint8_t grab_manual    : 1;   /* 装货等0x08卡住: 写1 用 kin_tgt.test_x/y 当目标 */
    uint8_t unload_manual  : 1;   /* 卸货等0x0C卡住: 写1 用 kin_tgt.test_x/y 当唯一落点 */
    uint8_t platform_sel   : 2;   /* 物料平台手动档位 0/1/2 -> A/B/C */
    uint8_t platform_trig  : 1;   /* 写1 按 sel 立即转物料平台 */
} KinTest_t;

/* ================= 标定 + 姿态参数 (Keil Watch 现场标定; 单位 mm / 度) ================= */
typedef struct {
    /* 云台 */
    float  gimbal_theta_offset; int8_t gimbal_dir; float gimbal_gain;
    /* 伸出(电机6) */
    float  extend_mm_per_deg, extend_zero_deg, extend_r_min_mm; int8_t extend_sign;
    float  extend_target_offset_mm, extend_r_trim_mm;
    float  extend_rail_x0_mm, extend_rail_y0_mm, extend_rail_phi_deg;
    float  unload_x_offset_mm, unload_y_offset_mm, grab_y_offset_mm;
    /* 升降(电机5) */
    float  hoist_mm_per_deg, hoist_zero_deg, hoist_z_min_mm; int8_t hoist_sign;
    /* 工作空间 */
    float  extend_r_max_mm, hoist_z_max_mm;
    /* 抓取/放置/落点高度 */
    float  grab_z_mm, drop_hoist_deg, reload_grab_hoist_deg;
    float  place_hoist_up_deg, place_extend_deg, place_gimbal_deg, place_gimbal_deg_c;
    float  home_gimbal_deg, home_extend_deg, home_hoist_deg, place_hoist_deg, place_extend2_deg, unload_grab_hoist_deg;
    /* 全程默认/休息姿态 */
    float  pose_default_gimbal_deg, pose_default_hoist_deg, pose_default_extend_deg;
    float  pose_boot_gimbal_deg, pose_boot_hoist_deg, pose_boot_extend_deg;
    /* 物料区专属姿态 (只在物料区用); 电机5 直接站在抓取高, 无单独下降步。
       pose_material_*      = 抓取预备位 134/460/260 (cmd=9/10/12 和微调完成后用)
       pose_material_trim_* = 微调前姿态 134/460/0  (电机6 收回, 别挡着转盘/物料, 微调完再伸出去) */
    float  pose_material_gimbal_deg, pose_material_hoist_deg, pose_material_extend_deg;
    float  pose_material_trim_extend_deg;
    /* 夹爪角度 */
    float  gripper_close_angle, gripper_open_angle;
} KinCal_t;

/* ================= 运动参数 + 状态计数 ================= */
typedef struct {
    float    gimbal_step_deg, gimbal_tol_deg, pos_tol_mm;
    float    gimbal_acc, gimbal_dec;   /* 云台梯形加减速 (度/拍²) */
    uint16_t hoist_acc, hoist_dec;   float hoist_vel;
    uint16_t extend_acc, extend_dec; float extend_vel;
    uint16_t gripper_delay_10ms, gimbal_settle_10ms;
    uint16_t keepalive;              /* 兜底重发计数 */
    uint32_t motion_count;           /* 触发次数, 确认重触发有没有进来 */
} KinMotion_t;

/* ================= 目标 / 位姿 / 调试值 (Watch 看) ================= */
typedef struct {
    float target_x, target_y;                          /* 目标 (mm, 塔吊系) */
    float target_theta, target_r, target_z_mm;         /* 内部目标(执行器位姿) */
    float goto_x, goto_y, goto_z;                      /* 前往坐标测试目标 (kin_flag.cmd=6) */
    float last_tx, last_ty;                            /* 上次已执行目标, 用于自动触发 */
    float servo, m6_deg, m5_deg;                       /* 内部目标(舵机/电机6/电机5 码值度) */
    float pose_x, pose_y, pose_z;                      /* 当前夹爪位姿 FK 结果 */
    float pose_theta, pose_r, pose_z_mm;
    float unload_x[3], unload_y[3];                    /* 0x0C 给的 3 个落点 XY */
    float drop_actual_x[3], drop_actual_y[3];          /* 卸货实际前往落点(带偏移后) 下标0/1/2=放第1/2/3个 */
    float reload_actual_x[3], reload_actual_y[3];      /* 回装实际前往落点(带偏移后) 下标0/1/2=抓第1/2/3个 */
    uint8_t drop_ring[3];                              /* 落点环序: 第 i 抓取的槽位 -> 放第几号环(1~3) */
    float test_x, test_y;                              /* 测试目标 (mm, 塔吊系) */
    float test_gimbal_deg, test_m6_deg;                /* 测试直接角度目标 */
} KinTgt_t;

extern KinFlag_t   kin_flag;
extern KinTest_t   kin_test;
extern KinCal_t    kin_cal;
extern KinMotion_t kin_motion;
extern KinTgt_t    kin_tgt;

/* ================= API ================= */
void Kin_FK(float theta_deg, float r_mm, float z_mm, float *x, float *y, float *z);
uint8_t Kin_IK(float x, float y, float z, float *theta_deg, float *r_mm, float *z_mm);

float Kin_ThetaToServo(float theta_deg);
float Kin_ServoToTheta(float servo_deg);
float Kin_ExtendMmToDeg(float r_mm);
float Kin_ExtendDegToMm(float deg);
float Kin_HoistMmToDeg(float z_mm);
float Kin_HoistDegToMm(float deg);

void Kin_GotoXY(float x, float y);   /* 上层给 (x,y), z 用 kin_cal.grab_z_mm; 按安全顺序走 */
void Kin_GotoPointXYZ(float x, float y, float z);  /* 前往坐标测试: (x,y,z) 逆解->安全顺序走到, 不夹取 */
void Kin_UnloadStart(void);          /* 卸货启动: 平台抓取 -> 收回 -> 落点 -> 等视觉 */
void Kin_UnloadDrop(void);           /* 卸货放置: 用 kin_tgt.target_x/y 解 θ/r, 到落点后降, 到位等放开 */
void Kin_ReloadStart(void);          /* 一键回装: 反序 3->2->1 从落点平地抓回放平台 (用已锁存的 0x0C 落点) */
void Kin_BootPosture(void);          /* 上电引导位姿: 夹爪开 -> 平台A -> 和 Tower_Boot_Task 同一姿态 (boot_*: 270/600/550, 不夹) */
void Kin_GotoPosture(float gimbal_deg, float m5_deg, float m6_deg);  /* 通用姿态切换 */
void Kin_GoDefault(void);            /* 去探查姿态 134/600/0 (粗加工/暂存) */
void Kin_GoMaterialProbe(void);      /* 去物料区专属探查姿态 134/460/260 (只摆姿态不抓) */
void Kin_GoMaterialExtend(void);     /* 只把电机6 从 0 伸到 260 (云台/电机5 不动); 底盘微调完之后用 */
void Kin_UnloadChainTest(void);      /* cmd=15: 探查姿态+底盘微调 -> 按QR环序卸3 -> 回装3 (粗加工站整链测试) */
void Kin_StagingUnloadTest(uint8_t stack);  /* cmd=16/17: 微调 -> 按QR环序放3, 不回抓 (stack=0 平地 / 1 码垛) */
void Kin_MaterialGrabStart(void);    /* 物料区固定位完整装货(cmd=10; cmd=12 也用它再置连抓次数): 摆姿态(即抓取高) -> 等 mat_grab_cmd/0x09 -> 闭爪 -> 升640 -> 放置 */
void Kin_MaterialProbeTrimStart(void); /* 物料区探查姿态 + 底盘微调: 摆姿态期间挂起纠偏, 到位后放开并等 within=1 (cmd=11) */
void Kin_DefaultProbeTrimStart(void); /* 粗加工/暂存探查姿态 + 底盘微调 (cmd=18): 摆 134/600/0, 到位放开纠偏等 within=1 -> 直接完成, 不伸电机6 */
void Kin_GoBoot(void);               /* 回运动姿态 310/600/500 */
uint8_t Kin_AtBootPose(void);        /* 运动姿态 310/600/500 是否已就位 (mission MS_POSE_BOOT 用:
                                        收工那一步 kin 已经直接回运动姿态时, 别再发一次 cmd=5 白抬电机5) */
void Kin_Stop(void);
void Kin_ResetRun(void);    /* 整场重来: 清这一场攒下的锁存/目标/内部步 (不动标定/整定, 不停电机; 先 Kin_Stop) */
void Kin_Update(void);      /* 10ms, 由塔吊任务调用 */
void Kin_TestRun(void);     /* 10ms, 手动测试触发(与 Kin_Update 同任务) */
void Kin_ReportPose(void);  /* 读当前电机位置 + 云台角 -> 算当前夹爪 (x,y,z) */
void Kin_ReadGripper(void); /* 读电机5/6 + 云台角 -> 正解到 kin_tgt.pose_* */

#endif /* __APP_KIN_H */
