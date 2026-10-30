#include "sys.h"

/**
  * @brief  塔吊抓取机构正/逆运动学 + 点到点运动 (圆柱坐标)
  *         塔吊系: 原点 = 云台转轴地面投影, +X = 前, +Y = 左, +Z = 上
  *         执行器单位: 云台 = 度, 电机5/6 = mm (内部换算到电机码值"度")
  *         标定系数 kin_* 先占位, 现场在 Keil Watch 里标定后填入。
  */

/* 塔吊电机实时位置 (app_tower.c 里 Tower_ReadPos5/6 维护, 单位"度") */
extern float motor5_pos;
extern float motor6_pos;

/* 上位机 0x09 GrabCmd 授权抓取 (Serial.c 收到 "1" 置 1, 分步模式下消费后清 0) */
extern uint8_t grab_cmd;

#define KIN_DEG2RAD (MATH_PI / 180.0f)
#define KIN_RAD2DEG (180.0f / MATH_PI)

/* 运动顺序 (状态机步骤)
 * 新流程: 云台+电机6 并行步(40~44) 与 电机5 单独升降步交替, 不再有"预降/停靠600"多步。
 * 抓取/放置前一律先升电机5 到 640 清障, 再转云台+伸缩。 */
#define KIN_STEP_IDLE          0
/* (1~4 旧顺序已废弃: 预降 / 云台 / 电机6 单独步) */
#define KIN_STEP_REACH_HOIST   5   /* 电机5 -> 抓取高度(最后降), 到位先张夹爪 */
#define KIN_STEP_WAIT_GRAB      6   /* 等抓取授权(视觉0x09 或手动 kin_flag.descend_cmd) -> 闭夹爪 */
#define KIN_STEP_WAIT_GRIP      7   /* 等夹爪闭合 -> 闭夹爪 -> 进入放置 */
#define KIN_STEP_PLACE_HOIST_UP 8   /* 电机5 -> 640 清障 */
/* (9~11 旧顺序已废弃: 放置前 m6 停靠 / 云台 / m6 伸出) */
#define KIN_STEP_PLACE_HOIST    12  /* 电机5 -> 600 放置高 */
#define KIN_STEP_PLACE_RELEASE  13  /* 夹爪开 -> 旋转物料平台 -> 回默认 */
/* (14~17 旧顺序已废弃) */
/* 卸货 (kin_flag.cmd=3 触发): 平台抓取(600) -> 640清障 -> 收回 -> 等视觉 -> 走落点坐标 -> 落点高 -> 依次放 3 个 */
#define KIN_STEP_UNLOAD_GRAB_HOIST_UP   18  /* 电机5 -> 600 平台抓取高 */
/* (19~22 旧顺序已废弃: 卸货抓取的 m6 停靠 / 云台 / m6 伸出 / 电机5 下降) */
#define KIN_STEP_UNLOAD_GRIP            23  /* 夹爪自动闭合 + 延时 */
#define KIN_STEP_UNLOAD_RETURN_HOIST_UP 24  /* 电机5 -> 640 清障 */
/* (25~28 旧顺序已废弃: 卸货收回的 m6 停靠 / 云台 / 电机5 / 收回) */
#define KIN_STEP_UNLOAD_WAIT_VISION     29  /* 等视觉 0x0C 落点坐标 */
/* (30~31 旧顺序已废弃: 卸货落点的云台 / 电机6 单独步) */
#define KIN_STEP_UNLOAD_DROP_HOIST      32  /* 电机5 -> 落点高(100/400) */
#define KIN_STEP_UNLOAD_WAIT_RELEASE    33  /* 等放开 -> 松夹爪 */
#define KIN_STEP_UNLOAD_RELEASE         34  /* 松夹爪 + 延时 -> 转平台 -> 回默认/完成 */
/* 通用姿态切换 (Kin_GotoPosture): 先升640清障 -> 云台+电机6 一起 -> 目标高 */
#define KIN_STEP_POSE_HOIST_UP    35   /* m5 -> 640 清障 */
#define KIN_STEP_POSE_DUAL        36   /* 云台+电机6 一起到目标角/目标伸出 */
#define KIN_STEP_POSE_HOIST       39   /* m5 -> 目标高 */
/* 物料区固定位装货 (kin_flag.cmd=10): 摆物料区探查姿态(135/460/200, 夹爪张开)
   -> 等授权(mat_grab_cmd 或视觉0x09) -> 直接闭夹爪 -> 升640 -> 现有装货后半段 (放置 -> 松爪) */
#define KIN_STEP_MAT_WAIT_GRAB    54   /* 等授权 -> 闭夹爪 -> 装货后半段 (探查就在抓取高, 无下降步) */
/* 物料区"探查姿态 + 底盘微调"单步测试 (kin_flag.cmd=11): 姿态到位后等底盘 within */
#define KIN_STEP_WAIT_CHASSIS     55   /* 等底盘视觉微调 Chassis_TrimDone() -> 完成 */

/* 云台 + 电机6 并行步 (云台PWM 与 电机6 CAN 同步动, 两轴都到位才推进) */
#define KIN_STEP_APPROACH_GRAB        40   /* 装货: 云台->θ + 电机6->r (抓取位) */
#define KIN_STEP_APPROACH_PLACE       41   /* 装货放置: 云台->310 + 电机6->500 */
#define KIN_STEP_UNLOAD_GRAB_APPROACH 42   /* 卸货抓取: 云台->310 + 电机6->500 */
#define KIN_STEP_UNLOAD_RETRACT       43   /* 卸货收回: 云台->135 + 电机6->0 */
#define KIN_STEP_UNLOAD_DROP_APPROACH 44   /* 卸货落点: 云台->θ + 电机6->r */
/* 回装 (kin_flag.cmd=7 触发): 与卸货相反, 反序 3->2->1 从落点平地抓回, 放回物料平台 */
#define KIN_STEP_RLD_HOIST_UP          45  /* m5 -> 640 清障 */
#define KIN_STEP_RLD_APPROACH_DROP     46  /* 云台+电机6 -> 落点[i] (θ,r) */
#define KIN_STEP_RLD_DROP_HOIST        47  /* m5 -> 平地抓取高 */
#define KIN_STEP_RLD_GRIP              48  /* 闭夹爪 + 延时 */
#define KIN_STEP_RLD_RETURN_HOIST_UP   49  /* m5 -> 640 清障 */
#define KIN_STEP_RLD_APPROACH_PLATFORM 50  /* 云台+电机6 -> 平台(310/460) */
#define KIN_STEP_RLD_PLATFORM_HOIST    51  /* m5 -> 平台抓取高 */
#define KIN_STEP_RLD_RELEASE           52  /* 松夹爪 + 转平台 + 下一个/完成 */

/* ================= 打包后的变量 (Keil Watch 只看 kin_flag/kin_test/kin_cal/kin_motion/kin_tgt) ================= */
KinFlag_t   kin_flag = {
    .reachable      = 1,
    .hoist_en       = 1,
    .handshake_en   = 1,
    .extend_cmd_dir = 1,
};
KinTest_t   kin_test = {0};
KinCal_t    kin_cal = {
    /* 云台 */
    .gimbal_theta_offset = 135.0f,   /* θ=0(+X) 时舵机值 = 135 (舵机重装后) */
    .gimbal_dir          = 1,        /* +1: 舵机值增大 -> θ增大(逆时针) */
    .gimbal_gain         = 1.0f,     /* 斜率 = 1.0 (用户改回: θ=90→+Y 舵机225) */
    /* 伸出(电机6) */
    .extend_mm_per_deg   = 0.174f,   /* 实测: 伸出140.76mm / 808.8度 */
    .extend_r_min_mm     = 104.48f,  /* 电机6=0(最短)时径向距离 */
    .extend_sign         = -1,       /* 伸出时 motor6_pos 读负值, 取 -1 让 r 增大 */
	.extend_r_trim_mm = 20.0f,
    /* 轨道偏置 (2026-10-17 实测): 导轨不穿过转轴, 导轨角 φ≈-2.52° */
    .extend_rail_x0_mm   = 105.12f,  /* θ=0 时电机6=0 的夹爪 x (mm) */
    .extend_rail_y0_mm   = 17.09f,   /* θ=0 时电机6=0 的夹爪 y (mm, +Y=左) */
    .extend_rail_phi_deg = -2.52f,   /* 导轨角 (°), 相对 +X */
    /* 升降(电机5) */
    .hoist_mm_per_deg    = 1.0f,
    .hoist_sign          = 1,
    /* 工作空间 */
    .extend_r_max_mm     = 245.0f,   /* 物理最长 r ≈ 245mm (808.8度 × 0.174 + 104.48) */
    .hoist_z_max_mm      = 1000.0f,
    /* 抓取/放置/落点高度 */
    .grab_z_mm           = HOIST_HEIGHT_ROTATE,   /* 装货抓取高度 (=HOIST_HEIGHT_ROTATE 460) */
    .drop_hoist_deg      = HOIST_HEIGHT_DROP_GROUND,   /* 卸货放置 平地落点高度 */
    .reload_grab_hoist_deg = HOIST_HEIGHT_GRAB_GROUND, /* 回装: 从平地抓取 电机5高度 */
    .place_hoist_up_deg  = HOIST_CLEARANCE_HEIGHT,/* 电机5 过渡清障 640 */
    .place_extend_deg    = 600.0f,   /* (保留) 电机6 转云台前停靠600 */
    .place_gimbal_deg    = 310.0f,   /* 云台放置/抓取角(舵机度, 物料平台 A/B 槽位) */
    .place_gimbal_deg_c  = 308.0f,   /* 云台放置/抓取角(舵机度, 物料平台 C 槽位) */
    .home_gimbal_deg     = 135.0f,   /* 云台回抓取位角(舵机度, 舵机重装后) */
    .home_hoist_deg      = HOIST_HEIGHT_GROUND,   /* 电机5 默认位(低) = 平地100 */
    .place_hoist_deg     = HOIST_HEIGHT_PLATFORM, /* 电机5 放置高度 = 物料平台600 */
    .place_extend2_deg   = 450.0f,   /* 电机6 放置伸出 */
    .unload_grab_hoist_deg = HOIST_HEIGHT_UNLOAD_GRAB, /* 卸货: 平台抓取时电机5高度 = 550 */
    /* 全程默认/休息姿态 */
    .pose_default_gimbal_deg = POSTURE_PROBE_GIMBAL,
    .pose_default_hoist_deg  = POSTURE_PROBE_HOIST,
    .pose_default_extend_deg = POSTURE_PROBE_EXTEND,
    .pose_boot_gimbal_deg    = POSTURE_MOTION_GIMBAL,
    .pose_boot_hoist_deg     = POSTURE_MOTION_HOIST,
    .pose_boot_extend_deg    = POSTURE_MOTION_EXTEND,
    /* 物料区专属探查姿态 (只在物料区用): 电机5 直接站在抓取高, 无单独下降步 */
    .pose_material_gimbal_deg = POSTURE_MATERIAL_GIMBAL,
    .pose_material_hoist_deg  = POSTURE_MATERIAL_HOIST,
    .pose_material_extend_deg = POSTURE_MATERIAL_EXTEND,
    /* 夹爪角度 */
    .gripper_close_angle = GRIPPER_CLOSE_ANGLE,
    .gripper_open_angle  = GRIPPER_OPEN_ANGLE,
};
KinMotion_t kin_motion = {
    .gimbal_step_deg    = 1.5f,   /* 云台巡航速度 0.5°/拍 ≈ 50°/s (要提速改这里) */
    .gimbal_acc         = 0.05f,   /* 云台梯形加减速 0.05°/拍² ≈ 100ms 加到满速 */
    .gimbal_dec         = 0.01f,
    .gimbal_tol_deg     = 0.5f,
    .pos_tol_mm         = 2.0f,
    .hoist_acc          = 100,
    .hoist_dec          = 100,
    .hoist_vel          = 100.0f,
    .extend_acc         = 100,
    .extend_dec         = 100,
    .extend_vel         = 100.0f,
    .gripper_delay_10ms = 50,
    .gimbal_settle_10ms = 50,
};
KinTgt_t    kin_tgt = {
    .drop_ring       = {1, 2, 3},   /* 卸货/堆叠落点环序: 第 i 抓取的槽位 -> 放第几号环(1~3) */
    .test_gimbal_deg = 310.0f,      /* coord_mode=1: 目标云台舵机° */
    .test_m6_deg     = 500.0f,      /* coord_mode=1: 目标电机6° */
};

/* 姿态切换内部目标 (Kin_GotoPosture 用) */
static float s_pose_gimbal = 0.0f;
static float s_pose_m5 = 0.0f;
static float s_pose_m6 = 0.0f;

/* 姿态到位后接着走的步 (0=正常完成): 物料区固定位装货用, 摆完姿态续到下一段 */
static uint8_t s_pose_after = 0;

/* 本工作区探查姿态类别: 1=物料区专属 (135/460/200), 0=通用 (135/600/0);
   装货完成"回探查姿态"按它选, 物料区装完回物料区那套 */
static uint8_t s_probe_is_material = 0;

/* cmd=11 的底盘微调: 姿态到位后只放开一次 (Chassis_TrimRelease) */
static uint8_t s_trim_released = 0;
/* 1=底盘微调武装是 cmd=11 拿的 (mission 自己 Arm 的不算); 被别的 cmd 打断时要交还 */
static uint8_t s_trim_owned = 0;

/* 卸货收回步云台目标: 落点已知 = 落点方向(直走省掉甩回正前135), 未知 = 135 中立位 */
static float s_retract_gimbal = 0.0f;

/* 手动测试接口内部态 (Kin_TestRun 用) */
static uint8_t s_kin_test_direct = 0;   /* 1=本次测试用直接角度覆盖云台/电机6 目标(消费后清0) */
static float   s_kin_test_servo = 0.0f; /* 直接角度模式的云台目标 */
static float   s_kin_test_m6    = 0.0f; /* 直接角度模式的电机6目标 */
static uint8_t s_kin_single_cycle = 0;  /* 1=单次拿放: 卸货放1个就回探查姿态 */

/* cmd=12 连抓: 还剩几次"回探查姿态后继续等 0x09"。0=放完就结束 (cmd=10 的默认行为) */
static uint8_t s_mat_chain_left = 0;

/* 物料区一件"抓->放->回探查位"走完待上报 0x0F (PickDone)。
   不能在放完那一刻就发: 上位机收到 0x0F 会马上发下一个 0x08, 而 Serial.c 在 kin_flag.active 时
   丢掉 0x08 的目标点, 这一件的坐标就没了。所以在回探查姿态到位(塔吊空闲、车不动)那拍才发。 */
static uint8_t s_pick_pending_report = 0;

/* 本件颜色号: 闭夹爪那一刻从 Serial_RxData.target_color_id 锁存 (就是视觉 0x08 发来的那个数),
   回位上报 0x0F 时原样带回给上位机。闭爪之后再来 0x08 也不影响已锁存的值。 */
static uint8_t s_pick_color = 0;

/* 卸货落点/夹爪/运动参数/状态 全部打包进上方 kin_flag/kin_cal/kin_motion/kin_tgt */

/* ================= 单位换算 ================= */
/* 舵机角归一化到 [0, 360): 360° 舵机只能在一圈内运动, 负角回绕 (如 -51 -> 309) */
float Kin_NormServo(float s)
{
    while (s >= 360.0f) s -= 360.0f;
    while (s <   0.0f) s += 360.0f;
    return s;
}

float Kin_ThetaToServo(float theta_deg)
{
    return Kin_NormServo(kin_cal.gimbal_theta_offset + kin_cal.gimbal_dir * kin_cal.gimbal_gain * theta_deg);
}
float Kin_ServoToTheta(float servo_deg)
{
    float th   = kin_cal.gimbal_dir * (servo_deg - kin_cal.gimbal_theta_offset) / kin_cal.gimbal_gain;
    float full = 360.0f / kin_cal.gimbal_gain;   /* 一圈舵机对应的 θ (gain>1 时略小于 360) */
    if (th >  full * 0.5f) th -= full;        /* 回绕后仍解回正确 θ */
    if (th < -full * 0.5f) th += full;
    return th;
}

float Kin_ExtendMmToDeg(float r_mm)
{
    return kin_cal.extend_zero_deg + kin_cal.extend_sign * (r_mm - kin_cal.extend_r_min_mm) / kin_cal.extend_mm_per_deg;
}
float Kin_ExtendDegToMm(float deg)
{
    return kin_cal.extend_r_min_mm + kin_cal.extend_sign * kin_cal.extend_mm_per_deg * (deg - kin_cal.extend_zero_deg);
}

float Kin_HoistMmToDeg(float z_mm)
{
    return kin_cal.hoist_zero_deg + kin_cal.hoist_sign * (z_mm - kin_cal.hoist_z_min_mm) / kin_cal.hoist_mm_per_deg;
}
float Kin_HoistDegToMm(float deg)
{
    return kin_cal.hoist_z_min_mm + kin_cal.hoist_sign * kin_cal.hoist_mm_per_deg * (deg - kin_cal.hoist_zero_deg);
}

/* ================= 正解 (θ, r, z) -> (x, y, z) ================= */
void Kin_FK(float theta_deg, float r_mm, float z_mm, float *x, float *y, float *z)
{
    float t  = theta_deg * KIN_DEG2RAD;
    float ph = kin_cal.extend_rail_phi_deg * KIN_DEG2RAD;
    float s  = r_mm - kin_cal.extend_r_min_mm;             /* 伸出量 (mm), r_min 时=0 */
    float xg = kin_cal.extend_rail_x0_mm + s * cosf(ph);   /* θ=0 系夹爪 */
    float yg = kin_cal.extend_rail_y0_mm + s * sinf(ph);

    *x = xg * cosf(t) - yg * sinf(t);   /* 云台转 θ 后到世界系 */
    *y = xg * sinf(t) + yg * cosf(t);
    *z = z_mm;
}

/* ================= 逆解 (x, y, z) -> (θ, r, z) =================
 * 返回 1=可达(未截断), 0=超出工作空间(已截断到边界, 上层据此判断目标不可达)
 * 含轨道偏置: 夹爪在 θ=0 系沿斜线 (x0 + s·cosφ, y0 + s·sinφ), 世界 = R(θ)·该点。
 * 解 ρ·sin(θ+φ-β)=c 的主支 (c=x0·sinφ-y0·cosφ, β=atan2(y,x), ρ=√(x²+y²)) → 夹爪朝外(s>0)。 */
uint8_t Kin_IK(float x, float y, float z, float *theta_deg, float *r_mm, float *z_mm)
{
    float rho  = sqrtf(x * x + y * y);
    float beta = atan2f(y, x);                          /* 目标方位角 */
    float ph   = kin_cal.extend_rail_phi_deg * KIN_DEG2RAD;
    float c    = kin_cal.extend_rail_x0_mm * sinf(ph) - kin_cal.extend_rail_y0_mm * cosf(ph);  /* 导轨线到转轴的带符号垂直偏置 */
    float arg, th, xg, yg, s, r;
    float zz = z;
    uint8_t ok = 1;

    arg = c / rho;
    if (arg >  1.0f) arg =  1.0f;   /* 目标太靠近转轴, 夹住 asin 防 NaN (下面 r 会截到 r_min) */
    if (arg < -1.0f) arg = -1.0f;

    th = (beta - ph + asinf(arg)) * KIN_RAD2DEG;

    /* 反转回 θ=0 系, 投影到导轨求伸出量 s -> r */
    xg =  x * cosf(th * KIN_DEG2RAD) + y * sinf(th * KIN_DEG2RAD);
    yg = -x * sinf(th * KIN_DEG2RAD) + y * cosf(th * KIN_DEG2RAD);
    s  = (xg - kin_cal.extend_rail_x0_mm) * cosf(ph) + (yg - kin_cal.extend_rail_y0_mm) * sinf(ph);
    r  = kin_cal.extend_r_min_mm + s;

    if (r  < kin_cal.extend_r_min_mm) { r  = kin_cal.extend_r_min_mm; ok = 0; }
    if (r  > kin_cal.extend_r_max_mm) { r  = kin_cal.extend_r_max_mm; ok = 0; }
    if (zz < kin_cal.hoist_z_min_mm)  { zz = kin_cal.hoist_z_min_mm;  ok = 0; }
    if (zz > kin_cal.hoist_z_max_mm)  { zz = kin_cal.hoist_z_max_mm;  ok = 0; }

    *theta_deg = th;
    *r_mm      = r;
    *z_mm      = zz;
    return ok;
}

/* ================= 点到点运动 (安全顺序) ================= */

/* 上层给 (x,y), z 用 kin_cal.grab_z_mm; 启动运动顺序 */
void Kin_GotoXY(float x, float y)
{
    float th, r, zz;
    y -= kin_cal.grab_y_offset_mm;   /* 装货: 视觉 y 大 3cm, 补 -30mm 再解算 */

    kin_flag.reachable = Kin_IK(x, y, kin_cal.grab_z_mm, &th, &r, &zz);   /* 含轨道偏置的逆解 */

    kin_motion.motion_count++;   /* 触发计数+1: Watch 看它确认重触发有没有进来 */
    kin_tgt.last_tx = x;      /* 记录本次目标, 供自动触发比较 */
    kin_tgt.last_ty = y;

    r -= kin_cal.extend_r_trim_mm;                        /* 魔法: r 长一截, 减修正 */
    if (r < kin_cal.extend_r_min_mm) r = kin_cal.extend_r_min_mm;
    if (r > kin_cal.extend_r_max_mm) r = kin_cal.extend_r_max_mm;

    kin_tgt.target_theta = th;
    kin_tgt.target_r     = r;
    kin_tgt.target_z_mm  = kin_cal.grab_z_mm;

    kin_tgt.servo       = Kin_ThetaToServo(th);   /* 已归一化到 [0,360), 负角回绕 */
    kin_tgt.m6_deg      = Kin_ExtendMmToDeg(r);
    if (s_kin_test_direct) { kin_tgt.servo = s_kin_test_servo; kin_tgt.m6_deg = s_kin_test_m6; s_kin_test_direct = 0; }  /* 测试: 直接角度覆盖 */
    kin_tgt.m5_deg      = kin_cal.grab_z_mm;          /* 抓取高度(回转平台460), 云台+电机6 先动, 电机5 最后降 */

    kin_flag.step      = KIN_STEP_APPROACH_GRAB;   /* 云台+电机6 先到(θ,r), 再电机5 降到抓取高度 */
    kin_flag.phase     = 0;                      /* 下一拍先发命令 */
    kin_motion.keepalive = 0;
    kin_flag.need_send = 1;
    kin_flag.done      = 0;
    kin_flag.active    = 1;
}

/* 卸货启动 (kin_flag.cmd=3): 平台抓取(310/447/500) -> 600高位(135/600/0) -> 等 0x0C 落点 -> 依次放 3 个 -> 到落点后降50 -> 等放开 */
void Kin_UnloadStart(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++) { kin_tgt.drop_actual_x[i] = 0.0f; kin_tgt.drop_actual_y[i] = 0.0f; }   /* 清上轮记录 */
    kin_flag.reachable  = 1;
    kin_motion.motion_count++;   /* 触发计数: 确认重触发有没有进来 */
    kin_flag.unload_idx  = 0;
    kin_flag.unload_lock = 1;   /* 锁定坐标: 探查位已收到 0x0C 则立即锁死不再更新; 没收到才等第一帧 */
    kin_flag.step       = KIN_STEP_UNLOAD_GRAB_HOIST_UP;   /* m5 升到平台600 -> 云台310+电机6 500 -> 自动夹取 */
    kin_flag.phase      = 0;
    kin_motion.keepalive  = 0;
    kin_flag.need_send  = 1;
    kin_flag.done       = 0;
    kin_flag.active     = 1;
}

/* 回装启动 (kin_flag.cmd=7): 与卸货相反, 用已锁存的 0x0C 三个落点, 反序 3->2->1 从平地抓回,
   依次放回物料平台。要求卸货已给过坐标(unload_have=1), 否则不动。 */
void Kin_ReloadStart(void)
{
    if (!kin_flag.unload_have) return;   /* 还没落点坐标, 无从抓回 */

    {
        uint8_t i;
        for (i = 0; i < 3; i++) { kin_tgt.reload_actual_x[i] = 0.0f; kin_tgt.reload_actual_y[i] = 0.0f; }   /* 清上轮记录 */
    }
    kin_flag.reachable  = 1;
    kin_motion.motion_count++;
    kin_flag.unload_idx  = 0;      /* 复用计数 0..2; 落点序号在 Kin_ReloadDropTarget 里反序取 drop_ring[2-idx] */
    kin_flag.unload_lock = 1;      /* 锁定坐标, 整轮用同一份 0x0C */
    kin_flag.platform_sel = 2;     /* 回原槽反序: 落点3->C, 落点2->B, 落点1->A; 先把平台转到C */
    Servo_SetAngle(SERVO_PLATFORM, PLATFORM_ANGLE_C);
    kin_flag.step       = KIN_STEP_RLD_HOIST_UP;   /* 先升640清障 */
    kin_flag.phase      = 0;
    kin_motion.keepalive  = 0;
    kin_flag.need_send  = 1;
    kin_flag.done       = 0;
    kin_flag.active     = 1;
}

/* 卸货落点目标解算(只算, 不启动运动): 用 0x0C 第 drop_ring[unload_idx] 号环落点逆解出
 * (θ, r, servo, m6_deg) 存进 kin_tgt。收回步落点已知时提前调它, 云台直走落点方向。 */
static void Kin_UnloadDropTarget(void)
{
    uint8_t ring = kin_tgt.drop_ring[kin_flag.unload_idx];   /* 该抓取槽位对应的环号(1~3), mission 按二维码环序填 */
    float x  = kin_tgt.unload_x[ring - 1] + kin_cal.unload_x_offset_mm;   /* 落点 x +偏移 +底盘停靠 corr_x */
    float y  = kin_tgt.unload_y[ring - 1] - kin_cal.unload_y_offset_mm;   /* 落点 y +偏移 +底盘停靠 corr_y */
    float th, r, zz;

    kin_flag.reachable = Kin_IK(x, y, 0.0f, &th, &r, &zz);   /* 含轨道偏置的逆解 (z 由电机5 单独控制, 这里不用) */

    kin_tgt.last_tx = x; kin_tgt.last_ty = y;
    kin_tgt.drop_actual_x[kin_flag.unload_idx] = x;   /* 记下本轮实际前往落点(供 Watch 看放第几个) */
    kin_tgt.drop_actual_y[kin_flag.unload_idx] = y;

    r -= kin_cal.extend_r_trim_mm;                        /* 魔法: r 长一截, 减修正 */
    if (r < kin_cal.extend_r_min_mm) r = kin_cal.extend_r_min_mm;
    if (r > kin_cal.extend_r_max_mm) r = kin_cal.extend_r_max_mm;

    kin_tgt.target_theta = th;
    kin_tgt.target_r     = r;
    if (s_kin_test_direct) { kin_tgt.servo = s_kin_test_servo; kin_tgt.m6_deg = s_kin_test_m6; }  /* 测试: 直接角度覆盖(消费点在 Kin_UnloadDrop) */
    else                   { kin_tgt.servo = Kin_ThetaToServo(th); kin_tgt.m6_deg = Kin_ExtendMmToDeg(r); }
}

/* 回装落点目标解算(只算, 不启动运动): 反序取 drop_ring[2-unload_idx] 号环落点逆解出
 * (θ, r, servo, m6_deg) 存进 kin_tgt。与 Kin_UnloadDropTarget 相同, 仅落点序号反序(3->2->1)。 */
static void Kin_ReloadDropTarget(void)
{
    uint8_t ring = kin_tgt.drop_ring[2 - kin_flag.unload_idx];   /* 反序: idx=0->环3, 1->环2, 2->环1 */
    float x  = kin_tgt.unload_x[ring - 1] + kin_cal.unload_x_offset_mm;
    float y  = kin_tgt.unload_y[ring - 1] - kin_cal.unload_y_offset_mm;
    float th, r, zz;

    kin_flag.reachable = Kin_IK(x, y, 0.0f, &th, &r, &zz);

    kin_tgt.last_tx = x; kin_tgt.last_ty = y;
    kin_tgt.reload_actual_x[kin_flag.unload_idx] = x;   /* 记下本轮实际前往落点(供 Watch 看抓第几个) */
    kin_tgt.reload_actual_y[kin_flag.unload_idx] = y;

    r -= kin_cal.extend_r_trim_mm;
    if (r < kin_cal.extend_r_min_mm) r = kin_cal.extend_r_min_mm;
    if (r > kin_cal.extend_r_max_mm) r = kin_cal.extend_r_max_mm;

    kin_tgt.target_theta = th;
    kin_tgt.target_r     = r;
    kin_tgt.servo   = Kin_ThetaToServo(th);
    kin_tgt.m6_deg  = Kin_ExtendMmToDeg(r);
}

/* 卸货放置: 解落点 -> 云台θ + m6 r (m5 停在640清障高), 到位后降落点高再等放开 */
void Kin_UnloadDrop(void)
{
    Kin_UnloadDropTarget();
    s_kin_test_direct = 0;   /* 消费一次性直接角度覆盖 */

    kin_flag.step      = KIN_STEP_UNLOAD_DROP_APPROACH;   /* 云台+电机6 一起到落点(θ,r), 电机5 停在640清障高, 到位后再降 */
    kin_flag.phase     = 0;
    kin_motion.keepalive = 0;
    kin_flag.need_send = 1;
}

/* 通用姿态切换: 塔吊走到指定 (云台, 电机5, 电机6)。
 * 安全顺序: m5->640(先升清障) -> 云台+电机6 一起到目标 -> m5->目标。
 * 用于: 上电引导(运动姿态310/600/500)、工作区探查姿态(135/600/0)、离站回运动姿态(310/600/500)。 */
void Kin_GotoPosture(float gimbal_deg, float m5_deg, float m6_deg)
{
    s_pose_gimbal = gimbal_deg;
    s_pose_m5     = m5_deg;
    s_pose_m6     = m6_deg;
    s_pose_after  = 0;          /* 默认到位即完成; 要续步的调用者在本函数返回后再置 */
    kin_flag.reachable = 1;
    kin_flag.step      = KIN_STEP_POSE_HOIST_UP;
    kin_flag.phase     = 0;
    kin_motion.keepalive = 0;
    kin_flag.need_send = 1;
    kin_flag.done      = 0;
    kin_flag.active    = 1;
}

/* 去工作区"探查姿态": 云台135 / 电机5 600 / 电机6 0 (粗加工/暂存) */
void Kin_GoDefault(void)
{
    s_probe_is_material = 0;
    Kin_GotoPosture(kin_cal.pose_default_gimbal_deg, kin_cal.pose_default_hoist_deg, kin_cal.pose_default_extend_deg);
}

/* 去物料区专属"探查姿态": 云台135 / 电机5 600 / 电机6 200 (只物料区用, 只摆姿态不抓) */
void Kin_GoMaterialProbe(void)
{
    s_probe_is_material = 1;
    Kin_GotoPosture(kin_cal.pose_material_gimbal_deg, kin_cal.pose_material_hoist_deg,
                    kin_cal.pose_material_extend_deg);
}

/* 是否已经站在物料区探查姿态 135/460/200 (云台命令值 + 电机5/6 最近读回值, 容差放宽一倍)。
   读回没更新(上电后没动过)时一定不匹配 -> 调用方退回完整安全顺序, 不会盲夹/空摆。 */
static uint8_t Kin_AtMaterialPose(void)
{
    return (fabsf(Servo_GetAngle(SERVO_GIMBAL) - kin_cal.pose_material_gimbal_deg) <= kin_motion.gimbal_tol_deg * 2.0f &&
            fabsf(fabsf(motor5_pos) - kin_cal.pose_material_hoist_deg) <= kin_motion.pos_tol_mm * 2.0f &&
            fabsf(fabsf(motor6_pos) - kin_cal.pose_material_extend_deg) <= kin_motion.pos_tol_mm * 2.0f) ? 1 : 0;
}

/* 物料区固定位完整装货 (kin_flag.cmd=10, cmd=12 复用):
   摆物料区探查姿态 135/460/200 (夹爪张开, 电机5 就站在抓取高) ->
   等授权 (kin_flag.mat_grab_cmd 或视觉 0x09 ok) -> 直接闭夹爪 ->
   升640清障 -> 云台放置角+电机6 放置伸出 -> 降到600 -> 松夹爪 -> 平台下一档 -> 回物料区探查姿态。
   cmd=9 只摆姿态不抓, 可单步验证; cmd=12 = 同一条链, 但回探查姿态后接着等下一个 0x09, 连抓
   MATERIAL_CHAIN_GRABS 件 (见 PLACE_RELEASE 里的 s_mat_chain_left)。 */
void Kin_MaterialGrabStart(void)
{
    s_probe_is_material = 1;
    s_mat_chain_left    = 0;   /* 默认单件: 放完就结束。cmd=12 在调用本函数之后再置连抓次数 */

    /* 已经站在物料区探查姿态(135/460/200)时不重走"升640->云台+电机6->降460"安全顺序,
       直接进等授权: 收到指令就闭爪升上去 (mission 里 cmd=9 摆完姿态再来的就是这种情况,
       省掉一次先升后降, 镜头不晃)。不在位/读回没更新 -> 走完整安全顺序, 不会盲夹。 */
    if (Kin_AtMaterialPose())
    {
        s_pose_after         = 0;
        kin_flag.reachable   = 1;
        kin_flag.done        = 0;
        kin_flag.active      = 1;
        kin_flag.phase       = 0;
        kin_flag.need_send   = 0;
        kin_motion.keepalive = 0;
        kin_flag.step        = KIN_STEP_MAT_WAIT_GRAB;
        return;
    }

    Kin_GotoPosture(kin_cal.pose_material_gimbal_deg, kin_cal.pose_material_hoist_deg,
                    kin_cal.pose_material_extend_deg);
    s_pose_after = KIN_STEP_MAT_WAIT_GRAB;   /* 姿态到位后直接等抓取授权 (不下降) */
}

/* 物料区"探查姿态 + 底盘微调"单步测试 (kin_flag.cmd=11):
   摆 135/460/200 期间挂起底盘视觉纠偏 (chassis_flag.trim_hold=1),
   姿态到位后再放开, 等视觉给出新的 within=1 才置 kin_flag.done=1。
   和 mission 的 MS_TRIM 用同一套接口 (Chassis_TrimArm/Release/Done)。
   视觉在该区不发 0x04/0x05 时会一直等 (Watch 看 kin_flag.step==55), 写 cmd=2 可停。
   用途: 单独原地验证"到物料区探查姿态 -> 底盘微调到位"这一步, 不走 mission。 */
void Kin_MaterialProbeTrimStart(void)
{
    s_probe_is_material = 1;
    s_trim_released     = 0;
    s_trim_owned        = 1;
    Chassis_TrimArm();      /* 武装: 清 within, 姿态期间不收帧 */

    /* 已经在探查姿态: 不重走升640往返, 直接进等微调 (视觉镜头不晃) */
    if (Kin_AtMaterialPose())
    {
        s_pose_after         = 0;
        kin_flag.reachable   = 1;
        kin_flag.done        = 0;
        kin_flag.active      = 1;
        kin_flag.phase       = 0;
        kin_flag.need_send   = 0;
        kin_motion.keepalive = 0;
        kin_flag.step        = KIN_STEP_WAIT_CHASSIS;
        return;
    }

    Kin_GotoPosture(kin_cal.pose_material_gimbal_deg, kin_cal.pose_material_hoist_deg,
                    kin_cal.pose_material_extend_deg);
    s_pose_after = KIN_STEP_WAIT_CHASSIS;   /* 姿态到位后接着等底盘微调 */
}

/* 回"运动姿态": 云台310 / 电机5 600 / 电机6 500 (行驶/换工作点) */
void Kin_GoBoot(void)
{
    Kin_GotoPosture(kin_cal.pose_boot_gimbal_deg, kin_cal.pose_boot_hoist_deg, kin_cal.pose_boot_extend_deg);
}

/* 前往坐标测试 (kin_flag.cmd=6): (x,y,z)mm 逆解 -> (θ,r,z) -> (云台/电机6/电机5 度),
   走 Kin_GotoPosture 安全顺序(电机5 先升640清障 -> 云台+电机6 一起 -> 电机5 降目标高)。
   只到位、不动夹爪, 供验证正/逆解与标定。 */
void Kin_GotoPointXYZ(float x, float y, float z)
{
    float th, r, zz;

    kin_flag.reachable = Kin_IK(x, y, z, &th, &r, &zz);

    kin_tgt.target_x     = x;
    kin_tgt.target_y     = y;
    kin_tgt.target_theta = th;
    kin_tgt.target_r     = r;
    kin_tgt.target_z_mm  = zz;

    kin_motion.motion_count++;
    Kin_GotoPosture(Kin_ThetaToServo(th), Kin_HoistMmToDeg(zz), Kin_ExtendMmToDeg(r));
}

/* 上电引导位姿: 夹爪开 + 物料平台回 A 档, 塔吊到 Tower_Boot_Task 姿态 310/600/500 (不夹取) */
void Kin_BootPosture(void)
{
    Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);   /* 夹爪开 */
    kin_flag.platform_sel = 0;
    Servo_SetAngle(SERVO_PLATFORM, PLATFORM_ANGLE_A);        /* 物料平台回 A 档 */
    Kin_GoBoot();
}

void Kin_Stop(void)
{
    kin_flag.active = 0;
    kin_flag.done   = 0;
    kin_flag.step   = KIN_STEP_IDLE;
    s_pose_after = 0;            /* 取消"姿态到位后续走"链 */
    s_mat_chain_left = 0;        /* 取消 cmd=12 连抓: 停止后不再自动进下一件 */
    s_pick_pending_report = 0;   /* 停止: 不补报没发出去的 0x0F (下一场从 mission_pick_seq 重新对) */
    s_trim_released = 0;
    s_trim_owned    = 0;
    Chassis_TrimDisarm();        /* 手动停止: 解除底盘微调武装, 退回自动纠偏 */
    kin_flag.unload_lock = 0;   /* 手动停止也解除锁定, 恢复坐标更新 */
    X_V2_Stop_Now(MOTOR_HOIST,  false);
    X_V2_Stop_Now(MOTOR_EXTEND, false);
}

/**
  * @brief  运动状态机 (每 10ms 调用)
  *         - kin_flag.cmd: 1=装货抓取 GotoXY(用 kin_tgt.target_x/y); 卸货等视觉时 1=到位等放开;
  *                    2=停止; 3=卸货(平台抓取600->落点高->等视觉); 4=探查姿态; 5=运动姿态;
  *                    6=前往坐标(kin_tgt.goto_x/y/z, 纯到位不夹取); 7=回装; 8=堆垛(码垛高);
  *                    9=物料区探查姿态 135/460/200 (只摆姿态); 10=物料区固定位完整装货
  *                    (摆姿态(即在抓取高) -> 等 mat_grab_cmd/0x09 -> 闭爪 -> 放置 -> 回物料区探查姿态);
  *                    11=物料区探查姿态 + 底盘微调 (姿态到位放开纠偏, 等 within=1 后完成);
  *                    12=物料区连抓 MATERIAL_CHAIN_GRABS 件 (每件等一个新的 0x09, 放完回探查位继续)
  *                    处理后回 0。
  *         - 抓取顺序: 云台+电机6 先到(θ,r) -> 电机5 降到抓取高度 -> 张夹爪 -> 等ok -> 闭夹爪
  *         - 电机5/6 发命令与读位置交替, 防"位置命令+读位置"背靠背冲突
  *         - 走完 -> kin_flag.active=0, kin_flag.done=1
  */
void Kin_Update(void)
{
    /* 房务: 处理 Watch 写的 mission_color.clr=1 (清空物料颜色记录, 手动测试重开一场用) */
    Mission_ColorTick();

    /* 探查姿态: 通用(cmd=4) 夹爪闭合 (清视觉视场); 物料区(cmd=9/11) 夹爪张开,
       就在抓取高等指令, 收到授权(0x09 / mat_grab_cmd)才闭合。 */
    if (kin_flag.cmd == 4)
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_close_angle);
    else if (kin_flag.cmd == 1 || kin_flag.cmd == 3 || kin_flag.cmd == 5 ||
             kin_flag.cmd == 6 || kin_flag.cmd == 7 || kin_flag.cmd == 8 ||
             kin_flag.cmd == 9 || kin_flag.cmd == 10 || kin_flag.cmd == 11 ||
             kin_flag.cmd == 12)
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);

    /* cmd=11 等底盘微调期间被别的命令打断: 必须交还底盘纠偏, 否则 trim_hold 会一直卡 1 */
    if (s_trim_owned && kin_flag.cmd != 0 && kin_flag.cmd != 2 && kin_flag.cmd != 11)
    {
        s_trim_owned    = 0;
        s_trim_released = 0;
        Chassis_TrimDisarm();
    }

    if (kin_flag.cmd == 1)
    {
        kin_flag.cmd = 0;
        if (kin_flag.step == KIN_STEP_UNLOAD_WAIT_VISION) { if (kin_flag.unload_have) Kin_UnloadDrop(); }  /* 卸货: 兜底手动触发(须已有0x0C落点) */
        else                                         Kin_GotoXY(kin_tgt.target_x, kin_tgt.target_y);  /* 装货: 抓取 */
    }
    else if (kin_flag.cmd == 2) { kin_flag.cmd = 0; Kin_Stop(); }
    else if (kin_flag.cmd == 3) { kin_flag.cmd = 0; Kin_UnloadStart(); }   /* 卸货: 平台抓取 -> 50默认 */
    else if (kin_flag.cmd == 4) { kin_flag.cmd = 0; Kin_GoDefault(); }     /* 去工作区探查姿态 134/600/0 */
    else if (kin_flag.cmd == 5) { kin_flag.cmd = 0; Kin_GoBoot(); }        /* 回 Tower_Boot_Task 姿态 310/600/500 */
    else if (kin_flag.cmd == 6) { kin_flag.cmd = 0; Kin_GotoPointXYZ(kin_tgt.goto_x, kin_tgt.goto_y, kin_tgt.goto_z); }  /* 前往坐标测试 */
    else if (kin_flag.cmd == 7) { kin_flag.cmd = 0; Kin_ReloadStart(); }   /* 一键回装: 反序3->2->1 平地抓回放平台 */
    else if (kin_flag.cmd == 8) { kin_flag.cmd = 0; kin_cal.drop_hoist_deg = HOIST_HEIGHT_PALLET; Kin_UnloadStart(); }   /* 一键堆垛: 码垛高放 0x0C 三落点 */
    else if (kin_flag.cmd == 9) { kin_flag.cmd = 0; Kin_GoMaterialProbe(); }     /* 物料区探查姿态 135/460/200 (只摆姿态) */
    else if (kin_flag.cmd == 10){ kin_flag.cmd = 0; Kin_MaterialGrabStart(); }   /* 物料区固定位完整装货 */
    else if (kin_flag.cmd == 11){ kin_flag.cmd = 0; Kin_MaterialProbeTrimStart(); } /* 物料区探查姿态 + 等底盘微调 */
    /* 物料区连抓: 同 cmd=10, 但每放完一件回探查姿态后继续等下一个 0x09, 共 MATERIAL_CHAIN_GRABS 件。
       必须写在 Kin_MaterialGrabStart() 之后: 该函数开头会把 s_mat_chain_left 清 0 (单次语义)。 */
    else if (kin_flag.cmd == 12){ kin_flag.cmd = 0; Kin_MaterialGrabStart(); s_mat_chain_left = MATERIAL_CHAIN_GRABS - 1; }

    /* 0x0C 卸货坐标: 未锁定时持续用最新帧更新; 回去夹东西后锁定不再更新
     * (若还没拿到过坐标, 仍接受第一帧, 避免夹完却无坐标可用卡死) */
    if (Serial_UnloadCoordReady)
    {
        Serial_UnloadCoordReady = 0;
        if (!kin_flag.unload_lock || !kin_flag.unload_have)
        {
            uint8_t i, n;
            n = Serial_RxData.unload_count;
            if (n > 3) n = 3;
            for (i = 0; i < n; i++)
            {
                kin_tgt.unload_x[i] = Serial_RxData.unload_x[i];
                kin_tgt.unload_y[i] = Serial_RxData.unload_y[i];
            }
            kin_flag.unload_have = 1;
        }
    }

    /* 只由 kin_flag.cmd=1 触发运动: 坐标(kin_tgt.target_x/y)变了不会自己动, 必须手动给 cmd */
    if (!kin_flag.active) return;

    /* 云台 + 电机6 并行步: 每拍斜坡云台(PWM), 同时电机6 走 CAN 收发交替; 两轴都到位才推进 */
    if (kin_flag.step == KIN_STEP_APPROACH_GRAB || kin_flag.step == KIN_STEP_APPROACH_PLACE ||
        kin_flag.step == KIN_STEP_UNLOAD_GRAB_APPROACH || kin_flag.step == KIN_STEP_UNLOAD_RETRACT ||
        kin_flag.step == KIN_STEP_UNLOAD_DROP_APPROACH || kin_flag.step == KIN_STEP_POSE_DUAL ||
        kin_flag.step == KIN_STEP_RLD_APPROACH_DROP || kin_flag.step == KIN_STEP_RLD_APPROACH_PLATFORM)
    {
        uint8_t next;
        float   g_tgt, m6_tgt;
        float   sv, err;
        uint8_t gimbal_done;
        float   place_gimbal_deg = (kin_flag.platform_sel == 2) ? kin_cal.place_gimbal_deg_c : kin_cal.place_gimbal_deg;  /* 平台C档抓放角 305, A/B 310 */

        switch (kin_flag.step)
        {
        case KIN_STEP_APPROACH_GRAB:         g_tgt = kin_tgt.servo;        m6_tgt = kin_tgt.m6_deg;        next = KIN_STEP_REACH_HOIST;       break;
        case KIN_STEP_APPROACH_PLACE:        g_tgt = place_gimbal_deg; m6_tgt = kin_cal.place_extend2_deg; next = KIN_STEP_PLACE_HOIST;       break;
        case KIN_STEP_UNLOAD_GRAB_APPROACH:  g_tgt = place_gimbal_deg; m6_tgt = kin_cal.place_extend2_deg; next = KIN_STEP_UNLOAD_GRIP;       break;
        case KIN_STEP_UNLOAD_RETRACT:        g_tgt = s_retract_gimbal;         m6_tgt = kin_cal.home_extend_deg;   next = KIN_STEP_UNLOAD_WAIT_VISION; break;
        case KIN_STEP_UNLOAD_DROP_APPROACH:  g_tgt = kin_tgt.servo;        m6_tgt = kin_tgt.m6_deg;        next = KIN_STEP_UNLOAD_DROP_HOIST; break;
        case KIN_STEP_RLD_APPROACH_DROP:     g_tgt = kin_tgt.servo;        m6_tgt = kin_tgt.m6_deg;        next = KIN_STEP_RLD_DROP_HOIST;       break;
        case KIN_STEP_RLD_APPROACH_PLATFORM: g_tgt = place_gimbal_deg;     m6_tgt = kin_cal.place_extend2_deg; next = KIN_STEP_RLD_PLATFORM_HOIST; break;
        default /* KIN_STEP_POSE_DUAL */:    g_tgt = s_pose_gimbal;        m6_tgt = s_pose_m6;              next = KIN_STEP_POSE_HOIST;       break;
        }

        Servo_MoveTo(SERVO_GIMBAL, g_tgt, kin_motion.gimbal_step_deg, kin_motion.gimbal_acc, kin_motion.gimbal_dec);   /* 梯形加减速, 单圈舵机直走不跨缝 */
        sv = Servo_GetAngle(SERVO_GIMBAL);
        err = fabsf(g_tgt - sv);   /* 单圈舵机: 直接差判到位, 不按 0/360 取近路 */
        gimbal_done = (err <= kin_motion.gimbal_tol_deg);

        kin_flag.phase ^= 1;
        if (kin_flag.phase)
        {
            if (kin_flag.need_send || ++kin_motion.keepalive >= 100)
            {
                kin_flag.need_send = 0;
                kin_motion.keepalive = 0;
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_flag.extend_cmd_dir, kin_motion.extend_acc, kin_motion.extend_dec,
                                      kin_motion.extend_vel, m6_tgt, 1, 0);
            }
        }
        else
        {
            Tower_ReadPos6();
            if (gimbal_done && fabsf(fabsf(motor6_pos) - fabsf(m6_tgt)) <= kin_motion.pos_tol_mm)
            {
                kin_flag.step = next;
                kin_flag.need_send = 1;
                kin_motion.keepalive = 0;
            }
        }
        return;
    }

    /* 等抓取授权: 视觉 GrabCmd(0x09) 或手动 kin_flag.descend_cmd, 二选一 -> 直接闭夹爪 (不预降) */
    if (kin_flag.step == KIN_STEP_WAIT_GRAB)
    {
        if (grab_cmd || kin_flag.descend_cmd || kin_flag.mat_grab_cmd)
        {
            grab_cmd = 0;
            Serial_RxData.grab_cmd = 0;   /* Watch: 授权已被消费, 不再挂 1 (grab_cmd_cnt 仍保留到达次数) */
            kin_flag.descend_cmd = 0;
            kin_flag.mat_grab_cmd = 0;
            kin_flag.step = KIN_STEP_WAIT_GRIP;
        }
        return;
    }

    /* 等夹爪闭合: 全自动(kin_flag.auto_grip) 或 手动 kin_flag.grip_cmd -> 闭夹爪 -> 进入放置 */
    if (kin_flag.step == KIN_STEP_WAIT_GRIP)
    {
        if (kin_flag.auto_grip || kin_flag.grip_cmd)
        {
            kin_flag.grip_cmd = 0;
            Mission_NoteGrabColor(Serial_RxData.target_color_id);   /* 拿到一件物料: 记颜色 (手动/自动都记) */
            s_pick_color = Serial_RxData.target_color_id;           /* 锁存本件颜色, 回位上报 0x0F 用 */
            Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_close_angle);   /* 闭夹爪抓取 */
            kin_flag.step = KIN_STEP_PLACE_HOIST_UP; kin_flag.need_send = 1; kin_motion.keepalive = 0;  /* 先升电机5 到640清障 */
        }
        return;
    }

    /* 物料区固定位装货: 探查姿态就已经站在抓取高(460), 夹爪张开。
       等授权 (kin_flag.mat_grab_cmd 自动/手动 或 视觉 0x09 ok) -> 直接闭夹爪 -> 升640去放置。
       没有"下降"这一步。 */
    if (kin_flag.step == KIN_STEP_MAT_WAIT_GRAB)
    {
        if (kin_flag.mat_grab_cmd || grab_cmd)
        {
            kin_flag.mat_grab_cmd = 0;
            grab_cmd = 0;
            Serial_RxData.grab_cmd = 0;   /* Watch: 授权已被消费, 不再挂 1 (grab_cmd_cnt 仍保留到达次数) */
            Mission_NoteGrabColor(Serial_RxData.target_color_id);   /* 拿到一件物料: 记颜色 (cmd=10/12 连抓也记) */
            s_pick_color = Serial_RxData.target_color_id;           /* 锁存本件颜色, 回位上报 0x0F 用 */
            Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_close_angle);   /* 直接闭夹爪抓取 */
            kin_flag.step = KIN_STEP_PLACE_HOIST_UP;   /* 先升电机5 到640清障, 再放置 */
            kin_flag.need_send = 1;
            kin_motion.keepalive = 0;
        }
        return;
    }

    /* 物料区姿态 + 底盘微调 (kin_flag.cmd=11): 姿态到位 -> 放开底盘纠偏 -> 等 within=1 */
    if (kin_flag.step == KIN_STEP_WAIT_CHASSIS)
    {
        if (!s_trim_released) { Chassis_TrimRelease(); s_trim_released = 1; }  /* 姿态停稳后才开始收新帧 */
        if (Chassis_TrimDone())
        {
            s_trim_released = 0;
            s_trim_owned    = 0;
            kin_flag.active = 0;
            kin_flag.done   = 1;
            kin_flag.step   = KIN_STEP_IDLE;
        }
        return;
    }

    /* 卸货夹取: 云台到位后先稳停(云台无回读, Servo_GetAngle 是命令值, 物理还没稳住),
       再闭夹爪, 稍等让舵机闭合, 再升640清障 —— 避免"还没到物料上方就夹" */
    if (kin_flag.step == KIN_STEP_UNLOAD_GRIP)
    {
        static uint16_t ugrip_tick = 0;
        if (++ugrip_tick < kin_motion.gimbal_settle_10ms) return;   /* 停在物料上方等云台物理稳住 */
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_close_angle);   /* 自动闭夹爪夹取 */
        if (ugrip_tick >= kin_motion.gimbal_settle_10ms + kin_motion.gripper_delay_10ms)
        {
            ugrip_tick = 0;
            kin_flag.step = KIN_STEP_UNLOAD_RETURN_HOIST_UP; kin_flag.need_send = 1; kin_motion.keepalive = 0;  /* 先升电机5 到640清障 */
        }
        return;
    }

    /* 回装抓取: 降到落点后先稳停(云台无回读), 再闭夹爪, 稍等, 再升640清障 */
    if (kin_flag.step == KIN_STEP_RLD_GRIP)
    {
        static uint16_t rgrip_tick = 0;
        if (++rgrip_tick < kin_motion.gimbal_settle_10ms) return;   /* 停在落点上方等云台物理稳住 */
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_close_angle);   /* 闭夹爪抓回 */
        if (rgrip_tick >= kin_motion.gimbal_settle_10ms + kin_motion.gripper_delay_10ms)
        {
            rgrip_tick = 0;
            kin_flag.step = KIN_STEP_RLD_RETURN_HOIST_UP; kin_flag.need_send = 1; kin_motion.keepalive = 0;   /* 先升电机5 到640清障 */
        }
        return;
    }

    /* 卸货: 收回后停在640清障高度等坐标就绪(坐标在后台持续更新, 回去夹东西时已锁定) -> 直接去放 */
    if (kin_flag.step == KIN_STEP_UNLOAD_WAIT_VISION)
    {
        if (kin_flag.unload_have)
            Kin_UnloadDrop();
        return;
    }

    /* 卸货放开: 全自动(kin_flag.auto_release) 或 手动 kin_flag.release_cmd -> 松夹爪 */
    if (kin_flag.step == KIN_STEP_UNLOAD_WAIT_RELEASE)
    {
        if (kin_flag.auto_release || kin_flag.release_cmd)
        {
            kin_flag.release_cmd = 0;
            Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);   /* 松夹爪 */
            kin_flag.step = KIN_STEP_UNLOAD_RELEASE;
        }
        return;
    }

    /* 卸货完成: 稍等让舵机动作, 平台转下一档(C->B->A); 还有落点则回平台抓下一个环, 放满 3 个回待命 */
    if (kin_flag.step == KIN_STEP_UNLOAD_RELEASE)
    {
        static uint16_t urel_tick = 0;
        if (++urel_tick >= kin_motion.gripper_delay_10ms)
        {
            urel_tick = 0;
            kin_flag.platform_sel = (kin_flag.platform_sel + 1) % 3;   /* 卸货 A->B->C 清空 */
            Servo_SetAngle(SERVO_PLATFORM,
                           kin_flag.platform_sel == 0 ? PLATFORM_ANGLE_A :
                           kin_flag.platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);

            kin_flag.unload_idx++;
            if (!s_kin_single_cycle && kin_flag.unload_idx < 3)
            {
                /* 还有下一个落点: 回平台抓下一个环 (600清障 -> 云台310 -> ...) */
                kin_flag.step = KIN_STEP_UNLOAD_GRAB_HOIST_UP;
                kin_flag.need_send = 1;
                kin_motion.keepalive = 0;
            }
            else
            {
                kin_flag.unload_lock = 0;   /* 整轮放完: 恢复坐标更新, 等下次回去夹东西再锁定 */
                s_kin_single_cycle = 0;
                Kin_GoDefault();       /* 放满 3 个环(或测试单次放完) -> 回探查姿态 135/600/0 再完成 */
            }
        }
        return;
    }

    /* 放置完成: 夹爪张开, 稍等让舵机动作, 然后回待命 */
    if (kin_flag.step == KIN_STEP_PLACE_RELEASE)
    {
        static uint16_t rel_tick = 0;
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);   /* 松夹爪 */
        if (++rel_tick >= kin_motion.gripper_delay_10ms)
        {
            rel_tick = 0;
            /* 放完一次: 夹爪保持默认张开, 旋转一次物料平台(换下一槽位), 回待命等下次视觉坐标 */
            kin_flag.platform_sel = (kin_flag.platform_sel + 1) % 3;
            Servo_SetAngle(SERVO_PLATFORM,
                           kin_flag.platform_sel == 0 ? PLATFORM_ANGLE_A :
                           kin_flag.platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);
            /* 放完 -> 回本工作区探查姿态: 物料区 135/460/200, 其它 135/600/0 */
            /* 这一件物料(抓->放)结束: 挂上"待上报 0x0F", 等回位那拍再发 (见 POSE_HOIST)。
               cmd=1 装货(mission 推广)、cmd=10 单抓、cmd=12 连抓 都走这条放完路径。 */
            s_pick_pending_report = 1;
            if (s_probe_is_material)
            {
                Kin_GoMaterialProbe();
                if (s_mat_chain_left)   /* cmd=12 连抓: 回到探查位继续等下一个 0x09 (必须在上面之后, GotoPosture 会清 s_pose_after) */
                {
                    s_mat_chain_left--;
                    s_pose_after = KIN_STEP_MAT_WAIT_GRAB;
                }
            }
            else Kin_GoDefault();
        }
        return;
    }

    /* 回装放置: 松夹爪 -> 延时 -> 转平台上一槽位(C->B->A); 还有落点则反序抓下一个, 抓满 3 个回默认 */
    if (kin_flag.step == KIN_STEP_RLD_RELEASE)
    {
        static uint16_t rrel_tick = 0;
        Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);   /* 松夹爪放回平台 */
        if (++rrel_tick >= kin_motion.gripper_delay_10ms)
        {
            rrel_tick = 0;
            kin_flag.platform_sel = (kin_flag.platform_sel + 2) % 3;   /* 回装反序转平台 C->B->A */
            Servo_SetAngle(SERVO_PLATFORM,
                           kin_flag.platform_sel == 0 ? PLATFORM_ANGLE_A :
                           kin_flag.platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);

            kin_flag.unload_idx++;
            if (kin_flag.unload_idx < 3)
            {
                kin_flag.step = KIN_STEP_RLD_HOIST_UP;   /* 反序抓下一个落点 */
                kin_flag.need_send = 1;
                kin_motion.keepalive = 0;
            }
            else
            {
                kin_flag.unload_lock = 0;   /* 抓满 3 个: 恢复坐标更新 */
                Kin_GoDefault();            /* 回探查姿态 */
            }
        }
        return;
    }

    /* 电机步: 发命令 / 读位置 交替 */
    kin_flag.phase ^= 1;
    if (kin_flag.phase)
    {
        /* 发命令拍: 目标变化发一次 + 约1s兜底重发 */
        if (kin_flag.need_send || ++kin_motion.keepalive >= 100)
        {
            kin_flag.need_send = 0;
            kin_motion.keepalive = 0;
            switch (kin_flag.step)
            {
            case KIN_STEP_REACH_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_tgt.m5_deg, 1, 0);   /* 电机5 降到抓取高度 */
                break;
            case KIN_STEP_PLACE_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.place_hoist_up_deg, 1, 0);   /* 640 清障 */
                break;
            case KIN_STEP_PLACE_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.place_hoist_deg, 1, 0);   /* 600 放置 */
                break;
            case KIN_STEP_POSE_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.place_hoist_up_deg, 1, 0);   /* 640 清障 */
                break;
            case KIN_STEP_POSE_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, s_pose_m5, 1, 0);   /* 目标高 */
                break;
            case KIN_STEP_UNLOAD_GRAB_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.unload_grab_hoist_deg, 1, 0);   /* 600 平台抓取高 */
                break;
            case KIN_STEP_UNLOAD_RETURN_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.place_hoist_up_deg, 1, 0);   /* 640 清障 */
                break;
            case KIN_STEP_UNLOAD_DROP_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.drop_hoist_deg, 1, 0);   /* 降到落点高(100/400) */
                break;
            case KIN_STEP_RLD_HOIST_UP:
            case KIN_STEP_RLD_RETURN_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.place_hoist_up_deg, 1, 0);   /* 640 清障 */
                break;
            case KIN_STEP_RLD_DROP_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.reload_grab_hoist_deg, 1, 0);   /* 降到平地抓取高 */
                break;
            case KIN_STEP_RLD_PLATFORM_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_flag.hoist_cmd_dir, kin_motion.hoist_acc, kin_motion.hoist_dec,
                                      kin_motion.hoist_vel, kin_cal.unload_grab_hoist_deg, 1, 0);   /* 降到平台抓取高(放回平台) */
                break;
            default: break;
            }
        }
    }
    else
    {
        /* 读位置拍: 判当前步到位, 推进下一步 */
        switch (kin_flag.step)
        {
        case KIN_STEP_REACH_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_tgt.m5_deg) <= kin_motion.pos_tol_mm)
            {
                Servo_SetAngle(SERVO_GRIPPER, kin_cal.gripper_open_angle);   /* 到位先张夹爪 */
                kin_flag.step = kin_flag.handshake_en ? KIN_STEP_WAIT_GRAB : KIN_STEP_WAIT_GRIP;  /* 等ok 或 直接闭 */
            }
            break;
        case KIN_STEP_PLACE_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_up_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_APPROACH_PLACE; kin_flag.need_send = 1; kin_motion.keepalive = 0; }   /* 电机5 升640 -> 云台+电机6 放置 */
            break;
        case KIN_STEP_PLACE_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_PLACE_RELEASE; }   /* m5 到600 -> 松夹爪 */
            break;
        case KIN_STEP_POSE_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_up_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_POSE_DUAL; kin_flag.need_send = 1; kin_motion.keepalive = 0; }   /* m5 升640 -> 云台+电机6 目标 */
            break;
        case KIN_STEP_POSE_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - s_pose_m5) <= kin_motion.pos_tol_mm)
            {
                /* 物料区一件「抓->放->回位」整条链走完, 塔吊已回位且空闲:
                   此时才上报 0x0F (抓取完成 + 本件颜色), 上位机据此把"期望颜色"推进到下一件。
                   (放完就发会提前触发下一个 0x08, 那一件会被 kin_flag.active 挡掉丢掉) */
                if (s_pick_pending_report)
                {
                    s_pick_pending_report = 0;
                    Mission_ReportPickDone(s_pick_color);
                }
                if (s_pose_after)   /* 姿态到位后还有下一段 (物料区抓取: 等授权 / cmd=11: 等底盘微调) */
                {
                    kin_flag.step = s_pose_after;
                    s_pose_after = 0;
                    kin_flag.need_send = 1;
                    kin_motion.keepalive = 0;
                }
                else { kin_flag.active = 0; kin_flag.done = 1; kin_flag.step = KIN_STEP_IDLE; }   /* m5 到位 -> 待命 */
            }
            break;
        default: break;
        case KIN_STEP_UNLOAD_GRAB_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.unload_grab_hoist_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_UNLOAD_GRAB_APPROACH; kin_flag.need_send = 1; kin_motion.keepalive = 0; }   /* m5 到600 -> 云台310+电机6 500 */
            break;
        case KIN_STEP_UNLOAD_RETURN_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_up_deg) <= kin_motion.pos_tol_mm)
            {
                /* 落点已知: 提前解好落点方向, 收回步云台直走落点(省掉先甩回正前135); 未知则回中立位135 */
                if (kin_flag.unload_have) { Kin_UnloadDropTarget(); s_retract_gimbal = kin_tgt.servo; }
                else                       s_retract_gimbal = kin_cal.home_gimbal_deg;
                kin_flag.step = KIN_STEP_UNLOAD_RETRACT; kin_flag.need_send = 1; kin_motion.keepalive = 0;
            }   /* m5 升640 -> 云台(落点/135)+电机6 0 */
            break;
        case KIN_STEP_UNLOAD_DROP_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.drop_hoist_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_UNLOAD_WAIT_RELEASE; }   /* m5 到落点高 -> 等放开 */
            break;
        case KIN_STEP_RLD_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_up_deg) <= kin_motion.pos_tol_mm)
            { Kin_ReloadDropTarget(); kin_flag.step = KIN_STEP_RLD_APPROACH_DROP; kin_flag.need_send = 1; kin_motion.keepalive = 0; }   /* m5 升640 -> 反序落点 */
            break;
        case KIN_STEP_RLD_DROP_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.reload_grab_hoist_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_RLD_GRIP; }   /* m5 到平地 -> 闭夹爪 */
            break;
        case KIN_STEP_RLD_RETURN_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.place_hoist_up_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_RLD_APPROACH_PLATFORM; kin_flag.need_send = 1; kin_motion.keepalive = 0; }   /* m5 升640 -> 平台 */
            break;
        case KIN_STEP_RLD_PLATFORM_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_cal.unload_grab_hoist_deg) <= kin_motion.pos_tol_mm)
            { kin_flag.step = KIN_STEP_RLD_RELEASE; }   /* m5 到平台高 -> 松夹爪 */
            break;
        }
    }
}

/* 读当前电机位置 + 云台角 -> 算当前夹爪 (x,y,z), 供上层"铆钉"/回传 (按需调用) */
void Kin_ReportPose(void)
{
    Tower_ReadPos5();
    Tower_ReadPos6();

    kin_tgt.pose_theta = Kin_ServoToTheta(Servo_GetAngle(SERVO_GIMBAL));
    kin_tgt.pose_r     = Kin_ExtendDegToMm(motor6_pos);
    kin_tgt.pose_z_mm  = Kin_HoistDegToMm(motor5_pos);
    Kin_FK(kin_tgt.pose_theta, kin_tgt.pose_r, kin_tgt.pose_z_mm,
           &kin_tgt.pose_x, &kin_tgt.pose_y, &kin_tgt.pose_z);
}

/* ================= 只读: 夹爪当前位置解算 (读一次) =================
 * Watch 写 kin_flag.read_trig=1 -> 读电机5/6实时位置 + 云台角, 正解出 (θ,r,z,x,y),
 * 结果在 kin_pose_*。纯读, 不发任何运动命令、不抢电机。
 * 注意: 读取会消费 CAN 总线上的位置帧; 若 Test_Task 同时开着连续读,
 * 建议先注释掉 MyTasks.c 里的 Test_Task() 再触发, 避免两处抢同一根总线。
 */
void Kin_ReadGripper(void)
{
    if (!kin_flag.read_trig) return;
    kin_flag.read_trig = 0;

    Tower_ReadPos5();
    Tower_ReadPos6();

    kin_tgt.pose_theta = Kin_ServoToTheta(Servo_GetAngle(SERVO_GIMBAL));
    kin_tgt.pose_r     = Kin_ExtendDegToMm(motor6_pos);
    kin_tgt.pose_z_mm  = Kin_HoistDegToMm(motor5_pos);
    Kin_FK(kin_tgt.pose_theta, kin_tgt.pose_r, kin_tgt.pose_z_mm,
           &kin_tgt.pose_x, &kin_tgt.pose_y, &kin_tgt.pose_z);
}

/* ================= 手动测试接口 (Keil Watch, 每10ms 由 Kin_TestRun 消费) =================
 * kin_test.mode: 0=关
 *   1=装货(回转平台460抓 -> 物料平台600放)
 *   2=卸货平地(物料平台600抓 -> 平地100放)
 *   3=回装(平地100抓 -> 物料平台600放)
 *   4=暂存(物料平台600抓 -> 暂存400放)
 * 目标二选一 (kin_test.coord_mode, 仅全自动 kin_test.handshake=0 时用):
 *   0=用 kin_tgt.test_x/y (mm, 塔吊系, 走逆解, 与视觉0x08/0x0C同)
 *   1=用 kin_tgt.test_gimbal_deg / kin_tgt.test_m6_deg (直接角度, 跳过逆解)
 * kin_test.handshake:
 *   0=全自动(默认, 现状): 不等视觉, 自动夹/松, 坐标用上面 coord_mode 二选一
 *   1=分步等视觉: 装货等 0x08 坐标+0x09 授权; 卸货等 0x0C 三个落点
 * 视觉卡住的解卡标志(写1, 消费后清0):
 *   等0x08 -> kin_test.grab_manual(用 kin_tgt.test_x/y 当目标)   等0x09 -> kin_flag.descend_cmd
 *   等0x0C -> kin_test.unload_manual(用 kin_tgt.test_x/y 当唯一落点) 闭夹爪->kin_flag.grip_cmd 松爪->kin_flag.release_cmd
 * 触发: 填好 mode + 坐标/角度后 kin_test.trig 写1。单次拿放, 完事回探查姿态135/600/0。
 * 忙时(kin_flag.active=1)写 trig 会顺延到本轮结束再触发。 */
/* (kin_test_* 标志已打包进 kin_test / kin_tgt, 见文件顶部) */

/* 装货视觉等待内部态 */
static uint8_t s_kin_test_wait_vision = 0;   /* 1=装货已触发, 停在等 0x08 坐标 */
static float   s_kin_test_grab_z        = 0.0f; /* 0x08 到后用的抓取高 */

void Kin_TestRun(void)
{
    /* 物料平台手动调档 (任何时候可用) */
    if (kin_test.platform_trig)
    {
        kin_test.platform_trig = 0;
        kin_flag.platform_sel = kin_test.platform_sel % 3;
        Servo_SetAngle(SERVO_PLATFORM,
                       kin_flag.platform_sel == 0 ? PLATFORM_ANGLE_A :
                       kin_flag.platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);
    }

    /* 卸货等 0x0C 落点卡住: 手动注入 kin_tgt.test_x/y 当唯一落点, 放 1 个就回收 */
    if (kin_test.unload_manual)
    {
        kin_test.unload_manual = 0;
        if (kin_flag.step == KIN_STEP_UNLOAD_WAIT_VISION && !kin_flag.unload_have)
        {
            kin_tgt.unload_x[0] = kin_tgt.test_x;
            kin_tgt.unload_y[0] = kin_tgt.test_y;
            kin_flag.unload_have = 1;
            s_kin_single_cycle = 1;   /* 只放这 1 个 */
        }
    }

    /* 装货视觉等待: 等 0x08 坐标(视觉) 或 kin_test.grab_manual(手动) 才动 */
    if (s_kin_test_wait_vision)
    {
        if (kin_test.trig)   /* 再写一次 trig = 取消这次等待, 回到待命 */
        {
            kin_test.trig = 0;
            s_kin_test_wait_vision = 0;
        }
        else if (kin_test.grab_manual)
        {
            kin_test.grab_manual = 0;
            s_kin_test_wait_vision = 0;
            kin_cal.grab_z_mm = s_kin_test_grab_z;
            Kin_GotoXY(kin_tgt.test_x, kin_tgt.test_y);
        }
        else if (Serial_ColorTargetReady)
        {
            Serial_ColorTargetReady = 0;
            s_kin_test_wait_vision = 0;
            kin_cal.grab_z_mm = s_kin_test_grab_z;
            Kin_GotoXY(kin_tgt.target_x, kin_tgt.target_y);   /* 0x08 已把坐标写进 kin_tgt.target_x/y */
        }
        return;
    }

    if (!kin_test.trig) return;
    if (kin_flag.active) return;   /* 忙: 保留 trig, 本轮走完再触发 */
    kin_test.trig = 0;

    if (kin_test.mode < 1 || kin_test.mode > 4) return;

    s_kin_test_direct = (kin_test.coord_mode != 0);
    s_kin_test_servo = kin_tgt.test_gimbal_deg;
    s_kin_test_m6    = kin_tgt.test_m6_deg;

    if (kin_test.handshake)
    {
        /* 分步等视觉: 打开握手, 自动夹/松; 装货等 0x08, 卸货等 0x0C */
        kin_flag.handshake_en    = 1;
        kin_flag.auto_grip    = 1;
        kin_flag.auto_release = 1;

        if (kin_test.mode == 1 || kin_test.mode == 3)
        {
            /* 装货: 先等 0x08 坐标再动 (0x09 授权在抓取位等) */
            s_kin_test_grab_z    = (kin_test.mode == 1) ? HOIST_HEIGHT_ROTATE : kin_cal.reload_grab_hoist_deg;
            s_kin_test_wait_vision = 1;
        }
        else   /* 2=卸货平地 4=暂存 */
        {
            kin_cal.drop_hoist_deg = (kin_test.mode == 2) ? HOIST_HEIGHT_DROP_GROUND : HOIST_HEIGHT_PALLET;
            s_kin_single_cycle = 0;   /* 允许 0x0C 的 3 个落点 */
            Kin_UnloadStart();        /* 平台抓 -> 等 0x0C -> 依次放 */
        }
        return;
    }

    /* 全自动(现状): 不等视觉 0x09/0x0C, 自动夹/松 */
    kin_flag.handshake_en = 0;
    kin_flag.auto_grip = 1;
    kin_flag.auto_release = 1;

    if (kin_test.mode == 1)              /* 装货: 回转平台460抓 -> 物料平台600放 */
    {
        kin_cal.grab_z_mm = HOIST_HEIGHT_ROTATE;
        Kin_GotoXY(kin_tgt.test_x, kin_tgt.test_y);
    }
    else if (kin_test.mode == 3)         /* 回装: 平地100抓 -> 物料平台600放 */
    {
        kin_cal.grab_z_mm = kin_cal.reload_grab_hoist_deg;
        Kin_GotoXY(kin_tgt.test_x, kin_tgt.test_y);
    }
    else if (kin_test.mode == 2)         /* 卸货平地: 物料平台600抓 -> 平地100放 */
    {
        kin_cal.drop_hoist_deg = HOIST_HEIGHT_DROP_GROUND;
        s_kin_single_cycle = 1;
        Kin_UnloadStart();
        kin_tgt.unload_x[0] = kin_tgt.test_x;
        kin_tgt.unload_y[0] = kin_tgt.test_y;
        kin_flag.unload_have = 1;             /* 手动给落点, 不等 0x0C */
    }
    else                                  /* 4=暂存: 物料平台600抓 -> 暂存400放 */
    {
        kin_cal.drop_hoist_deg = HOIST_HEIGHT_PALLET;
        s_kin_single_cycle = 1;
        Kin_UnloadStart();
        kin_tgt.unload_x[0] = kin_tgt.test_x;
        kin_tgt.unload_y[0] = kin_tgt.test_y;
        kin_flag.unload_have = 1;
    }
}
