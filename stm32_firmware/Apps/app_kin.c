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

/* 运动顺序 (状态机步骤; 旧 2 停靠步已废弃不用) */
#define KIN_STEP_IDLE          0
#define KIN_STEP_DESCEND       1   /* 电机5 -> 447 (收到ok后下探) */
#define KIN_STEP_SAFE_EXTEND   2   /* (废弃) */
#define KIN_STEP_GIMBAL        3   /* 云台   -> 目标角 */
#define KIN_STEP_REACH_EXTEND  4   /* 电机6 -> 目标 r */
#define KIN_STEP_REACH_HOIST   5   /* 电机5 -> 550 (预降高, 先) */
#define KIN_STEP_WAIT_GRAB      6   /* 等抓取授权(视觉0x09 或手动 kin_descend_cmd) -> 下探447 */
#define KIN_STEP_WAIT_GRIP      7   /* 等夹爪闭合(手动 kin_grip_cmd) -> 闭夹爪 -> 进入放置 */
#define KIN_STEP_PLACE_HOIST_UP 8   /* 电机5 -> 600 (先升) */
#define KIN_STEP_PLACE_EXTEND   9   /* 电机6 -> 600 (后) */
#define KIN_STEP_PLACE_GIMBAL   10  /* 云台   -> 220 (放置角) */
#define KIN_STEP_PLACE_EXTEND2  11  /* 电机6 -> 500 (先伸到放置位) */
#define KIN_STEP_PLACE_HOIST    12  /* 电机5 -> 480 (再降到放置高度) */
#define KIN_STEP_PLACE_RELEASE  13  /* 夹爪开 -> 旋转物料平台 */
#define KIN_STEP_RETURN_HOIST   14  /* 电机5 -> 600 (回默认: 先升, 清障) */
#define KIN_STEP_RETURN_EXTEND  15  /* 电机6 -> 600 (回默认: 后伸, 清障) */
#define KIN_STEP_RETURN_GIMBAL  16  /* 云台   -> 42 (回默认角) */
#define KIN_STEP_RETURN_EXTEND_RETRACT 17 /* 电机6 -> 0 (收回到默认, 电机5保持600) */
/* 卸货 (kin_cmd=3 触发): 平台抓取(220/447/500) -> 50默认(42/50/0) -> 等视觉 -> 到位 -> 等放开 */
#define KIN_STEP_UNLOAD_GRAB_HOIST_UP   18  /* 电机5 -> 600 (park 先升) */
#define KIN_STEP_UNLOAD_GRAB_EXTEND     19  /* 电机6 -> 600 (park 后) */
#define KIN_STEP_UNLOAD_GRAB_GIMBAL     20  /* 云台   -> 220 (平台) */
#define KIN_STEP_UNLOAD_GRAB_EXTEND2    21  /* 电机6 -> 500 (平台槽) */
#define KIN_STEP_UNLOAD_GRAB_HOIST      22  /* 电机5 -> 447 (抓取高) */
#define KIN_STEP_UNLOAD_GRIP            23  /* 夹爪自动闭合 + 延时 */
#define KIN_STEP_UNLOAD_RETURN_HOIST_UP 24  /* 电机5 -> 600 (lift 清障 先升) */
#define KIN_STEP_UNLOAD_RETURN_EXTEND   25  /* 电机6 -> 600 (后) */
#define KIN_STEP_UNLOAD_RETURN_GIMBAL   26  /* 云台   -> 42 (回前) */
#define KIN_STEP_UNLOAD_RETURN_HOIST    27  /* 电机5 -> 50 (降默认低) */
#define KIN_STEP_UNLOAD_RETURN_RETRACT  28  /* 电机6 -> 0 (收回默认) */
#define KIN_STEP_UNLOAD_WAIT_VISION     29  /* 等视觉坐标(kin_cmd=1) */
#define KIN_STEP_UNLOAD_DROP_GIMBAL     30  /* 云台   -> 视觉θ */
#define KIN_STEP_UNLOAD_DROP_EXTEND     31  /* 电机6 -> 视觉r */
#define KIN_STEP_UNLOAD_WAIT_RELEASE    32  /* 等放开(手动 kin_release_cmd) -> 松夹爪 */
#define KIN_STEP_UNLOAD_RELEASE         33  /* 松夹爪 + 延时 -> 转平台 -> 完成 */

/* ================= 标定参数 (占位, 现场标定) ================= */
float  kin_gimbal_theta_offset = 42.0f;      /* 现场标定: θ=0(+X) 时舵机值 = 42 */
int8_t kin_gimbal_dir          = 1;          /* +1: 舵机值增大 -> θ增大(逆时针) */
float  kin_gimbal_gain         = 1.033333f;  /* 斜率(舵机值/θ) = (135-42)/90 = 93/90 */

float  kin_extend_mm_per_deg   = 0.174f;    /* 实测: 伸出140.76mm / 808.8度 */
float  kin_extend_zero_deg     = 0.0f;      /* 电机6=0 时 r=104.48mm */
float  kin_extend_r_min_mm     = 104.48f;   /* 电机6=0(最短)时径向距离 = √(104.47²+1.36²) */
int8_t kin_extend_sign         = -1;        /* 伸出时 motor6_pos 读负值(0=最短→-808.8=最远), 取 -1 让 r 增大 */
float  kin_extend_target_offset_mm = 30.0f; /* 视觉给的点在伸出方向偏近20mm, 逆解命令补上(只改命令, 不改正解/FK) */

float  kin_hoist_mm_per_deg    = 1.0f;
float  kin_hoist_zero_deg      = 0.0f;
float  kin_hoist_z_min_mm      = 0.0f;
int8_t kin_hoist_sign          = 1;

float  kin_extend_r_max_mm     = 245.0f;   /* 物理最长 r ≈ 245mm (808.8度 × 0.174 + 104.48) */
float  kin_hoist_z_max_mm      = 1000.0f;

/* 抓取高度(下位机定死) + 转云台前停靠位 (抓取/放置共用, 单位电机度, 与 Tower 一致) */
float  kin_grab_z_mm           = 447.0f;    /* 电机5 收到ok后下探高度 447 */
float  kin_grab_approach_deg   = 550.0f;    /* 电机5 抓取前预降(视觉对准)高度 550 */
uint8_t kin_hoist_enable       = 1;       /* 1=走电机5升降轴; 0=跳过升降步 */
uint8_t kin_handshake_enable   = 1;       /* 1=分步模式: 到抓取位后等 ok(0x09) 再闭夹爪; 0=全自动直接闭 */

/* 装货放置 (抓取后自动执行): 单位与 Tower 一致 —— 电机5/6 用电机度(正数), 云台用舵机度 */
float kin_place_hoist_up_deg = 600.0f;   /* 电机5 转云台前停靠600 (抓取->放置 / 放置->回默认 共用) */
float kin_place_extend_deg   = 600.0f;   /* 电机6 转云台前停靠600 (同上) */
float kin_place_gimbal_deg   = 220.0f;   /* 云台放置角(舵机度) */
float kin_home_gimbal_deg    = 42.0f;    /* 云台回抓取位角(舵机度) */
float kin_home_extend_deg    = 0.0f;     /* 电机6 回抓取位(收回, 最短) */
float kin_home_hoist_deg     = 50.0f;    /* 电机5 默认位(低) */
float kin_place_hoist_deg    = 480.0f;   /* 电机5 放置高度 */
float kin_place_extend2_deg  = 500.0f;   /* 电机6 放置伸出 */
float kin_unload_grab_hoist_deg = 447.0f; /* 卸货: 平台抓取时电机5高度 (装货放置480 改成447) */

/* 卸货: 0x0C 帧一次给 3 个落点(3 组 xy), 状态机依次放 */
float   kin_unload_x[3] = {0.0f, 0.0f, 0.0f};
float   kin_unload_y[3] = {0.0f, 0.0f, 0.0f};
uint8_t kin_unload_idx  = 0;   /* 当前放第几个 (0/1/2) */
uint8_t kin_unload_have = 0;   /* 1=已拿到 3 个落点, 后续轮不再等新帧 */

/* 夹爪角度 (Watch 可调) */
float kin_gripper_close_angle = GRIPPER_CLOSE_ANGLE;  /* 抓取闭合 */
float kin_gripper_open_angle  = GRIPPER_OPEN_ANGLE;   /* 放置张开 */
uint16_t kin_gripper_delay_10ms = 50;   /* 夹爪动作后等待(10ms 单位) */

/* 手动触发命令 (锁存: Watch 写1, 状态机消费后清0) */
uint8_t kin_descend_cmd = 0;   /* 手动下降抓取授权, 与视觉 grab_cmd(0x09) 等价 */
uint8_t kin_grip_cmd    = 0;   /* 手动夹爪闭合抓取(装货用) */
uint8_t kin_release_cmd = 0;   /* 手动松开夹爪(卸货用) */
uint8_t kin_platform_sel = 0;  /* 物料平台槽位 0/1/2 -> A/B/C, 每次放置完成后旋转一次 */

float    kin_gimbal_step_deg   = 1.0f;
float    kin_gimbal_tol_deg    = 0.5f;
float    kin_pos_tol_mm        = 2.0f;
uint8_t  kin_hoist_cmd_dir     = 0;   /* 沿用实测: 电机5 dir=0 */
uint8_t  kin_extend_cmd_dir    = 1;   /* 沿用实测: 电机6 dir=1 */
uint16_t kin_hoist_acc         = 100;
uint16_t kin_hoist_dec         = 100;
float    kin_hoist_vel         = 100.0f;
uint16_t kin_extend_acc        = 100;
uint16_t kin_extend_dec        = 100;
float    kin_extend_vel        = 100.0f;

/* ================= 状态 ================= */
uint8_t kin_cmd       = 0;
uint8_t kin_active    = 0;
uint8_t kin_done      = 0;
uint8_t kin_reachable = 1;

float kin_target_x = 0.0f, kin_target_y = 0.0f;
float kin_last_tx  = 0.0f, kin_last_ty  = 0.0f;  /* 上次已执行的目标, 用于自动触发 */
float kin_target_theta = 0.0f, kin_target_r = 0.0f, kin_target_z_mm = 0.0f;

float kin_pose_x = 0.0f, kin_pose_y = 0.0f, kin_pose_z = 0.0f;
float kin_pose_theta = 0.0f, kin_pose_r = 0.0f, kin_pose_z_mm = 0.0f;

uint8_t  kin_step = KIN_STEP_IDLE;   /* 当前步骤 (Watch 可查, 看卡在哪一步) */
float    kin_tgt_servo = 0.0f;       /* 目标舵机角 */
float    kin_tgt_m5_deg = 0.0f;      /* 目标电机5码值(度) */
float    kin_tgt_m6_deg = 0.0f;      /* 目标电机6码值(度) */
uint8_t  kin_phase = 0;              /* 0/1 交错: 发命令 / 读位置 (防 CAN 背靠背冲突) */
uint16_t kin_keepalive = 0;          /* 兜底重发计数 */
uint8_t  kin_need_send = 0;          /* 1=当前步需要发一次位置命令 */
uint32_t kin_motion_count = 0;       /* 每次 Kin_GotoXY +1: 确认重触发有没有进来 */

/* ================= 单位换算 ================= */
float Kin_ThetaToServo(float theta_deg)
{
    return kin_gimbal_theta_offset + kin_gimbal_dir * kin_gimbal_gain * theta_deg;
}
float Kin_ServoToTheta(float servo_deg)
{
    return kin_gimbal_dir * (servo_deg - kin_gimbal_theta_offset) / kin_gimbal_gain;
}

float Kin_ExtendMmToDeg(float r_mm)
{
    return kin_extend_zero_deg + kin_extend_sign * (r_mm - kin_extend_r_min_mm) / kin_extend_mm_per_deg;
}
float Kin_ExtendDegToMm(float deg)
{
    return kin_extend_r_min_mm + kin_extend_sign * kin_extend_mm_per_deg * (deg - kin_extend_zero_deg);
}

float Kin_HoistMmToDeg(float z_mm)
{
    return kin_hoist_zero_deg + kin_hoist_sign * (z_mm - kin_hoist_z_min_mm) / kin_hoist_mm_per_deg;
}
float Kin_HoistDegToMm(float deg)
{
    return kin_hoist_z_min_mm + kin_hoist_sign * kin_hoist_mm_per_deg * (deg - kin_hoist_zero_deg);
}

/* ================= 正解 (θ, r, z) -> (x, y, z) ================= */
void Kin_FK(float theta_deg, float r_mm, float z_mm, float *x, float *y, float *z)
{
    float t = theta_deg * KIN_DEG2RAD;
    *x = r_mm * cosf(t);
    *y = r_mm * sinf(t);
    *z = z_mm;
}

/* ================= 逆解 (x, y, z) -> (θ, r, z) =================
 * 返回 1=可达(未截断), 0=超出工作空间(已截断到边界, 上层据此判断目标不可达) */
uint8_t Kin_IK(float x, float y, float z, float *theta_deg, float *r_mm, float *z_mm)
{
    float r  = sqrtf(x * x + y * y);
    float th = atan2f(y, x) * KIN_RAD2DEG;
    float zz = z;
    uint8_t ok = 1;

    if (r  < kin_extend_r_min_mm) { r  = kin_extend_r_min_mm; ok = 0; }
    if (r  > kin_extend_r_max_mm) { r  = kin_extend_r_max_mm; ok = 0; }
    if (zz < kin_hoist_z_min_mm)  { zz = kin_hoist_z_min_mm;  ok = 0; }
    if (zz > kin_hoist_z_max_mm)  { zz = kin_hoist_z_max_mm;  ok = 0; }

    *theta_deg = th;
    *r_mm      = r;
    *z_mm      = zz;
    return ok;
}

/* ================= 点到点运动 (安全顺序) ================= */

/* 上层给 (x,y), z 用 kin_grab_z_mm; 启动运动顺序 */
void Kin_GotoXY(float x, float y)
{
    float r  = sqrtf(x * x + y * y) + kin_extend_target_offset_mm;  /* 视觉点在伸出方向少20mm, 补上再算 */
    float th = atan2f(y, x) * KIN_RAD2DEG;

    kin_reachable = 1;
    kin_motion_count++;   /* 触发计数+1: Watch 看它确认重触发有没有进来 */
    kin_last_tx = x;      /* 记录本次目标, 供自动触发比较 */
    kin_last_ty = y;
    if (r < kin_extend_r_min_mm) { r = kin_extend_r_min_mm; kin_reachable = 0; }
    if (r > kin_extend_r_max_mm) { r = kin_extend_r_max_mm; kin_reachable = 0; }

    kin_target_theta = th;
    kin_target_r     = r;
    kin_target_z_mm  = kin_grab_z_mm;

    kin_tgt_servo       = Kin_ThetaToServo(th);
    kin_tgt_m6_deg      = Kin_ExtendMmToDeg(r);
    kin_tgt_m5_deg      = kin_grab_approach_deg;

    kin_step      = kin_hoist_enable ? KIN_STEP_REACH_HOIST : KIN_STEP_GIMBAL;   /* 先电机5 -> 550预降, 再舵机, 再电机6 */
    kin_phase     = 0;                      /* 下一拍先发命令 */
    kin_keepalive = 0;
    kin_need_send = 1;
    kin_done      = 0;
    kin_active    = 1;
}

/* 卸货启动 (kin_cmd=3): 平台抓取(220/447/500) -> 50默认(42/50/0) -> 等 0x0C 落点 -> 依次放 3 个 -> 等放开 */
void Kin_UnloadStart(void)
{
    kin_reachable  = 1;
    kin_motion_count++;   /* 触发计数: 确认重触发有没有进来 */
    kin_unload_idx  = 0;
    kin_unload_have = 0;
    kin_step       = KIN_STEP_UNLOAD_GRAB_HOIST_UP;   /* 先600清障 -> 云台220 -> m6 500 -> m5 447 -> 自动夹取 */
    kin_phase      = 0;
    kin_keepalive  = 0;
    kin_need_send  = 1;
    kin_done       = 0;
    kin_active     = 1;
}

/* 卸货放置: 用 0x0C 第 kin_unload_idx 个落点(塔吊系绝对 x,y) -> θ/r, m5 保持默认低50, 到位等放开 */
void Kin_UnloadDrop(void)
{
    float x  = kin_unload_x[kin_unload_idx];
    float y  = kin_unload_y[kin_unload_idx];
    float r  = sqrtf(x * x + y * y) + kin_extend_target_offset_mm;  /* 视觉点在伸出方向偏近, 补上再算 */
    float th = atan2f(y, x) * KIN_RAD2DEG;

    kin_reachable = 1;
    kin_last_tx = x; kin_last_ty = y;
    if (r < kin_extend_r_min_mm) { r = kin_extend_r_min_mm; kin_reachable = 0; }
    if (r > kin_extend_r_max_mm) { r = kin_extend_r_max_mm; kin_reachable = 0; }

    kin_target_theta = th;
    kin_target_r     = r;
    kin_tgt_servo    = Kin_ThetaToServo(th);
    kin_tgt_m6_deg   = Kin_ExtendMmToDeg(r);

    kin_step      = KIN_STEP_UNLOAD_DROP_GIMBAL;   /* 云台 -> 落点θ, 再电机6 -> 落点r */
    kin_phase     = 0;
    kin_keepalive = 0;
    kin_need_send = 1;
}

void Kin_Stop(void)
{
    kin_active = 0;
    kin_done   = 0;
    kin_step   = KIN_STEP_IDLE;
    X_V2_Stop_Now(MOTOR_HOIST,  false);
    X_V2_Stop_Now(MOTOR_EXTEND, false);
}

/**
  * @brief  运动状态机 (每 10ms 调用)
  *         - kin_cmd: 1=装货抓取 GotoXY(用 kin_target_x/y); 卸货等视觉时 1=到位等放开;
  *                    2=停止; 3=卸货(平台抓取->50默认->等视觉)。处理后回 0。
  *         - 抓取顺序: 电机5预降550 -> 云台 -> 电机6目标 -> 等授权 -> 下探447 -> 等夹爪
  *         - 电机5/6 发命令与读位置交替, 防"位置命令+读位置"背靠背冲突
  *         - 走完 -> kin_active=0, kin_done=1
  */
void Kin_Update(void)
{
    if (kin_cmd == 1)
    {
        kin_cmd = 0;
        if (kin_step == KIN_STEP_UNLOAD_WAIT_VISION) { if (kin_unload_have) Kin_UnloadDrop(); }  /* 卸货: 兜底手动触发(须已有0x0C落点) */
        else                                         Kin_GotoXY(kin_target_x, kin_target_y);  /* 装货: 抓取 */
    }
    else if (kin_cmd == 2) { kin_cmd = 0; Kin_Stop(); }
    else if (kin_cmd == 3) { kin_cmd = 0; Kin_UnloadStart(); }   /* 卸货: 平台抓取 -> 50默认 */

    /* 只由 kin_cmd=1 触发运动: 坐标(kin_target_x/y)变了不会自己动, 必须手动给 cmd */
    if (!kin_active) return;

    /* 云台步: 每拍斜坡到目标角 (PWM, 不占 CAN); 抓取角 kin_tgt_servo / 放置角 kin_place_gimbal_deg / 默认角 kin_home_gimbal_deg */
    if (kin_step == KIN_STEP_GIMBAL || kin_step == KIN_STEP_PLACE_GIMBAL || kin_step == KIN_STEP_RETURN_GIMBAL ||
        kin_step == KIN_STEP_UNLOAD_GRAB_GIMBAL || kin_step == KIN_STEP_UNLOAD_RETURN_GIMBAL || kin_step == KIN_STEP_UNLOAD_DROP_GIMBAL)
    {
        uint8_t next;
        float   tgt;
        switch (kin_step)
        {
        case KIN_STEP_GIMBAL:              tgt = kin_tgt_servo;       next = KIN_STEP_REACH_EXTEND;         break;
        case KIN_STEP_PLACE_GIMBAL:        tgt = kin_place_gimbal_deg;next = KIN_STEP_PLACE_EXTEND2;        break;
        case KIN_STEP_RETURN_GIMBAL:       tgt = kin_home_gimbal_deg; next = KIN_STEP_RETURN_EXTEND_RETRACT; break;
        case KIN_STEP_UNLOAD_GRAB_GIMBAL:  tgt = kin_place_gimbal_deg;next = KIN_STEP_UNLOAD_GRAB_EXTEND2;  break;
        case KIN_STEP_UNLOAD_RETURN_GIMBAL:tgt = kin_home_gimbal_deg; next = KIN_STEP_UNLOAD_RETURN_HOIST; break;
        default /* UNLOAD_DROP_GIMBAL */: tgt = kin_tgt_servo;       next = KIN_STEP_UNLOAD_DROP_EXTEND;   break;
        }
        {
            float cur = Servo_GetAngle(SERVO_GIMBAL);
            float d = tgt - cur;
            if (d >  kin_gimbal_step_deg) d =  kin_gimbal_step_deg;
            if (d < -kin_gimbal_step_deg) d = -kin_gimbal_step_deg;
            Servo_SetAngle(SERVO_GIMBAL, cur + d);

            if (fabsf(Servo_GetAngle(SERVO_GIMBAL) - tgt) <= kin_gimbal_tol_deg)
            {
                kin_step = next;
                kin_need_send = 1;
                kin_keepalive = 0;
            }
        }
        return;
    }

    /* 等抓取授权: 视觉 GrabCmd(0x09) 或手动 kin_descend_cmd, 二选一 -> 下探到抓取高度 */
    if (kin_step == KIN_STEP_WAIT_GRAB)
    {
        if (grab_cmd || kin_descend_cmd)
        {
            grab_cmd = 0;
            kin_descend_cmd = 0;
            kin_step = KIN_STEP_DESCEND;
            kin_need_send = 1;
            kin_keepalive = 0;
        }
        return;
    }

    /* 等夹爪闭合: 手动 kin_grip_cmd -> 闭夹爪 -> 进入放置 */
    if (kin_step == KIN_STEP_WAIT_GRIP)
    {
        if (kin_grip_cmd)
        {
            kin_grip_cmd = 0;
            Servo_SetAngle(SERVO_GRIPPER, kin_gripper_close_angle);   /* 闭夹爪抓取 */
            kin_step = KIN_STEP_PLACE_HOIST_UP; kin_need_send = 1; kin_keepalive = 0;  /* 先升电机5 到600 */
        }
        return;
    }

    /* 卸货夹取: 自动闭夹爪, 稍等让舵机闭合, 再升600清障 */
    if (kin_step == KIN_STEP_UNLOAD_GRIP)
    {
        static uint16_t ugrip_tick = 0;
        Servo_SetAngle(SERVO_GRIPPER, kin_gripper_close_angle);   /* 自动闭夹爪夹取 */
        if (++ugrip_tick >= kin_gripper_delay_10ms)
        {
            ugrip_tick = 0;
            kin_step = KIN_STEP_UNLOAD_RETURN_HOIST_UP; kin_need_send = 1; kin_keepalive = 0;  /* 先升电机5 到600 */
        }
        return;
    }

    /* 卸货: 停在50默认等 0x0C 卸货坐标 (首轮等新帧并快照 3 落点; 后续轮坐标已存直接放) */
    if (kin_step == KIN_STEP_UNLOAD_WAIT_VISION)
    {
        if (Serial_UnloadCoordReady)
        {
            uint8_t i, n;
            Serial_UnloadCoordReady = 0;
            n = Serial_RxData.unload_count;
            if (n > 3) n = 3;
            for (i = 0; i < n; i++)
            {
                kin_unload_x[i] = Serial_RxData.unload_x[i];
                kin_unload_y[i] = Serial_RxData.unload_y[i];
            }
            kin_unload_have = 1;
        }
        if (kin_unload_have)
            Kin_UnloadDrop();
        return;
    }

    /* 卸货放开: 手动 kin_release_cmd -> 松夹爪 */
    if (kin_step == KIN_STEP_UNLOAD_WAIT_RELEASE)
    {
        if (kin_release_cmd)
        {
            kin_release_cmd = 0;
            Servo_SetAngle(SERVO_GRIPPER, kin_gripper_open_angle);   /* 松夹爪 */
            kin_step = KIN_STEP_UNLOAD_RELEASE;
        }
        return;
    }

    /* 卸货完成: 稍等让舵机动作, 平台转下一档(C->B->A); 还有落点则回平台抓下一个环, 放满 3 个回待命 */
    if (kin_step == KIN_STEP_UNLOAD_RELEASE)
    {
        static uint16_t urel_tick = 0;
        if (++urel_tick >= kin_gripper_delay_10ms)
        {
            urel_tick = 0;
            kin_platform_sel = (kin_platform_sel + 2) % 3;   /* 卸货 C->B->A 清空 */
            Servo_SetAngle(SERVO_PLATFORM,
                           kin_platform_sel == 0 ? PLATFORM_ANGLE_A :
                           kin_platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);

            kin_unload_idx++;
            if (kin_unload_idx < 3)
            {
                /* 还有下一个落点: 回平台抓下一个环 (600清障 -> 云台220 -> ...) */
                kin_step = KIN_STEP_UNLOAD_GRAB_HOIST_UP;
                kin_need_send = 1;
                kin_keepalive = 0;
            }
            else
            {
                kin_active = 0; kin_done = 1; kin_step = KIN_STEP_IDLE;
            }
        }
        return;
    }

    /* 放置完成: 夹爪张开, 稍等让舵机动作, 然后回待命 */
    if (kin_step == KIN_STEP_PLACE_RELEASE)
    {
        static uint16_t rel_tick = 0;
        Servo_SetAngle(SERVO_GRIPPER, kin_gripper_open_angle);   /* 松夹爪 */
        if (++rel_tick >= kin_gripper_delay_10ms)
        {
            rel_tick = 0;
            /* 放完一次: 夹爪保持默认张开, 旋转一次物料平台(换下一槽位), 回待命等下次视觉坐标 */
            kin_platform_sel = (kin_platform_sel + 1) % 3;
            Servo_SetAngle(SERVO_PLATFORM,
                           kin_platform_sel == 0 ? PLATFORM_ANGLE_A :
                           kin_platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);
            kin_step = KIN_STEP_RETURN_HOIST;   /* 放完 -> 回默认(先600清障) */
            kin_need_send = 1;
            kin_keepalive = 0;
        }
        return;
    }

    /* 电机步: 发命令 / 读位置 交替 */
    kin_phase ^= 1;
    if (kin_phase)
    {
        /* 发命令拍: 目标变化发一次 + 约1s兜底重发 */
        if (kin_need_send || ++kin_keepalive >= 100)
        {
            kin_need_send = 0;
            kin_keepalive = 0;
            switch (kin_step)
            {
            case KIN_STEP_REACH_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_tgt_m6_deg, 1, 0);
                break;
            case KIN_STEP_REACH_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_tgt_m5_deg, 1, 0);
                break;
            case KIN_STEP_DESCEND:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_grab_z_mm, 1, 0);   /* 447 下探 */
                break;
            case KIN_STEP_PLACE_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_place_hoist_up_deg, 1, 0);
                break;
            case KIN_STEP_PLACE_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend_deg, 1, 0);
                break;
            case KIN_STEP_PLACE_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_place_hoist_deg, 1, 0);
                break;
            case KIN_STEP_PLACE_EXTEND2:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend2_deg, 1, 0);
                break;
            case KIN_STEP_RETURN_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_place_hoist_up_deg, 1, 0);   /* 600 清障 */
                break;
            case KIN_STEP_RETURN_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend_deg, 1, 0);   /* 600 清障 */
                break;
            case KIN_STEP_RETURN_EXTEND_RETRACT:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_home_extend_deg, 1, 0);   /* 0 默认 */
                break;
            case KIN_STEP_UNLOAD_GRAB_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_place_hoist_up_deg, 1, 0);   /* 600 park */
                break;
            case KIN_STEP_UNLOAD_GRAB_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend_deg, 1, 0);   /* 600 park */
                break;
            case KIN_STEP_UNLOAD_GRAB_EXTEND2:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend2_deg, 1, 0);   /* 500 平台槽 */
                break;
            case KIN_STEP_UNLOAD_GRAB_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_unload_grab_hoist_deg, 1, 0);   /* 447 平台抓取高 */
                break;
            case KIN_STEP_UNLOAD_RETURN_HOIST_UP:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_place_hoist_up_deg, 1, 0);   /* 600 清障 */
                break;
            case KIN_STEP_UNLOAD_RETURN_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_place_extend_deg, 1, 0);   /* 600 清障 */
                break;
            case KIN_STEP_UNLOAD_RETURN_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_home_hoist_deg, 1, 0);   /* 50 默认低 */
                break;
            case KIN_STEP_UNLOAD_RETURN_RETRACT:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_home_extend_deg, 1, 0);   /* 0 收回 */
                break;
            case KIN_STEP_UNLOAD_DROP_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_tgt_m6_deg, 1, 0);   /* 视觉 r */
                break;
            default: break;
            }
        }
    }
    else
    {
        /* 读位置拍: 判当前步到位, 推进下一步 */
        switch (kin_step)
        {
        case KIN_STEP_REACH_EXTEND:
            Tower_ReadPos6();
            if (fabsf(Kin_ExtendDegToMm(motor6_pos) - kin_target_r) <= kin_pos_tol_mm)
            {
                /* 电机6 到位(抓取点上方, 电机5 在550预降高): 分步模式等 ok 下探447, 否则直接下探 */
                if (kin_hoist_enable)
                {
                    kin_step = kin_handshake_enable ? KIN_STEP_WAIT_GRAB : KIN_STEP_DESCEND;
                    kin_need_send = 1; kin_keepalive = 0;
                }
                else { kin_active = 0; kin_done = 1; kin_step = KIN_STEP_IDLE; }
            }
            break;
        case KIN_STEP_REACH_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_grab_approach_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_GIMBAL; }   /* 电机5 先到550预降高 -> 转云台 */
            break;
        case KIN_STEP_DESCEND:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_grab_z_mm) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_WAIT_GRIP; }   /* 下探到447 -> 等夹爪闭合 */
            break;
        case KIN_STEP_PLACE_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_place_hoist_up_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_PLACE_EXTEND; kin_need_send = 1; kin_keepalive = 0; }   /* 电机5 先升 -> 电机6 到600 */
            break;
        case KIN_STEP_PLACE_EXTEND:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_PLACE_GIMBAL; }   /* 两轴都停靠好, 转云台 */
            break;
        case KIN_STEP_PLACE_EXTEND2:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend2_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_PLACE_HOIST; kin_need_send = 1; kin_keepalive = 0; }  /* m6 先到位 -> 电机5 再降 */
            break;
        case KIN_STEP_PLACE_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_place_hoist_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_PLACE_RELEASE; }   /* m5 再到位 -> 松夹爪 */
            break;
        case KIN_STEP_RETURN_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_place_hoist_up_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_RETURN_EXTEND; kin_need_send = 1; kin_keepalive = 0; }   /* m5 先升600 -> m6 伸600 */
            break;
        case KIN_STEP_RETURN_EXTEND:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_RETURN_GIMBAL; }   /* m6 伸600 -> 回云台42 */
            break;
        case KIN_STEP_RETURN_EXTEND_RETRACT:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_home_extend_deg) <= kin_pos_tol_mm)
            { kin_active = 0; kin_done = 1; kin_step = KIN_STEP_IDLE; }   /* m6收0, 电机5保持600 -> 待命 */
            break;
        case KIN_STEP_UNLOAD_GRAB_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_place_hoist_up_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_GRAB_EXTEND; kin_need_send = 1; kin_keepalive = 0; }   /* m5 先升600 -> m6 到600 */
            break;
        case KIN_STEP_UNLOAD_GRAB_EXTEND:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_GRAB_GIMBAL; }   /* m6 到600 -> 云台220 */
            break;
        case KIN_STEP_UNLOAD_GRAB_EXTEND2:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend2_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_GRAB_HOIST; kin_need_send = 1; kin_keepalive = 0; }   /* m6 先500 -> m5 到447 */
            break;
        case KIN_STEP_UNLOAD_GRAB_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_unload_grab_hoist_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_GRIP; }   /* m5 到447 -> 自动夹取 */
            break;
        case KIN_STEP_UNLOAD_RETURN_HOIST_UP:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_place_hoist_up_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_RETURN_EXTEND; kin_need_send = 1; kin_keepalive = 0; }   /* m5 先升600 -> m6 到600 */
            break;
        case KIN_STEP_UNLOAD_RETURN_EXTEND:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_place_extend_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_RETURN_GIMBAL; }   /* m6 到600 -> 云台42 */
            break;
        case KIN_STEP_UNLOAD_RETURN_HOIST:
            Tower_ReadPos5();
            if (fabsf(fabsf(motor5_pos) - kin_home_hoist_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_RETURN_RETRACT; kin_need_send = 1; kin_keepalive = 0; }   /* m5 到50 -> m6 收0 */
            break;
        case KIN_STEP_UNLOAD_RETURN_RETRACT:
            Tower_ReadPos6();
            if (fabsf(fabsf(motor6_pos) - kin_home_extend_deg) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_WAIT_VISION; Serial_UnloadCoordReady = 0; }   /* m6 收0 -> 等 0x0C 卸货坐标(清旧帧, 首轮等新帧) */
            break;
        case KIN_STEP_UNLOAD_DROP_EXTEND:
            Tower_ReadPos6();
            if (fabsf(Kin_ExtendDegToMm(motor6_pos) - kin_target_r) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_UNLOAD_WAIT_RELEASE; }   /* m6 到视觉r -> 等放开 */
            break;
        default: break;
        }
    }
}

/* 读当前电机位置 + 云台角 -> 算当前夹爪 (x,y,z), 供上层"铆钉"/回传 (按需调用) */
void Kin_ReportPose(void)
{
    Tower_ReadPos5();
    Tower_ReadPos6();

    kin_pose_theta = Kin_ServoToTheta(Servo_GetAngle(SERVO_GIMBAL));
    kin_pose_r     = Kin_ExtendDegToMm(motor6_pos);
    kin_pose_z_mm  = Kin_HoistDegToMm(motor5_pos);
    Kin_FK(kin_pose_theta, kin_pose_r, kin_pose_z_mm,
           &kin_pose_x, &kin_pose_y, &kin_pose_z);
}

/* ================= 只读: 夹爪当前位置解算 (读一次) =================
 * Watch 写 kin_read_trig=1 -> 读电机5/6实时位置 + 云台角, 正解出 (θ,r,z,x,y),
 * 结果在 kin_pose_*。纯读, 不发任何运动命令、不抢电机。
 * 注意: 读取会消费 CAN 总线上的位置帧; 若 Motor56_Test_Task 同时开着连续读,
 * 建议先 motor56_test_enable=0 再触发, 避免两处抢同一根总线。
 */
uint8_t kin_read_trig = 0;   /* Watch 写 1 触发读一次, 读完自动清 0 */

void Kin_ReadGripper(void)
{
    if (!kin_read_trig) return;
    kin_read_trig = 0;

    Tower_ReadPos5();
    Tower_ReadPos6();

    kin_pose_theta = Kin_ServoToTheta(Servo_GetAngle(SERVO_GIMBAL));
    kin_pose_r     = Kin_ExtendDegToMm(motor6_pos);
    kin_pose_z_mm  = Kin_HoistDegToMm(motor5_pos);
    Kin_FK(kin_pose_theta, kin_pose_r, kin_pose_z_mm,
           &kin_pose_x, &kin_pose_y, &kin_pose_z);
}
