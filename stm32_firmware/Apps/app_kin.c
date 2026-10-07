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

#define KIN_DEG2RAD (MATH_PI / 180.0f)
#define KIN_RAD2DEG (180.0f / MATH_PI)

/* 安全运动顺序 (状态机步骤) */
#define KIN_STEP_IDLE          0
#define KIN_STEP_SAFE_EXTEND   1   /* 电机6 -> 安全伸出 */
#define KIN_STEP_SAFE_HOIST    2   /* 电机5 -> 安全高度 */
#define KIN_STEP_GIMBAL        3   /* 云台   -> 目标角 */
#define KIN_STEP_REACH_EXTEND  4   /* 电机6 -> 目标 r */
#define KIN_STEP_REACH_HOIST   5   /* 电机5 -> 抓取 z */

/* ================= 标定参数 (占位, 现场标定) ================= */
float  kin_gimbal_theta_offset = 42.0f;      /* 现场标定: θ=0(+X) 时舵机值 = 42 */
int8_t kin_gimbal_dir          = 1;          /* +1: 舵机值增大 -> θ增大(逆时针) */
float  kin_gimbal_gain         = 1.033333f;  /* 斜率(舵机值/θ) = (135-42)/90 = 93/90 */

float  kin_extend_mm_per_deg   = 0.174f;    /* 实测: 伸出140.76mm / 808.8度 */
float  kin_extend_zero_deg     = 0.0f;      /* 电机6=0 时 r=104.48mm */
float  kin_extend_r_min_mm     = 104.48f;   /* 电机6=0(最短)时径向距离 = √(104.47²+1.36²) */
int8_t kin_extend_sign         = -1;        /* 伸出时 motor6_pos 读负值, 取 -1 才让 r 增大 */

float  kin_hoist_mm_per_deg    = 1.0f;
float  kin_hoist_zero_deg      = 0.0f;
float  kin_hoist_z_min_mm      = 0.0f;
int8_t kin_hoist_sign          = 1;

float  kin_extend_r_max_mm     = 1000.0f;
float  kin_hoist_z_max_mm      = 1000.0f;

/* 抓取高度(下位机定死) + 转云台前安全位姿 (现场标定) */
float  kin_grab_z_mm           = 0.0f;    /* TODO 现场量: 抓取时夹爪距地面高度 */
float  kin_safe_extend_mm      = 600.0f;  /* 转云台前: 电机6 安全位 */
float  kin_safe_hoist_mm       = 600.0f;  /* 转云台前: 电机5 安全位 */

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
float kin_target_theta = 0.0f, kin_target_r = 0.0f, kin_target_z_mm = 0.0f;

float kin_pose_x = 0.0f, kin_pose_y = 0.0f, kin_pose_z = 0.0f;
float kin_pose_theta = 0.0f, kin_pose_r = 0.0f, kin_pose_z_mm = 0.0f;

static uint8_t  kin_step = KIN_STEP_IDLE;   /* 当前步骤 */
static float    kin_tgt_servo = 0.0f;       /* 目标舵机角 */
static float    kin_tgt_m5_deg = 0.0f;      /* 目标电机5码值(度) */
static float    kin_tgt_m6_deg = 0.0f;      /* 目标电机6码值(度) */
static float    kin_safe_extend_deg = 0.0f; /* 安全位电机6码值 */
static float    kin_safe_hoist_deg  = 0.0f; /* 安全位电机5码值 */
static uint8_t  kin_phase = 0;              /* 0/1 交错: 发命令 / 读位置 (防 CAN 背靠背冲突) */
static uint16_t kin_keepalive = 0;          /* 兜底重发计数 */
static uint8_t  kin_need_send = 0;          /* 1=当前步需要发一次位置命令 */

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

/* 上层给 (x,y), z 用 kin_grab_z_mm; 启动安全运动顺序 */
void Kin_GotoXY(float x, float y)
{
    float r  = sqrtf(x * x + y * y);
    float th = atan2f(y, x) * KIN_RAD2DEG;

    kin_reachable = 1;
    if (r < kin_extend_r_min_mm) { r = kin_extend_r_min_mm; kin_reachable = 0; }
    if (r > kin_extend_r_max_mm) { r = kin_extend_r_max_mm; kin_reachable = 0; }

    kin_target_theta = th;
    kin_target_r     = r;
    kin_target_z_mm  = kin_grab_z_mm;

    kin_tgt_servo       = Kin_ThetaToServo(th);
    kin_tgt_m6_deg      = Kin_ExtendMmToDeg(r);
    kin_tgt_m5_deg      = Kin_HoistMmToDeg(kin_grab_z_mm);
    kin_safe_extend_deg = Kin_ExtendMmToDeg(kin_safe_extend_mm);
    kin_safe_hoist_deg  = Kin_HoistMmToDeg(kin_safe_hoist_mm);

    kin_step      = KIN_STEP_SAFE_EXTEND;   /* 先电机6 -> 安全位 */
    kin_phase     = 0;                      /* 下一拍先发命令 */
    kin_keepalive = 0;
    kin_need_send = 1;
    kin_done      = 0;
    kin_active    = 1;
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
  *         - kin_cmd: 1=GotoXY(用 kin_target_x/y), 2=停止 (处理后回 0)
  *         - 安全顺序: 电机6安全 -> 电机5安全 -> 云台 -> 电机6目标 -> 电机5抓取
  *         - 电机5/6 发命令与读位置交替, 防"位置命令+读位置"背靠背冲突
  *         - 走完 -> kin_active=0, kin_done=1
  */
void Kin_Update(void)
{
    if (kin_cmd == 1)      { kin_cmd = 0; Kin_GotoXY(kin_target_x, kin_target_y); }
    else if (kin_cmd == 2) { kin_cmd = 0; Kin_Stop(); }

    if (!kin_active) return;

    /* 云台步: 每拍斜坡到目标角 (PWM, 不占 CAN) */
    if (kin_step == KIN_STEP_GIMBAL)
    {
        float cur = Servo_GetAngle(SERVO_GIMBAL);
        float d = kin_tgt_servo - cur;
        if (d >  kin_gimbal_step_deg) d =  kin_gimbal_step_deg;
        if (d < -kin_gimbal_step_deg) d = -kin_gimbal_step_deg;
        Servo_SetAngle(SERVO_GIMBAL, cur + d);

        if (fabsf(Servo_GetAngle(SERVO_GIMBAL) - kin_tgt_servo) <= kin_gimbal_tol_deg)
        {
            kin_step      = KIN_STEP_REACH_EXTEND;   /* 云台到位 -> 先电机6 */
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
            case KIN_STEP_SAFE_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_safe_extend_deg, 1, 0);
                break;
            case KIN_STEP_SAFE_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_safe_hoist_deg, 1, 0);
                break;
            case KIN_STEP_REACH_EXTEND:
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, kin_extend_cmd_dir, kin_extend_acc, kin_extend_dec,
                                      kin_extend_vel, kin_tgt_m6_deg, 1, 0);
                break;
            case KIN_STEP_REACH_HOIST:
                X_V2_Traj_Pos_Control(MOTOR_HOIST, kin_hoist_cmd_dir, kin_hoist_acc, kin_hoist_dec,
                                      kin_hoist_vel, kin_tgt_m5_deg, 1, 0);
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
        case KIN_STEP_SAFE_EXTEND:
            Tower_ReadPos6();
            if (fabsf(Kin_ExtendDegToMm(motor6_pos) - kin_safe_extend_mm) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_SAFE_HOIST; kin_need_send = 1; kin_keepalive = 0; }
            break;
        case KIN_STEP_SAFE_HOIST:
            Tower_ReadPos5();
            if (fabsf(Kin_HoistDegToMm(motor5_pos) - kin_safe_hoist_mm) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_GIMBAL; }   /* 两轴都安全了, 才转云台 */
            break;
        case KIN_STEP_REACH_EXTEND:
            Tower_ReadPos6();
            if (fabsf(Kin_ExtendDegToMm(motor6_pos) - kin_target_r) <= kin_pos_tol_mm)
            { kin_step = KIN_STEP_REACH_HOIST; kin_need_send = 1; kin_keepalive = 0; }
            break;
        case KIN_STEP_REACH_HOIST:
            Tower_ReadPos5();
            if (fabsf(Kin_HoistDegToMm(motor5_pos) - kin_target_z_mm) <= kin_pos_tol_mm)
            { kin_active = 0; kin_done = 1; kin_step = KIN_STEP_IDLE; }
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
