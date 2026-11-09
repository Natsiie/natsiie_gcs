#include "sys.h"

/**
  * @brief  麦轮底盘控制 (4 × EMM固件闭环步进, 速度模式)
  *         - 遥控直控: sw[0] 锁定, Rx左右 Ry前后, Lx旋转(速率直控+松杆锁航向)
  *         - 点对点定位: btn0 回原点(0,0) / btn1 沿车头往前, 里程计闭环 P 控 + 陀螺仪锁车头
  *         - 麦轮里程计: 读4轮位置 + 陀螺仪 yaw 航迹推算
  *         电机地址: 1=BL(后左) 2=BR(后右) 3=FL(前左) 4=FR(前右); 四轮 EMM 固件, 上电自动使能
  */

/* ================= 底盘打包变量 (Keil Watch 只看 chassis_flag/chassis_cfg/chassis_val) ================= */
ChassisFlag_t chassis_flag = {
    .test_mode        = 0,
    .yaw_enable       = 1,
    .min_omega_en     = CHASSIS_MIN_OMEGA_EN,
    .need_sw_unlock   = 1,
    .rot_prio_en      = CHASSIS_ROT_PRIO_EN,
    .odom_use_autorep = CHASSIS_AUTOREP_EN,
    .trim_enable      = 1,
    .autorep_rearm_en = 0,   /* 默认关: 每秒给四个电机全量重发会把它们的定时器对齐 -> 每 20ms 一起抢仲裁,
                                低地址永远赢, 3/4 号饿死停报 (实测 stale 13% -> 0.2%)。见 docs/decisions/010 */
};
ChassisCfg_t chassis_cfg = {
    .test_speed          = CHASSIS_SPEED_TEST,   /* 速度全部在 car_config.h 的"底盘速度总表" */
    .yaw_kp              = CHASSIS_YAW_KP,   /* P (参考 yyb Kp=3.3: 0.6 太弱压不住摩擦卡死) */
    .yaw_ki              = CHASSIS_YAW_KI,   /* I (参考为0) */
    .yaw_kd              = CHASSIS_YAW_KD,   /* D: 锁航向不需要 */
    .yaw_kd_rate         = CHASSIS_YAW_KD_RATE, /* 角速度阻尼 (RPM/(°/s)), 反了取负 */
    .yaw_damp_max        = CHASSIS_YAW_DAMP_MAX,
    .yaw_rate_dz         = CHASSIS_YAW_RATE_DZ,
    .min_omega           = CHASSIS_MIN_OMEGA,
    .yaw_max_out         = CHASSIS_YAW_MAX_OUT,
    .yaw_max_i           = CHASSIS_YAW_MAX_I,
    .yaw_deadzone        = CHASSIS_YAW_DEADZONE,
    .yaw_iband           = CHASSIS_YAW_IBAND,
    .yaw_invert          = CHASSIS_YAW_INVERT,
    .yaw_rate_lpf_alpha  = 0.3f,     /* Gz 低通滤波系数, 越小越平滑但滞后越大; 1=不过滤 */
    .rot_settle_ticks    = CHASSIS_ROT_SETTLE_TICKS,   /* 原地旋转到位稳定拍数 (10ms/拍) */
    .rc_speed            = CHASSIS_SPEED_RC,     /* 遥控平移满杆 (car_config.h) */
    .rc_rotate_rate      = CHASSIS_SPEED_RC_ROT, /* 遥控旋转满杆 (car_config.h) */
    .rc_settle_ms        = 300.0f,   /* 松杆后滑行稳定时间(ms) */
    .rc_rx_invert        = -1,
    .rc_ry_invert        = 1,
    .rc_lx_invert        = 1,
    .wheel_max           = CHASSIS_WHEEL_MAX,
    .odom_wheel_radius_mm = 48.5f,   /* 麦轮半径 (mm): 外径97/2 */
    .odom_gear_ratio     = 1.0f,     /* 减速比, 直驱=1 */
    .odom_yaw_invert     = 1,
    .odom_update_cycles  = 5,        /* 每 N 个 10ms 周期更新一次 */
    .pos_kp              = 0.2f,     /* 位置 P (RPM/mm): 只在 pos_decel=0 (老模式兜底) 时用 */
    .pos_decel           = CHASSIS_POS_DECEL_MMPS2,  /* 定位刹车减速度 (mm/s²; 0=退回老 P 控) */
    .pos_taper_mm        = CHASSIS_POS_TAPER_MM,     /* 末端线性收口半径 (mm) */
    .pos_max_speed       = CHASSIS_SPEED_POS,    /* ★主速度旋钮: 定位/导航 (car_config.h) */
    .pos_deadzone_mm     = 10.0f,   /* 到点死区 (mm) */
    .pos_min_speed       = 0.0f,    /* 定位速度下限: 0=关; mission 启动区那一段临时打开 */
    .pos_creep           = CHASSIS_POS_CREEP_RPM,  /* 末端蠕动下限: 死区外保证能动 (car_config.h) */
    .pos_fwd_mm          = 1200.0f,  /* btn1 前进距离 (mm) */
    .trim_gain_x         = 1.0f,
    .trim_gain_y         = 1.0f,
    .trim_gain_yaw       = 1.0f,
    .qr_scan_speed       = CHASSIS_SPEED_QR,     /* 扫码直行 (car_config.h) */
    .acc                 = CHASSIS_ACC,          /* EMM 加速度档位 0~255 (car_config.h) */
};
ChassisVal_t chassis_val = {0};   /* 运行状态: 目标/当前朝向, 里程计, 纠偏值, 电压 (全部运行时填) */

/* Gz 角速度低通滤波: 原始陀螺仪角速度噪声会被 -kd_rate 阻尼项放大, 先滤再阻尼 */
static Math_LPF_t chassis_yaw_rate_lpf;
static uint8_t    chassis_yaw_rate_lpf_inited = 0;

/* 底盘自判纠偏到位的连续帧计数 (见 Chassis_ApplyVisionTrim 的 trim_conv 部分) */
static uint16_t s_trim_conv_cnt = 0;
/* 本次进站是否已经回过 0x11 AlignDone (每站只回一条, Chassis_TrimArm 清零) */
static uint8_t  s_align_done_sent = 0;

/* 整场重来请求: Chassis_ResetHeading() (DATE 任务) 置 1, App_Chassis_Task 下一拍清一次
   偏航 PID 积分。yaw_pid 是任务里的静态量, 外面拿不到, 只能这样递个请求, 不跨任务抢写。 */
static uint8_t s_yaw_pid_clear_req = 0;

static float Chassis_YawRateFilt(void)
{
    if (!chassis_yaw_rate_lpf_inited)
    {
        Math_LPF_Init(&chassis_yaw_rate_lpf, chassis_cfg.yaw_rate_lpf_alpha);
        chassis_yaw_rate_lpf_inited = 1;
    }
    chassis_yaw_rate_lpf.alpha = chassis_cfg.yaw_rate_lpf_alpha;  /* 允许 Watch 实时改 */
    return Math_LPF_Calc(&chassis_yaw_rate_lpf, Imu_GetYawRate());
}

/* 统一偏航修正 omega = P项 + 阻尼项: 拆出 P项/阻尼项供 VOFA 观测, 并维护
   chassis_val.yaw_target = 实际传给 PID 的目标 (修遥测 bug: 原地旋转用 chassis_val.rotate_target,
   不能留旧 chassis_val.yaw_target)。
   ★ Chassis_YawRateFilt() 每拍只能调一次 (推进 LPF 状态), 遥测处只读 chassis_val.yaw_rate_f 副本, 不得再调。 */
static float Chassis_YawOmega(PID_Controller_t *pid, float target, float yaw)
{
    float rate_f;
    float damp;
    float om;

    rate_f = Chassis_YawRateFilt();
    chassis_val.yaw_p_term = chassis_cfg.yaw_invert * PID_Calc(pid, target, yaw);

    damp = -chassis_cfg.yaw_kd_rate * rate_f;
    if (damp >  chassis_cfg.yaw_damp_max) damp =  chassis_cfg.yaw_damp_max;
    if (damp < -chassis_cfg.yaw_damp_max) damp = -chassis_cfg.yaw_damp_max;
    if (fabsf(rate_f) < chassis_cfg.yaw_rate_dz) damp = 0.0f;   /* 噪声不进电机 */

    chassis_val.yaw_damp   = damp;
    chassis_val.yaw_rate_f = rate_f;
    chassis_val.yaw_target = target;

    /* 最小修正量(默认关): 慢速时 P 要的零点几 RPM 可能小于 EMM 量化台阶 -> 四轮拿到同一整数命令
       -> 差速 0 -> 不修正。给 omega 一个下限, 保证"决定修就真产生差速"。只动 omega, 不动 vx/vy。 */
    om = chassis_val.yaw_p_term + damp;
    if (chassis_flag.min_omega_en && fabsf(om) > 0.01f && fabsf(om) < chassis_cfg.min_omega)
        om = (om > 0.0f) ? chassis_cfg.min_omega : -chassis_cfg.min_omega;
    return om;
}

/* EMM 使能/失能: [addr,0xF3,0xAB,state,snF,0x6B]; 使能=锁轴 失能=松轴 */
static void Emm_En(uint8_t addr, uint8_t state)
{
    uint8_t cmd[6];
    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = state;
    cmd[4] = 0;      /* snF */
    cmd[5] = 0x6B;
    can_SendCmd(cmd, 6);
}

/* EMM 速度模式: [addr,0xF6,dir,vel_hi,vel_lo,acc,snF,0x6B]; dir 0=CW 1=CCW
   snF: 0=立即执行, 1=缓存(暂存到电机, 等同步触发命令一起执行) */
static void Emm_Vel(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
    uint8_t cmd[8];
    if (vel > 3000) vel = 3000;   /* EMM 速度上限 3000RPM */
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = 0x6B;
    can_SendCmd(cmd, 8);
}

/* EMM 同步运动触发: [0,0xFF,0x66,0x6B]; 广播地址0, 让所有缓存(snF=1)的命令同时执行 */
static void Emm_SyncTrigger(void)
{
    uint8_t cmd[4];
    cmd[0] = 0;      /* 广播地址 */
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;
    can_SendCmd(cmd, 4);
}

/* EMM 定时主动上报位置: [addr,0x11,0x18,0x36,time_hi,time_lo,0x6B]
   time_ms>0 = 设一次持续生效, 电机每隔 time_ms 主动发一次位置帧 (回包格式同 0x36 读取);
   time_ms=0 = 停止上报 (手册 5.5.1 示例3)。 */
static void Emm_AutoReturn_Pos(uint8_t addr, uint16_t time_ms)
{
    uint8_t cmd[7];
    cmd[0] = addr;
    cmd[1] = 0x11;
    cmd[2] = 0x18;
    cmd[3] = 0x36;
    cmd[4] = (uint8_t)(time_ms >> 8);
    cmd[5] = (uint8_t)(time_ms >> 0);
    cmd[6] = 0x6B;
    can_SendCmd(cmd, 7);
}

/* 四轮一起使能/失能 (地址 1~4) */
static void Chassis_EnAll(uint8_t state)
{
    Emm_En(1, state);
    Emm_En(2, state);
    Emm_En(3, state);
    Emm_En(4, state);
}

/* 立即失能四轮 (松轴), 不等 App_Chassis_Task 的 en 跳变检测 (那是下一拍才发)。
   运行中长按一键键中止(misson 调试重启)时调: 车要人手搬回启动区, 轮子必须松开。
   同步写 chassis_flag.en=0: 任务下一拍按 en 跳变还会补发一次失能, 重复发无害。 */
void Chassis_DisableMotors(void)
{
    chassis_flag.en = 0;
    Chassis_EnAll(0);
}

/* 发送 4 轮速度 (EMM 同步运动): 每轮一条缓存(snF=1)速度命令, 最后广播 0xFF 触发四轮同时执行,
   起停同步性比逐条立即执行好; 回包 ACK 由中断逐个吃掉, 阻塞读前先清 rxFrameFlag 无冲突 */
static void Chassis_SendWheels(float w_bl, float w_br, float w_fl, float w_fr)
{
    Emm_Vel(1, (w_bl < 0) ? 1 : 0, (uint16_t)fabsf(w_bl), chassis_cfg.acc, 1);
    Emm_Vel(2, (w_br < 0) ? 1 : 0, (uint16_t)fabsf(w_br), chassis_cfg.acc, 1);
    Emm_Vel(3, (w_fl < 0) ? 1 : 0, (uint16_t)fabsf(w_fl), chassis_cfg.acc, 1);
    Emm_Vel(4, (w_fr < 0) ? 1 : 0, (uint16_t)fabsf(w_fr), chassis_cfg.acc, 1);
    Emm_SyncTrigger();
}

/* 麦轮混控 + 旋转优先限幅 + 四轮同步发。
   前进=左右反向 平移=前后反向 旋转=四轮同向 (vy 符号由 MECANUM_VY_SIGN 统一控制)。
   旋转优先: 平移分量每轮最大 |vx|+|vy|, 旋转分量 |omega| 同号加四轮; 二者之和
   超每轮预算 chassis_cfg.wheel_max 时等比压缩平移、保 omega 全额 —— 高速直行时
   车头矫正始终有余量不被平移吃掉。chassis_flag.rot_prio_en=0 退回裸混控。 */
static void Chassis_MixWheels(float vx, float vy, float omega)
{
    float w_bl, w_br, w_fl, w_fr;
    float t, k;

    if (chassis_flag.rot_prio_en)
    {
        t = fabsf(vx) + fabsf(vy);
        if (t > 0.0f && t + fabsf(omega) > chassis_cfg.wheel_max)
        {
            k = (chassis_cfg.wheel_max - fabsf(omega)) / t;
            if (k < 0.0f) k = 0.0f;      /* omega 已顶满预算: 平移全让位 */
            if (k > 1.0f) k = 1.0f;
            vx *= k;
            vy *= k;
        }
    }

    w_bl =  vx - MECANUM_VY_SIGN * vy + omega;
    w_br = -vx - MECANUM_VY_SIGN * vy + omega;
    w_fl =  vx + MECANUM_VY_SIGN * vy + omega;
    w_fr = -vx + MECANUM_VY_SIGN * vy + omega;

    Chassis_SendWheels(w_bl, w_br, w_fl, w_fr);
}

/* 读底盘电池电压: 电机1母线电压 (=电池电压, 四轮同一条电源线)
   走 0x24 单读总线电压 (手册 5.5.4), 回包 Data[1..2]=16位 mV 大端。
   非阻塞: 回包由 CAN 中断填 can_vbus_mv[1]/can_vbus_flag[1]。
   拆成"每拍消费"和"500ms 请求"两半:
   - 消费每拍查 (成本极小), 回包一到下一拍 chassis_val.vbus 就更新, 不再慢一包;
   - 请求节拍不变 (500ms 一帧, 不跟速度命令抢总线)。
   两者都放在 App_Chassis_Task 最前面: 遥控锁定/扫码/旋转那三个提前 return 的分支
   原来会把底部的读取整个跳过, 车在 IDLE 等启动时电压根本不刷新。 */
static void Chassis_VBusPoll(void)
{
    if (can_vbus_flag[1])
    {
        can_vbus_flag[1] = 0;
        chassis_val.vbus = can_vbus_mv[1] / 1000.0f;   /* mV -> V */
    }
}

static void Chassis_VBusRequest(void)
{
    uint8_t cmd[3];

    cmd[0] = 1; cmd[1] = 0x24; cmd[2] = 0x6B;   /* 单读电机1总线电压 (0x24) */
    can_SendCmd(cmd, 3);                        /* 发读请求(非阻塞) */
}

/* ================= 麦轮里程计 (BL/BR/FL/FR = 电机 1/2/3/4) ================= */
static uint8_t chassis_odom_inited = 0;
static float   chassis_odom_prev_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f}; /* BL BR FL FR 上周期电机角度(度) */
static uint16_t chassis_odom_prev_cnt[4] = {0, 0, 0, 0}; /* 上一周期各轮上报帧计数 (新鲜度检查用) */
static uint8_t  chassis_autorep_prev = 0;                 /* 上次 autorep 开关状态 (检测 1->0 跳变, 发关闭命令) */
static uint16_t chassis_autorep_rearm = 0;                /* 上膛命令节拍: 0=本拍重发, 满 20 拍(1s)再重发 */
static uint8_t  chassis_autorep_once  = 0;                /* 本次 autorep 开启后是否已发过首次上膛命令 */

/* 里程计逐轮诊断 (Keil Watch 只读, 故意不加 static):
   chassis_odom_dmm[k]    第 k 轮本拍位移 (mm, BL BR FL FR)
   chassis_odom_reject[k] 第 k 轮被判为"位置不连续"而丢弃的次数 —— 哪个轮子在跳一目了然 */
float    chassis_odom_dmm[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
uint16_t chassis_odom_reject[4] = {0, 0, 0, 0};

/* 位置帧率诊断 (Keil Watch 直接看, 不用掐表/不会读错):
   can_pos_rate[k]     上一秒内地址 k 收到的 0x36 位置帧数 (= 帧/秒), 每秒刷新一次
   can_pos_rate_max[k] 历史最大帧率, 抓偶发飙高 (别的设备抢这个地址) / 偶发丢帧
   正常: 底盘 1~4 都应为 50 (CHASSIS_AUTOREP_MS=20)。明显低于 50 = 该电机定时上报没生效
   或总线上丢帧; 明显高于 50 = 这个地址上有第二个设备在发帧。塔吊 5/6 是轮询读取, 约 20/秒。
   索引 0 不用, 1~MOTOR_ADDR_MAX 对应电机地址。 */
__IO uint16_t can_pos_rate[7]     = {0};
__IO uint16_t can_pos_rate_max[7] = {0};

/* 里程计新鲜度诊断: 每 50ms 一拍, 哪个轮子"本拍没收到新帧"(只能保持上一拍位置, 该轮位移记 0)。
   chassis_odom_stale[k] 累计次数, chassis_odom_cycles 是总拍数 -> 丢帧率 = stale / cycles */
uint16_t chassis_odom_stale[4]  = {0, 0, 0, 0};
uint32_t chassis_odom_cycles    = 0;

/* 上报间隔诊断 (比帧率更直接: 帧率是"一秒平均", 这个是"两帧之间最长隔了多久"):
   can_pos_gap_cur[k] 距离该地址上一帧已经过了多少 ms (实时值, 每 10ms 刷新)
   can_pos_gap_max[k] 上电以来相邻两帧的最大间隔 (ms)
   正常 (CHASSIS_AUTOREP_MS=20): cur 在 0~20 之间跳, max ≤ 30。
   cur 一直停在 60/100 以上 = 这个电机正在停报; max 上百 = 中间丢过一串帧。
   注意: 10ms 任务刷新, 所以读数带 ±10ms 量化, 20ms 周期会看到 10~20。 */
__IO uint16_t can_pos_gap_cur[7]      = {0};
__IO uint16_t can_pos_gap_max[7]      = {0};
__IO uint32_t can_pos_gap_max_tick[7] = {0};   /* 最大间隔出现的那一刻 (上电后 ms): <2000 = 上电/使能瞬态, 几万 = 运行中真的断了 */
__IO uint16_t can_pos_gap_bad[7]      = {0};   /* 间隔 > 60ms 的累计次数: 不复位就一直是那个数 = 一次性事件; 一直涨 = 正在丢 */

/* 相对车头航向: 长按 key 时把当前 Imu_GetYaw() 记为车头基准 chassis_val.yaw_ref,
   之后所有"航向/目标 yaw"都用这个相对值 (0=正前方=车头基准, CCW 为正)。 */
float Chassis_GetHeading(void)
{
    return Math_NormalizeAngle(Imu_GetYaw() - chassis_val.yaw_ref);
}

/* 建立车头基准: 记录当前 yaw 为 0°, 并清里程计/定位目标, 供长按 key 启动时调用 */
/* 记录当前 yaw 为车头基准 + 里程计清零 + 这一场的目标/残留全部归零。
   长按启动 / 运行中中止(整场重来)都走这里, 新一场的状态要和刚上电一致:
   定位目标、旋转目标、纠偏残量、停靠快照、偏航 PID 积分, 一样都不带过去。 */
void Chassis_ResetHeading(void)
{
    chassis_val.yaw_ref = Imu_GetYaw();
    chassis_val.odom_x_mm = 0.0f;
    chassis_val.odom_y_mm = 0.0f;
    chassis_val.odom_yaw_deg = 0.0f;
    chassis_val.odom_vx_mmps = 0.0f;
    chassis_val.odom_vy_mmps = 0.0f;
    chassis_odom_inited = 0;   /* 下次里程计更新重新记基准 (prev_pos 重新采样, 不积分) */
    chassis_val.yaw_target = 0.0f;

    /* 定位/旋转/纠偏的残留目标 (刚上电是 0; 上一场留下的目标会被下一次启动直接拿去用) */
    chassis_val.pos_target_x_mm = 0.0f;
    chassis_val.pos_target_y_mm = 0.0f;
    chassis_val.rotate_target   = 0.0f;
    chassis_val.trim_err_x   = 0.0f;
    chassis_val.trim_err_y   = 0.0f;
    chassis_val.trim_err_yaw = 0.0f;
    chassis_val.trim_corr_x  = 0.0f;
    chassis_val.trim_corr_y  = 0.0f;
    chassis_val.trim_corr_yaw = 0.0f;
    chassis_val.dock_corr_x  = 0.0f;
    chassis_val.dock_corr_y  = 0.0f;
    s_trim_conv_cnt = 0;

    /* 偏航 PID 积分 + Gz 低通: 下一拍清/重新初始化, 不把上一场的累积带进新一场 */
    s_yaw_pid_clear_req = 1;
    chassis_yaw_rate_lpf_inited = 0;
}

/* 工作点视觉纠偏开关: 按位图判断该点是否跳过纠偏 (bit0=物料 bit1=粗码 bit2=暂存) */
uint8_t Chassis_TrimSkipStation(NavPointId id)
{
    if (id == NAV_MATERIAL) return (chassis_flag.trim_skip & 0x01) ? 1 : 0;
    if (id == NAV_ROUGH)    return (chassis_flag.trim_skip & 0x02) ? 1 : 0;
    if (id == NAV_STAGING)  return (chassis_flag.trim_skip & 0x04) ? 1 : 0;
    return 0;
}

/* 底盘自判纠偏到位的连续帧计数 s_trim_conv_cnt 定义见文件顶部静态区 */

/* ---- 进站视觉微调武装 (mission MS_TRIM / kin_flag.cmd=11 共用) -------------
   为什么需要: trim_within 只在收到 0x04/0x05 帧时被覆盖, 从不复位。若到站后
   视觉还没发过一帧, 会拿上一站残留的 within=1 直接放行, "到探查位再微调"这一
   步等于没做。武装后只认 Release 之后的新帧; 塔吊还在摆姿态时(hold=1)帧直接丢掉。
   注意: 视觉若在该工作区根本不发 0x04/0x05, TrimDone 永远为 0, 调用方要自带超时。 */
void Chassis_TrimArm(void)
{
    chassis_flag.trim_within = 0;
    chassis_flag.trim_fresh  = 0;
    chassis_flag.trim_conv   = 0;
    s_trim_conv_cnt          = 0;
    s_align_done_sent        = 0;   /* 新站: 允许再回一条 0x11 AlignDone */
    chassis_flag.trim_armed  = 1;
    chassis_flag.trim_hold   = 1;
    chassis_flag.trim_block  = 0;   /* 进新站: 解除上一站"电机6 已伸出"的封死 */
}

void Chassis_TrimRelease(void)
{
    Serial_ChassisTrimReady   = 0;   /* 姿态期间积压的旧帧丢掉, 只认放开之后的新帧 */
    Serial_TurntableTrimReady = 0;   /* 0x0E 同理: 摆姿态时收到的转盘帧一律作废 */
    chassis_flag.trim_hold  = 0;
}

void Chassis_TrimDisarm(void)
{
    chassis_flag.trim_hold  = 0;
    chassis_flag.trim_armed = 0;
    chassis_flag.trim_fresh = 0;
    chassis_flag.trim_conv  = 0;
    s_trim_conv_cnt         = 0;
}

/* 本站纠偏封死: 本站不再收任何视觉纠偏帧, 直到下一次 Chassis_TrimArm() (进新站摆姿态) 解封。
   两种情况用:
   1) 微调已经到位、电机6 要伸出去作业了。伸出去的夹爪/电机6 会挡在摄像头前面,
      视觉算出来的 corr 就是错的, 再纠就走到别的点上, 所以从这一刻起一帧都不收;
   2) 跳过本站作业直接离站 (MS_TRIM 的 SKIP 分支): 车马上要开走, 武装还挂着的话
      转盘节点(常开, 行车途中也发)来一帧 within=1 就会把车拽回去, 所以离站前也封死。 */
void Chassis_TrimBlock(void)
{
    chassis_flag.trim_hold    = 0;
    chassis_flag.trim_armed   = 0;
    chassis_flag.trim_fresh   = 0;
    chassis_flag.trim_conv    = 0;
    s_trim_conv_cnt           = 0;
    chassis_flag.trim_block   = 1;
    Serial_ChassisTrimReady   = 0;
    Serial_TurntableTrimReady = 0;
}

/* "纠偏到位"= 视觉说到了, 或者底盘自己判到了:
   视觉 within 是它自己的容差 (叠加位 2mm), 比底盘死区 (pos_deadzone_mm 10mm) 紧得多,
   底盘永远达不到 -> 只等 within 的话, 叠加位那一步会永久卡住 (全自动只是靠 trim_wait 超时放行)。
   底盘自判 (trim_conv) 的含义是"要纠的量已经落进我自己的死区, 再纠车也不会动了",
   这时候继续往下走用的是 0x0C 给的云台系落点 (相对车算), 不会因为这点车位偏差报废。 */
uint8_t Chassis_TrimDone(void)
{
    return (chassis_flag.trim_armed && chassis_flag.trim_fresh &&
            (chassis_flag.trim_within || chassis_flag.trim_conv)) ? 1 : 0;
}

/* 阻塞读单个电机实时位置(度): 成功写 *pos_deg 返回1, 失败返回0
   EMM 固件: 65536 计数 = 360度 (X 固件是 0.1度, 别混)
   改为复用 CAN 中断维护的 can_pos_* 数组: 等 can_pos_cnt[addr] 变化 (签名/返回值不变)。
   autorep 开着时 cnt 会被周期上报推进, 本函数会拿到最新的自动上报值 (预期行为)。 */
static uint8_t Chassis_Odom_ReadPos(uint8_t addr, float *pos_deg)
{
    uint8_t  cmd[3];
    uint16_t c0 = can_pos_cnt[addr];
    uint32_t t  = 0;

    cmd[0] = addr; cmd[1] = 0x36; cmd[2] = 0x6B;   /* 读实时位置 (EMM) */
    can_SendCmd(cmd, 3);

    while (can_pos_cnt[addr] == c0 && ++t < 2000000) { }   /* 等 cnt 变化 */
    if (can_pos_cnt[addr] == c0) return 0;                 /* 超时 */

    *pos_deg = (float)can_pos_raw[addr] * 360.0f / 65536.0f;   /* EMM: 65536 = 360° */
    if (can_pos_sign[addr]) *pos_deg = -*pos_deg;
    return 1;
}

/* 非阻塞轮询读位置 (chassis_flag.odom_poll=1 时启用):
   每 10ms 一拍, 轮流给一个轮子发"读实时位置"(0x36), 4 拍轮完一圈 = 每轮 40ms 一个新鲜样本。
   回包走中断槽 can_pos_raw/cnt, 和定时上报完全同一条解码路径, 区别只是"我们主动问"
   而不是"等电机自己报" —— 实测电机定时上报会自己停 (看 can_pos_gap_bad/can_pos_rate),
   主动问不受它影响。不阻塞: 发完就走, 回包下一拍到了就看 cnt 涨没涨。 */
static void Chassis_Odom_PollTick(void)
{
    static uint8_t slot = 0;
    uint8_t cmd[3];

    cmd[0] = (uint8_t)((slot & 3u) + 1u);   /* 1,2,3,4 轮流 */
    cmd[1] = 0x36;
    cmd[2] = 0x6B;
    can_SendCmd(cmd, 3);

    slot = (uint8_t)((slot + 1u) & 3u);
}

static void Chassis_Odom_Update(void)
{
    float pos[4], new_pos, yaw_deg;
    uint8_t k;
    uint16_t cnt;
    uint8_t autorep_on;

    /* 定时上报本拍是否在用: 开了 odom_poll (主动轮询) 就不用电机自己报 */
    autorep_on = (chassis_flag.odom_use_autorep && !chassis_flag.odom_poll) ? 1u : 0u;

    /* autorep 1->0 跳变: 发 time_ms=0 关掉电机定时上报, 否则退回阻塞读会被上报帧干扰 */
    if (!autorep_on && chassis_autorep_prev)
    {
        Emm_AutoReturn_Pos(1, 0);
        Emm_AutoReturn_Pos(2, 0);
        Emm_AutoReturn_Pos(3, 0);
        Emm_AutoReturn_Pos(4, 0);
        chassis_autorep_rearm = 0;      /* 下次重新开启时立刻上膛, 不等到节拍到点 */
        chassis_autorep_once  = 0;
    }
    chassis_autorep_prev = autorep_on;

    if (autorep_on)
    {
        /* 上膛命令 (0x11 0x18 0x36 <ms>) 的两条铁律, 都是 2026-11-08 台架实测撞出来的:
           ① 不能每拍重发 —— 电机内部的 20ms 定时器每收到一次就重新计时;
           ② 更不能"四个电机连着一起发" —— 那会把四轮的定时器对齐到同一相位, 之后每 20ms
              四帧同时抢仲裁。CAN 仲裁按 ID 定胜负, 电机 N 的上报 ID = N<<8, 低地址永远赢,
              3/4 号连续输够了自己停报。实测(静止 3 分钟)全量重发时 stale 13% / gap_bad 200,
              改成只救停报的那一个之后 stale 0.2% / gap_bad 6。见 docs/decisions/010。
           => 默认 autorep_rearm_en=0 (只在上电时装一次), 日常靠下面"谁停了就单独救谁"。 */
        if (chassis_autorep_rearm == 0)
        {
            /* 首次必发; 之后是否每秒全量重发由 chassis_flag.autorep_rearm_en 决定 (默认 0)。 */
            if (!chassis_autorep_once || chassis_flag.autorep_rearm_en)
            {
                Emm_AutoReturn_Pos(1, CHASSIS_AUTOREP_MS);
                Emm_AutoReturn_Pos(2, CHASSIS_AUTOREP_MS);
                Emm_AutoReturn_Pos(3, CHASSIS_AUTOREP_MS);
                Emm_AutoReturn_Pos(4, CHASSIS_AUTOREP_MS);
                chassis_autorep_once = 1;
            }
        }
        if (++chassis_autorep_rearm >= 20) chassis_autorep_rearm = 0;   /* 20 拍 x 50ms = 1s */

        /* 停报急救 (2026-11-08 台架实测后加): 电机定时上报跑几十秒会**自己停**,
           实测停过 46 次/分钟, 靠上面"每秒重发一次"把它点回来 —— 但最坏要等整整 1 秒,
           这 1 秒里该轮位移记 0 (车速 300mm/s 就丢约 75mm), 这就是"导航偶尔跑不准"的来源。
           改成逐轮盯梢: 谁超过 100ms 没新帧, 就单独给它补一条, 最坏只丢 ~100ms。
           健康的轮子一条都不发, 不增加总线负担。 */
        for (k = 0; k < 4; k++)
        {
            if (can_pos_gap_cur[k + 1] > 100u)
            {
                Emm_AutoReturn_Pos((uint8_t)(k + 1), CHASSIS_AUTOREP_MS);
            }
        }
    }

    /* 定时上报 或 主动轮询: 两种模式的数据都落在同一个中断槽 can_pos_raw/cnt,
       新鲜度判断完全一样, 所以只在"要不要发命令"上有区别。 */
    if (autorep_on || chassis_flag.odom_poll)
    {
        /* 逐轮独立处理: 本拍收到新帧就用新位置, 丢帧则保持上一拍位置(该轮本拍位移记 0)。
           不再要求 4 轮同时有新帧, 也绝不用车速外推 —— 外推+积分会重复计位移(双重积分),
           丢帧越频繁速度被放得越大, 里程计快速虚高到几千万。丢帧只让该轮本拍位移=0,
           整车里程计仍随其它新帧前进, P 控误差照常缩小, 不会冻住也不会冲过头。
           首次基准见下面 !chassis_odom_inited 分支: 要等 4 轮都真收到过一帧。 */
        for (k = 0; k < 4; k++)
        {
            cnt = can_pos_cnt[k + 1];
            if (cnt != chassis_odom_prev_cnt[k])
            {
                pos[k] = (float)can_pos_raw[k + 1] * 360.0f / 65536.0f;
                if (can_pos_sign[k + 1]) pos[k] = -pos[k];
                chassis_odom_prev_cnt[k] = cnt;
            }
            else
            {
                chassis_odom_stale[k]++;          /* 本拍没收到新帧 (诊断: 哪个轮子在掉帧) */
                pos[k] = chassis_odom_prev_pos[k];
            }
        }
    }
    else
    {
        /* 旧路径: 读 4 轮位置 (电机地址和 Chassis_SendWheels 一致: 1=BL 2=BR 3=FL 4=FR; 失败保持旧值) */
        if (Chassis_Odom_ReadPos(1, &new_pos)) pos[0] = new_pos; else pos[0] = chassis_odom_prev_pos[0];
        if (Chassis_Odom_ReadPos(2, &new_pos)) pos[1] = new_pos; else pos[1] = chassis_odom_prev_pos[1];
        if (Chassis_Odom_ReadPos(3, &new_pos)) pos[2] = new_pos; else pos[2] = chassis_odom_prev_pos[2];
        if (Chassis_Odom_ReadPos(4, &new_pos)) pos[3] = new_pos; else pos[3] = chassis_odom_prev_pos[3];
    }

    yaw_deg = Chassis_GetHeading();
    chassis_val.odom_yaw_deg = yaw_deg;

    if (!chassis_odom_inited)
    {
        /* 首次基准必须等 4 轮都收到过真实位置 (can_pos_cnt != 0)。
           上电头几拍 can_pos_raw 还是 0, 若此时就记基准, 第一帧真实位置到达时
           增量 = 电机绝对位置(可能是几千度) -> 里程计一步跳几米~几十米, 导航目标跟着飞。 */
        uint8_t ok = 1;
        for (k = 0; k < 4; k++) if (can_pos_cnt[k + 1] == 0) ok = 0;
        if (!ok) return;                       /* 还没收齐: 本拍什么都不做, 位置保持 0 */
        for (k = 0; k < 4; k++) chassis_odom_prev_pos[k] = pos[k];
        chassis_odom_inited = 1;
        return;   /* 首次只记录基准, 不积分 */
    }

    chassis_odom_cycles++;   /* 诊断分母: 与 chassis_odom_stale[] 比出各轮丢帧率 */

    /* 电机角增量(度) -> 轮弧长(mm) */
    float scale = chassis_cfg.odom_wheel_radius_mm * (MATH_PI / 180.0f) / chassis_cfg.odom_gear_ratio;

    /* 单轮跳变检查 (2026-11-01 加)。现象: 物料区->粗加工 途中里程计莫名叠加到 -10000,-10000,
       yaw 正常、目标点也正常。
       成因: EMM 报的是**多圈绝对位置**, 跑一分钟就上万度; 某一轮掉电重启 / 内部计数器复位 /
       收到坏帧, 都会让 增量 = -(该轮累计的绝对位置) -> 除以 4 之后 x/y 同量级、同符号地
       一步飞出几十米, P 环再拿这个假位置把车开跑。
       单轮只会让 dx 和 dy 同时偏(BL 电机1 / FR 电机4 偏同号, BR 电机2 / FL 电机3 偏反号),
       所以"x/y 一样大"本身就说明是单个轮子, 查 chassis_odom_reject[k] 就知道是哪个。
       超限的那一拍不积分(最多丢 30mm), 基准就地重对, 之后照常累加。 */
    for (k = 0; k < 4; k++)
    {
        float ddeg = pos[k] - chassis_odom_prev_pos[k];

        if (ddeg > ODOM_MAX_WHEEL_STEP_DEG || ddeg < -ODOM_MAX_WHEEL_STEP_DEG)
        {
            chassis_odom_reject[k]++;
            ddeg = 0.0f;                     /* 本拍不算位移 */
        }
        chassis_odom_dmm[k] = ddeg * scale;  /* 各轮本拍位移 (mm) */
        chassis_odom_prev_pos[k] = pos[k];
    }

    float d_bl = chassis_odom_dmm[0];
    float d_br = chassis_odom_dmm[1];
    float d_fl = chassis_odom_dmm[2];
    float d_fr = chassis_odom_dmm[3];

    /* 正运动学 (逆解反函数): a=BL b=BR c=FL d=FR
       vx = (a - b + c - d) / 4   前进
       vy = MECANUM_VY_SIGN * (-a - b + c + d) / 4   左移 */
    float dx_robot = (d_bl - d_br + d_fl - d_fr) / 4.0f;
    float dy_robot = MECANUM_VY_SIGN * (-d_bl - d_br + d_fl + d_fr) / 4.0f;

    /* 车体系 -> 世界系 (陀螺仪 yaw 作航向) */
    float yaw_rad = chassis_cfg.odom_yaw_invert * yaw_deg * (MATH_PI / 180.0f);
    float c = cosf(yaw_rad), s = sinf(yaw_rad);
    chassis_val.odom_x_mm += dx_robot * c - dy_robot * s;
    chassis_val.odom_y_mm += dx_robot * s + dy_robot * c;

    /* 车体速度 (mm/s): 周期 = chassis_cfg.odom_update_cycles * 10ms */
    float cyc = (chassis_cfg.odom_update_cycles > 0) ? (float)chassis_cfg.odom_update_cycles : 1.0f;
    float dt_s = (10.0f * cyc) / 1000.0f;
    chassis_val.odom_vx_mmps = dx_robot / dt_s;
    chassis_val.odom_vy_mmps = dy_robot / dt_s;
}

/* 底盘用懒初始化: 偏航 PID / 里程计基准在任务首次进入时自动建立, 上电无需额外动作 */
void App_Chassis_Init(void)
{
}

/**
  * @brief  视觉底盘纠偏: 收到 0x04/0x05 帧后, 先更新可观测状态
  *         (偏了多少 err / 要矫正多少 corr / 是否矫正成功 within),
  *         未到位(within=0)时以当前里程计位姿为基准, 把 corr 旋转到世界系
  *         叠加成新目标并再定位。corr 是"当前还差多少"(绝对误差), 每次以
  *         当前位置重算目标, 收敛后 corr≈0、within=1 不再动。
  */
void Chassis_ApplyVisionTrim(void)
{
    /* 可观测: 偏了多少 / 要矫正多少(含增益) / 是否矫正成功 */
    chassis_val.trim_err_x   = Serial_RxData.ring_err_x;
    chassis_val.trim_err_y   = Serial_RxData.ring_err_y;
    chassis_val.trim_err_yaw = Serial_RxData.ring_ang_err;
    chassis_val.trim_corr_x   = chassis_cfg.trim_gain_x   * Serial_RxData.ring_corr_x;
    chassis_val.trim_corr_y   = chassis_cfg.trim_gain_y   * Serial_RxData.ring_corr_y;
    chassis_val.trim_corr_yaw = chassis_cfg.trim_gain_yaw * Serial_RxData.ring_corr_yaw;
    chassis_flag.trim_within   = Serial_RxData.ring_within;
    if (chassis_flag.trim_armed) chassis_flag.trim_fresh = 1;   /* 标记: 武装之后收到过新帧 */

    /* 底盘自判到位 (trim_conv): 要纠的量已经落进底盘自己的死区 —— 下面点对点定位里
       ef/el 等价于 -trim_corr_y / trim_corr_x, yaw 用 PID 死区, 三者都比我自己的死区小
       就意味着"目标算出来也不会走"。连续 CHASSIS_TRIM_CONV_FRAMES 帧都成立才认位, 防单帧抖。
       为什么要它: 视觉 within 用的是它自己的容差 (叠加位 2mm), 比 pos_deadzone_mm(10mm) 紧,
       底盘永远达不到, 只等 within 会卡死在"等微调"那一步。 */
    if (chassis_flag.trim_armed)
    {
        if (fabsf(chassis_val.trim_corr_x)   < chassis_cfg.pos_deadzone_mm &&
            fabsf(chassis_val.trim_corr_y)   < chassis_cfg.pos_deadzone_mm &&
            fabsf(chassis_val.trim_corr_yaw) < chassis_cfg.yaw_deadzone)
        {
            if (s_trim_conv_cnt < 250) s_trim_conv_cnt++;
            if (s_trim_conv_cnt >= CHASSIS_TRIM_CONV_FRAMES) chassis_flag.trim_conv = 1;
        }
        else
        {
            s_trim_conv_cnt = 0;
        }
    }

    /* 区域矫正完成回传 (0x11 AlignDone: "stage,success"; 视觉 race_supervisor 靠它启动下一阶段):
       每进站 (Chassis_TrimArm) 最多回一条 success=1, 发在"底盘判到位"那一拍 —— 和 mission MS_TRIM
       放行用的是同一个条件 (视觉 within 或 底盘自判 trim_conv)。
       不逐帧回: 视觉收到 success=0 只会打"继续等待"日志, 10Hz 会刷屏, 所以失败不发。
       stage 取 Serial_TrimFrameType (解析那拍记下来的), 不用 Serial_RxData.type —— 两拍之间可能
       又插进来一帧 0x02/0x03, 那样 stage 就定错甚至不回了。
       stage: 1=物料区转盘(0x0E) 2=粗加工三圆环(0x04) 3=暂存叠料(0x05)。 */
    if (Serial_AlignDoneEn && !s_align_done_sent &&
        chassis_flag.trim_armed && chassis_flag.trim_fresh &&
        (chassis_flag.trim_within || chassis_flag.trim_conv))
    {
        uint8_t stage = 0;

        switch (Serial_TrimFrameType)
        {
        case SERIAL_TYPE_TURNTABLE_ALIGN: stage = 1; break;   /* 物料区转盘 */
        case SERIAL_TYPE_RING_STATION:    stage = 2; break;   /* 粗加工三圆环 */
        case SERIAL_TYPE_STACKED_RING:    stage = 3; break;   /* 暂存叠料 */
        default: break;
        }
        if (stage)
        {
            Serial_SendAlignDone(stage, 1);
            s_align_done_sent = 1;
        }
    }

    if (chassis_flag.trim_within) return;   /* 已在容差内(矫正成功), 不再纠偏 */

    {
        float yaw = Chassis_GetHeading();
        float th  = chassis_cfg.odom_yaw_invert * yaw * (MATH_PI / 180.0f);
        float c = cosf(th), s = sinf(th);

        /* 视觉 corr 坐标系与里程计车体系差 90°: corr_x+ = 里程计 y+(左), corr_y+ = 里程计 x-(后)。
         * 先换成里程计车体分量(前/左), 再旋转到世界系 (与里程计车体->世界同一旋转)。 */
        float bx = -chassis_val.trim_corr_y;   /* 车体前进分量 (corr_y+ -> x-) */
        float by =  chassis_val.trim_corr_x;   /* 车体左移分量 (corr_x+ -> y+) */

        chassis_val.pos_target_x_mm = chassis_val.odom_x_mm + (bx * c - by * s);
        chassis_val.pos_target_y_mm = chassis_val.odom_y_mm + (bx * s + by * c);
        chassis_val.yaw_target      = yaw + chassis_val.trim_corr_yaw;
        chassis_flag.pos_active = 1;   /* 再定位到修正后目标 */
    }
}

/**
  * @brief  原地旋转到世界系 yaw (mission 用): 设目标, 底盘任务里 PID 转到位后清 active
  *         转完后点对点继续用该 yaw 锁车头 (chassis_val.yaw_target 一并更新)。
  */
void Chassis_RotateTo(float yaw_deg)
{
    chassis_val.rotate_target = yaw_deg;
    chassis_val.yaw_target    = yaw_deg;
    chassis_flag.rotate_active = 1;
}

/**
  * @brief  底盘单项测试: 按 chassis_flag.test_mode 跑 前进/后退/左右平移 + 顺逆时针旋转
  *         平移(1~4)时用陀螺仪偏航 PID 锁车头走直线
  */
void App_Chassis_Test_Task(void)
{
    static PID_Controller_t yaw_pid;
    static uint8_t yaw_pid_inited = 0;
    float s = chassis_cfg.test_speed;
    float vx = 0, vy = 0, base_omega = 0;
    float yaw, yaw_corr, omega;

    yaw = Imu_GetYaw();
    chassis_val.yaw_cur = yaw;

    /* PID 初始化(一次) + 每周期把 Watch 里的增益同步进 PID */
    if (!yaw_pid_inited)
    {
        PID_Init(&yaw_pid, chassis_cfg.yaw_kp, chassis_cfg.yaw_ki, chassis_cfg.yaw_kd,
                 chassis_cfg.yaw_max_out, chassis_cfg.yaw_max_i,
                 chassis_cfg.yaw_deadzone, chassis_cfg.yaw_iband, 0.5f, 1);   /* is_angle_loop=1 */
        yaw_pid_inited = 1;
    }
    yaw_pid.Kp = chassis_cfg.yaw_kp;
    yaw_pid.Ki = chassis_cfg.yaw_ki;
    yaw_pid.Kd = chassis_cfg.yaw_kd;
    yaw_pid.max_out = chassis_cfg.yaw_max_out;
    yaw_pid.max_i = chassis_cfg.yaw_max_i;
    yaw_pid.deadzone = chassis_cfg.yaw_deadzone;
    yaw_pid.i_band = chassis_cfg.yaw_iband;

    /* 速度分量 (按实测映射: 前进=左右反向, 平移=前后反向, 旋转=四轮同向) */
    switch (chassis_flag.test_mode)
    {
    case 1: vx =  s; break;            /* 前进 */
    case 2: vx = -s; break;            /* 后退 */
    case 3: vy =  s; break;            /* 左移 */
    case 4: vy = -s; break;            /* 右移 */
    case 5: base_omega =  s; break;    /* 顺时针 */
    case 6: base_omega = -s; break;    /* 逆时针 */
    case 0:
    default: break;                    /* 停止 */
    }

    /* 停止时锁当前朝向 + 清 PID 积分 */
    if (chassis_flag.test_mode == 0)
    {
        chassis_val.yaw_target = yaw;
        PID_Clear(&yaw_pid);
    }

    /* 平移(1~4)时用陀螺仪修正车头; 旋转(5/6)不修正 */
    yaw_corr = 0;
    if (chassis_flag.test_mode >= 1 && chassis_flag.test_mode <= 4 && chassis_flag.yaw_enable)
    {
        yaw_corr = chassis_cfg.yaw_invert * PID_Calc(&yaw_pid, chassis_val.yaw_target, yaw);
    }
    chassis_val.yaw_out = yaw_corr;

    omega = base_omega + yaw_corr;

    Chassis_MixWheels(vx, vy, omega);   /* 混控 + 旋转优先限幅 + 四轮同步发 */
}

/**
  * @brief  遥控器直控底盘 (10ms): sw[0] 锁定, Rx左右 Ry前后, Lx 转目标航向 + 陀螺仪航向保持
  *         含点对点定位 (btn0回原点 / btn1往前) + 里程计 + 电池电压
  */
void App_Chassis_Task(void)
{
    static PID_Controller_t yaw_pid;
    static uint8_t yaw_pid_inited = 0;
    static uint16_t odom_tick = 0;
    static uint16_t vbus_tick = 0;
    static uint8_t  qr_armed = 0;    /* 扫码直行: 首次进入捕获车头朝向 */
    static uint8_t  rot_armed = 0;   /* 原地旋转: 首次进入清 PID 积分 */
    static uint16_t rot_stable_tick = 0;   /* 原地旋转到位稳定计数: 连续落在死区内满 rot_settle_ticks 才判转完 */
    static uint8_t  s_en_prev = 0;       /* 初始与 chassis_flag.en 一致, 避免上电误触发 */
    static uint16_t s_en_boot_tick = 0;
    static uint8_t  s_en_boot_done = 0;
    static TickType_t rate_last_tick = 0;          /* 帧率统计: 上次刷新的时刻 (ms) */
    static uint16_t   rate_prev_cnt[7] = {0};      /* 帧率统计: 上次刷新时的 can_pos_cnt 快照 */
    static uint16_t   gap_prev_cnt[7] = {0};       /* 帧间隔统计: 上次检查时的计数快照 */
    static TickType_t gap_last_tick[7] = {0};      /* 帧间隔统计: 上一帧到达的时刻 (ms) */
    float vx, vy, omega, yaw;

    /* 上报间隔统计: 每 10ms 看一眼各地址有没有新帧, 有就记"距上一帧隔了多久"。
       帧率(按秒平均)看不出一拍两拍地丢, 这个能。放在最前面, 不受下面提前 return 影响。 */
    {
        TickType_t now_gap = xTaskGetTickCount();
        uint8_t k;
        for (k = 1; k <= MOTOR_ADDR_MAX; k++)
        {
            uint16_t gc = can_pos_cnt[k];
            if (gc != gap_prev_cnt[k])
            {
                if (gap_last_tick[k] != 0)          /* 上电第一帧不参与 (距上电时间没意义) */
                {
                    uint32_t g_ms = (uint32_t)(now_gap - gap_last_tick[k]);
                    uint16_t g16  = (g_ms > 0xFFFFu) ? 0xFFFFu : (uint16_t)g_ms;
                    if (g_ms > 60u) can_pos_gap_bad[k]++;      /* 正常 20ms, 超过 60 = 中间确实少了帧 */
                    if (g16 > can_pos_gap_max[k])
                    {
                        can_pos_gap_max[k]      = g16;
                        can_pos_gap_max_tick[k] = (uint32_t)now_gap;   /* 记下"什么时候断的" */
                    }
                }
                gap_prev_cnt[k]  = gc;
                gap_last_tick[k] = now_gap;
                can_pos_gap_cur[k] = 0;
            }
            else if (gap_last_tick[k] != 0)
            {
                uint32_t g_ms = (uint32_t)(now_gap - gap_last_tick[k]);
                can_pos_gap_cur[k] = (g_ms > 0xFFFFu) ? 0xFFFFu : (uint16_t)g_ms;
            }
        }
    }

    /* 位置帧率统计: 每满 1 秒把"这一秒各地址收到多少帧"归一化成 帧/秒 写进 can_pos_rate[]。
       用真实 tick 差归零, 任务被阻塞/延后也不会把帧率算歪。
       放在本函数最前面: 下面有几处提前 return (遥控锁定等), 统计必须不受它们影响。 */
    {
        TickType_t now_tick = xTaskGetTickCount();
        if (rate_last_tick == 0) rate_last_tick = now_tick;   /* 首次进入: 记起点 */
        if ((uint32_t)(now_tick - rate_last_tick) >= 1000u)
        {
            uint32_t dt_ms = (uint32_t)(now_tick - rate_last_tick);
            uint8_t  k;
            for (k = 1; k <= MOTOR_ADDR_MAX; k++)
            {
                uint16_t now_cnt = can_pos_cnt[k];
                uint16_t d_cnt   = (uint16_t)(now_cnt - rate_prev_cnt[k]);   /* uint16 回绕天然正确 */
                rate_prev_cnt[k] = now_cnt;
                can_pos_rate[k]  = (uint16_t)((uint32_t)d_cnt * 1000u / dt_ms);
                if (can_pos_rate[k] > can_pos_rate_max[k]) can_pos_rate_max[k] = can_pos_rate[k];
            }
            rate_last_tick = now_tick;
        }
    }

    /* 四轮使能: 上电前 1 秒每拍重发一次, 确保电机就绪后一定被使能; 之后按 chassis_flag.en 跳变发 */
    if (!s_en_boot_done)
    {
        Chassis_EnAll(chassis_flag.en ? 1 : 0);
        if (++s_en_boot_tick >= 100)
        {
            s_en_boot_done = 1;
            s_en_prev = chassis_flag.en;   /* 结束重发时同步, 避免紧接着误触发一次 */
        }
    }
    else if (chassis_flag.en != s_en_prev)
    {
        s_en_prev = chassis_flag.en;
        Chassis_EnAll(chassis_flag.en ? 1 : 0);
    }

    /* 电池电压: 每拍消费回包(有就更新), 每 500ms 发一次读请求。
       必须在下面三个提前 return (遥控锁定/扫码/旋转) 之前, 否则 IDLE 等启动时一直不刷新。 */
    Chassis_VBusPoll();
    if (++vbus_tick >= 50)
    {
        vbus_tick = 0;
        Chassis_VBusRequest();
    }

    /* 主动轮询读位置: 每 10ms 问一个轮子, 4 拍轮完一圈 (odom_poll=1 时) */
    if (chassis_flag.odom_poll)
    {
        Chassis_Odom_PollTick();
    }

    /* 每 chassis_cfg.odom_update_cycles 个 10ms 周期更新一次里程计 (读4轮位置, 约2-4ms) */
    if (++odom_tick >= chassis_cfg.odom_update_cycles)
    {
        odom_tick = 0;
        Chassis_Odom_Update();
    }

    /* 读陀螺仪 (相对车头航向) */
    yaw = Chassis_GetHeading();
    chassis_val.yaw_cur = yaw;

    /* PID 初始化一次 + 每周期同步 Watch 增益 */
    if (!yaw_pid_inited)
    {
        PID_Init(&yaw_pid, chassis_cfg.yaw_kp, chassis_cfg.yaw_ki, chassis_cfg.yaw_kd,
                 chassis_cfg.yaw_max_out, chassis_cfg.yaw_max_i,
                 chassis_cfg.yaw_deadzone, chassis_cfg.yaw_iband, 0.5f, 1);  /* is_angle_loop=1 */
        chassis_val.yaw_target = yaw;
        yaw_pid_inited = 1;
    }
    yaw_pid.Kp = chassis_cfg.yaw_kp;
    yaw_pid.Ki = chassis_cfg.yaw_ki;
    yaw_pid.Kd = chassis_cfg.yaw_kd;
    yaw_pid.max_out = chassis_cfg.yaw_max_out;
    yaw_pid.max_i = chassis_cfg.yaw_max_i;
    yaw_pid.deadzone = chassis_cfg.yaw_deadzone;
    yaw_pid.i_band = chassis_cfg.yaw_iband;

    /* 整场重来 (长按启动/中止) 请求: 清一次偏航 PID 积分, 不带上一场的累积 */
    if (s_yaw_pid_clear_req)
    {
        s_yaw_pid_clear_req = 0;
        PID_Clear(&yaw_pid);
    }
    /* 锁定: sw[0] != 2 (含中间档1) 或失联 -> 停 4 轮 + 锁当前朝向 + 清积分
       定位中则转"暂停"(保留目标): 手动定位解锁后需再按 btn0/btn1, 全程导航解锁后自动继续 */
    if (chassis_flag.need_sw_unlock && (ELRS_RC.sw[0] != 2 || !ELRS_RC.online))
    {
        if (chassis_flag.pos_active == 1) chassis_flag.pos_active = 2;   /* 定位 -> 暂停 */
        chassis_val.yaw_target = yaw;
        PID_Clear(&yaw_pid);
        Chassis_SendWheels(0, 0, 0, 0);
        return;
    }

    /* 扫码直行 (mission): 沿当前车头锁航向直行, 覆盖导航/纠偏/遥控 */
    if (chassis_flag.qr_scan_active && !chassis_flag.rc_override)
    {
        if (!qr_armed) { qr_armed = 1; chassis_val.yaw_target = yaw; PID_Clear(&yaw_pid); }
        vx = chassis_cfg.qr_scan_speed;
        vy = 0.0f;
        omega = Chassis_YawOmega(&yaw_pid, chassis_val.yaw_target, yaw);
        Chassis_MixWheels(vx, vy, omega);
        return;
    }
    qr_armed = 0;

    /* 原地旋转 (mission): 只转 yaw, 到位退出 (world yaw, 与 Imu 同系)
       ★ 到位判据加"连续稳定确认": 进死区瞬间车头可能还带惯性角速度, 立即切平移会带着没转正的车头直行。
       连续 rot_settle_ticks 拍都落在死区内(惯性已被摩擦吃掉, 车头停稳)才判转完。 */
    if (chassis_flag.rotate_active && !chassis_flag.rc_override)
    {
        if (!rot_armed) { rot_armed = 1; rot_stable_tick = 0; PID_Clear(&yaw_pid); }
        if (fabsf(Math_NormalizeAngle(chassis_val.rotate_target - yaw)) <= chassis_cfg.yaw_deadzone)
        {
            if (++rot_stable_tick >= chassis_cfg.rot_settle_ticks)
            {
                chassis_flag.rotate_active = 0;
                rot_stable_tick = 0;
                Chassis_SendWheels(0, 0, 0, 0);
            }
            else
            {
                /* 已进死区但未稳够: 死区内 P 输出为 0(电机停), 停轮等惯性滑停 */
                Chassis_SendWheels(0, 0, 0, 0);
            }
        }
        else
        {
            rot_stable_tick = 0;
            omega = Chassis_YawOmega(&yaw_pid, chassis_val.rotate_target, yaw);
            Chassis_SendWheels(omega, omega, omega, omega);
        }
        return;
    }
    rot_armed = 0;

    /* 全程导航状态机 (优先于手动 btn0/btn1 定位); 遥控优先时让路 */
    if (!chassis_flag.rc_override) Nav_Update();

    /* 视觉底盘纠偏: 收到 0x04/0x05 帧 -> 更新偏了多少/要矫正多少/是否成功, 未到位则纠偏
       (底盘锁定 chassis_flag.lock=1 时忽略, 塔吊作业期间不再纠偏;
        塔吊还在摆探查姿态时 chassis_flag.trim_hold=1 也忽略, 帧直接丢掉, 等姿态到位再收新帧;
        chassis_flag.trim_block=1 时本站彻底封死 —— 电机6 已伸出去挡住摄像头, 纠偏只会越纠越错)
       0x0E 转盘纠偏多一道门控: 发送方(物料区转盘节点)是常开的, 行车途中也在发,
       不要求武装的话一帧进来就会把 pos_active 拉起来劫持导航(Nav_Update 优先级更低),
       所以只认 Chassis_TrimArm 之后、Chassis_TrimDisarm 之前的帧。 */
    if (chassis_flag.trim_enable && !chassis_flag.trim_hold && !chassis_flag.lock && !chassis_flag.trim_block && !chassis_flag.rc_override && !nav_test_active && !Chassis_TrimSkipStation(nav_at_point) &&
        (Serial_ChassisTrimReady || (chassis_flag.trim_armed && Serial_TurntableTrimReady)))
    {
        Serial_ChassisTrimReady   = 0;
        Serial_TurntableTrimReady = 0;
        Chassis_ApplyVisionTrim();
    }

    /* 按钮边沿: 手动定位 btn0 回原点(0,0) / btn1 沿当前车头往前; 导航中忽略避免抢目标 */
    if (!nav_active)
    {
        static bool last_btn0 = false, last_btn1 = false;

        if (ELRS_RC.btns[0] && !last_btn0)
        {
            chassis_val.pos_target_x_mm = 0.0f;
            chassis_val.pos_target_y_mm = 0.0f;
            chassis_flag.pos_active = 1;
            chassis_val.yaw_target = yaw;         /* 定位期间锁当前车头朝向 */
            PID_Clear(&yaw_pid);
        }
        if (ELRS_RC.btns[1] && !last_btn1)
        {
            /* 暂停中再按 -> 恢复原目标继续; 否则新目标: 沿当前车头往前 chassis_cfg.pos_fwd_mm */
            if (chassis_flag.pos_active != 2)
            {
                float th = chassis_cfg.odom_yaw_invert * yaw * (MATH_PI / 180.0f);
                chassis_val.pos_target_x_mm = chassis_val.odom_x_mm + chassis_cfg.pos_fwd_mm * cosf(th);
                chassis_val.pos_target_y_mm = chassis_val.odom_y_mm + chassis_cfg.pos_fwd_mm * sinf(th);
            }
            chassis_flag.pos_active = 1;
            chassis_val.yaw_target = yaw;
            PID_Clear(&yaw_pid);
        }
        last_btn0 = ELRS_RC.btns[0];
        last_btn1 = ELRS_RC.btns[1];
    }

    if (chassis_flag.pos_active == 1 && !chassis_flag.rc_override)
    {
        /* 点对点定位: 里程计闭环 + 速度规划, 航向 PID 锁死 (世界系误差 -> 车体系) */
        float ex = chassis_val.pos_target_x_mm - chassis_val.odom_x_mm;
        float ey = chassis_val.pos_target_y_mm - chassis_val.odom_y_mm;
        float th = chassis_cfg.odom_yaw_invert * yaw * (MATH_PI / 180.0f);
        float ef =  ex * cosf(th) + ey * sinf(th);   /* 前进方向误差 */
        float el = -ex * sinf(th) + ey * cosf(th);   /* 左移方向误差 */

        /* ---- 速度规划 (又快又准), 旋钮在 car_config.h "点对点定位速度规划" ----
           老代码是纯 P 控 v = kp×误差: 指数收敛, 离点越近越慢, 最后 200mm 要爬好几秒 ——
           停靠慢的根源就在这。现在按刹车曲线给速度:
             远处      -> 直接顶满 pos_max_speed 巡航 (不再被 P 提前压慢)
             刹车段    -> sqrt(2·a·d) 恒定减速度 (a=pos_decel, d=|ef|+|el| 与混控同口径)
             末端收口  -> 离点 pos_taper_mm 内线性收到死区边上为 0, 触点比老 P 控还软
           方向仍按误差比例分 (vx:vy = ef:el), 轨迹还是直线, 与老代码同一口径。
           pos_decel<=0 整段退回老 P 控 (现场兜底, Watch 改一下就行)。 */
        float d = fabsf(ef) + fabsf(el);            /* 剩余距离 (mm) */
        if (chassis_cfg.pos_decel > 0.0f && d > 0.0f)
        {
            float k_mm = 2.0f * MATH_PI * chassis_cfg.odom_wheel_radius_mm / 60.0f;  /* 轮速 RPM -> mm/s */
            float A    = chassis_cfg.pos_decel / (k_mm * k_mm);                     /* 减速度 (RPM²/mm) */
            float v    = sqrtf(2.0f * A * d);                                       /* 刹车曲线 */
            if (v > chassis_cfg.pos_max_speed) v = chassis_cfg.pos_max_speed;      /* 远处巡航 */

            if (chassis_cfg.pos_taper_mm > chassis_cfg.pos_deadzone_mm &&
                d < chassis_cfg.pos_taper_mm)
            {
                /* 末端线性收口: 从收口半径处的刹车速度线性收到死区边上 0。
                   收口起点取刹车曲线在该半径的值, 速度连续 (减速度略变陡, 不是跳变)。 */
                float v_t = sqrtf(2.0f * A * chassis_cfg.pos_taper_mm);
                v = v_t * (d - chassis_cfg.pos_deadzone_mm) /
                         (chassis_cfg.pos_taper_mm - chassis_cfg.pos_deadzone_mm);
                if (v < 0.0f) v = 0.0f;             /* 死区边上 -> 0, 不往回顶 */
            }

            vx = v * ef / d;
            vy = v * el / d;
        }
        else
        {
            /* 老 P 控 (pos_decel=0 兜底)。限幅必须等比 —— 逐轴各自卡 pos_max_speed 会拧弯
               行驶方向: 两个轴按同一 kp 算, 速度矢量本来指向目标(直线); 一旦某个轴被单独
               卡住, 方向就偏, 于是"误差小的那个轴先走完", 轨迹变成 L 形。
               例: 收车段 (200,-1915) -> (-35,0), ef=235mm / el=1915mm,
                   逐轴限幅时 vx=47、vy=383 被卡 -> x 先到, 然后纯横移 1.9 米。
               按总轮速 |vx|+|vy| 等比压缩: 方向不变, 两轴同时按比例到位 = 直线。 */
            vx = chassis_cfg.pos_kp * ef;
            vy = chassis_cfg.pos_kp * el;
            if (chassis_cfg.pos_max_speed > 0.0f)
            {
                float sp = fabsf(vx) + fabsf(vy);
                if (sp > chassis_cfg.pos_max_speed)
                {
                    float k = chassis_cfg.pos_max_speed / sp;
                    vx *= k;
                    vy *= k;
                }
            }
        }

        /* 末端蠕动下限: 收口在死区边上(v=0)归零, 但 EMM 低速段 + 麦轮静摩擦会让车停在
           死区外 10~13mm —— 那里命令只有 0.6~1.2RPM, (uint16_t) 截断后就是 0, 轮子根本不发;
           而到点判据要求进死区 -> 永远差那几毫米, pos_active 卡在 1, mission 里所有
           "等 !pos_active" 的状态(启动区 X 到位/导航各点)全卡死。
           只要还在死区外就保证至少 pos_creep 的轮速, 方向仍按 (ef,el) 等比 -> 轨迹不变;
           进来就被下面的到点判据接住。0 = 关闭 (退回旧行为)。 */
        if (chassis_cfg.pos_creep > 0.0f && d > chassis_cfg.pos_deadzone_mm)
        {
            float sp = fabsf(vx) + fabsf(vy);
            if (sp < chassis_cfg.pos_creep)
            {
                vx = chassis_cfg.pos_creep * ef / d;
                vy = chassis_cfg.pos_creep * el / d;
            }
        }

        /* 速度下限 (mission 启动区"不停车通过"): 纯 P 控在离点前几百 mm 就开始减速,
           到接管半径(50mm)时只有 10RPM, 看着就是"快到点先停下来再窜出去"。
           给个下限: 没进到点死区之前保持不低于 pos_min_speed, 等比放大 = 方向不变(还是直线过去),
           于是这一段是匀速滚过去, 到点上被 mission 提前接管时速度完全连续。
           0 = 关闭 -> 老行为 (只影响 pos_min_speed>0 的那一段, 其它点对点定位不变)。
           注意顺序: 先按 max 压、再抬到 min, 所以 min 必须 <= max 才有意义 (两者都取 100)。 */
        if (chassis_cfg.pos_min_speed > 0.0f)
        {
            float sp = fabsf(vx) + fabsf(vy);
            if (sp > 0.0f && sp < chassis_cfg.pos_min_speed)
            {
                float k = chassis_cfg.pos_min_speed / sp;
                vx *= k;
                vy *= k;
            }
        }

        /* 到点: 总误差 d=|ef|+|el| 进死区 -> 停 + 退出定位.
           必须和速度曲线的归零点同一个口径 (收口段在 d=pos_deadzone_mm 处收到 0),
           原来按"两个轴各自 < 死区"判, 会出现 v 已经=0 但 d>10 的死缝 (见上面蠕动下限). */
        if (d <= chassis_cfg.pos_deadzone_mm)
        {
            chassis_flag.pos_active = 0;
            chassis_val.dock_corr_x = Serial_RxData.ring_corr_x;   /* 自动停靠: 快照视觉 corr, 给塔吊卸货补进落点 */
            chassis_val.dock_corr_y = Serial_RxData.ring_corr_y;
            vx = 0.0f;
            vy = 0.0f;
        }

        omega = Chassis_YawOmega(&yaw_pid, chassis_val.yaw_target, yaw);
    }
    else if (chassis_flag.lock)
    {
        /* 塔吊作业锁底盘: 忽略遥控, 停 4 轮保持原位 (防作业时误动) */
        vx = 0.0f;
        vy = 0.0f;
        omega = 0.0f;
        chassis_val.yaw_target = yaw;
        PID_Clear(&yaw_pid);
    }
    else
    {
        /* 平移速度: 摇杆前/左先当"世界系"速度, 需要时再旋回车体系 */
        {
            float vx_w =  chassis_cfg.rc_ry_invert * ELRS_RC.Ry * chassis_cfg.rc_speed;  /* 世界前进 +X */
            float vy_w = -chassis_cfg.rc_rx_invert * ELRS_RC.Rx * chassis_cfg.rc_speed;  /* 世界左移 +Y */
            if (chassis_flag.rc_field_orient)
            {
                /* 世界系 -> 车体系 (与点对点定位 ef/el 同款 R(-th)):
                   推杆向前 = 朝世界 +X 走, 不管车头朝哪; 方向反了把 chassis_cfg.odom_yaw_invert 取反 */
                float th = chassis_cfg.odom_yaw_invert * yaw * (MATH_PI / 180.0f);
                vx =  vx_w * cosf(th) + vy_w * sinf(th);   /* 车体前进 */
                vy = -vx_w * sinf(th) + vy_w * cosf(th);   /* 车体左移 */
            }
            else
            {
                vx = vx_w;   /* 车体系直控 (原行为) */
                vy = vy_w;
            }
        }

        /* Lx 旋转: 摇杆直接比例控角速度 (跟手); 松杆后先滑行停稳, 再 PID 锁航向 */
        {
            float lx = chassis_cfg.rc_lx_invert * ELRS_RC.Lx;
            static uint16_t settle_tick = 0;
            uint16_t settle_max = (uint16_t)(chassis_cfg.rc_settle_ms / 10.0f);

            if (fabsf(lx) > 0.05f)
            {
                /* 摇杆在动: 直接速率控制 (跟手) */
                settle_tick = 0;
                chassis_val.yaw_target = yaw;
                PID_Clear(&yaw_pid);
                omega = lx * chassis_cfg.rc_rotate_rate;
            }
            else if (settle_tick < settle_max)
            {
                /* 摇杆刚回中: 目标跟随车头滑行停稳, 不加修正 -> 不"回弹" */
                settle_tick++;
                chassis_val.yaw_target = yaw;
                PID_Clear(&yaw_pid);
                omega = 0.0f;
            }
            else
            {
                /* 停稳后: 陀螺仪 PID 锁住当前航向 (P死区) + Gz 角速度阻尼 (消左右晃)
                   阻尼项 -kd_rate*yaw_rate 反了(越修越晃/发散)就把 chassis_cfg.yaw_kd_rate 取负 */
                omega = Chassis_YawOmega(&yaw_pid, chassis_val.yaw_target, yaw);
            }
        }
    }
    chassis_val.yaw_out = omega;

    Chassis_MixWheels(vx, vy, omega);   /* 混控 + 旋转优先限幅 + 四轮同步发 */

    /* (电池电压已挪到任务开头: 每拍消费回包 + 每 500ms 发读请求) */

    /* VOFA 调试 (临时): vofa_enable=1 时把串口屏 USART6 切给 VOFA 波形
       通道顺序: err_yaw(误差) / yaw_out(修正输出RPM) / yaw(实际航向) / aim_yaw(目标航向) / p_term(P项RPM) / rate_filt(滤波角速度) */
    Vofa_Enable();
    if (vofa_enable)
    {
        Chassis_Feedback1.aim_yaw = chassis_val.yaw_target;
        Chassis_Feedback1.yaw     = chassis_val.yaw_cur;
        Chassis_Feedback1.err_yaw = Math_NormalizeAngle(chassis_val.yaw_target - chassis_val.yaw_cur);
        Chassis_Feedback1.yaw_out = chassis_val.yaw_out;
        Chassis_Feedback1.p_term    = chassis_val.yaw_p_term;
        Chassis_Feedback1.rate_filt = chassis_val.yaw_rate_f;
        USART6_Send_VOFA_JustFloat1();
    }
}

