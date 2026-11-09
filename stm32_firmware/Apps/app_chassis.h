#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include "app_nav.h"    /* NavPointId (Chassis_TrimSkipStation 参数用) */

/**
  * @brief  麦轮底盘: 遥控直控 + 陀螺仪航向保持 + 点对点定位 + 里程计 + 电池电压
  *         电机地址: 1=BL(后左) 2=BR(后右) 3=FL(前左) 4=FR(前右)
  */
void App_Chassis_Init(void);        /* 上电初始化 (懒初始化, 预留) */
void App_Chassis_Task(void);        /* 10ms: 遥控直控底盘 (含定位/里程计/电压) */
void App_Chassis_Test_Task(void);   /* 10ms: 底盘单项测试 (Watch 改 chassis_flag.test_mode) */
void Chassis_ApplyVisionTrim(void); /* 视觉底盘微调: 0x04/0x05 corr -> 底盘目标位姿 */

/* ================= 底盘打包变量 (Keil Watch 只看 chassis_flag/chassis_cfg/chassis_val) ================= */

/* 标志位: 位域打包, 全部叠进 1 个 uint32 (Keil Watch 只看 chassis_flag) */
typedef struct {
    uint32_t test_mode       : 3;   /* 0停 1前进 2后退 3左移 4右移 5顺时针 6逆时针 */
    uint32_t yaw_enable      : 1;   /* 1=平移时用陀螺仪修正车头 */
    uint32_t min_omega_en    : 1;   /* 1=启用最小修正量 (默认关) */
    uint32_t rc_field_orient : 1;   /* 1=世界系遥控 (推杆向前=世界+X) */
    uint32_t need_sw_unlock  : 1;   /* 1=需 sw[0]==2 且遥控在线才动; 0=旁路(纯视觉/mission) */
    uint32_t rot_prio_en     : 1;   /* 1=旋转优先限幅 (车头矫正优先, 平移让位) */
    uint32_t odom_use_autorep: 1;   /* 1=用 EMM 定时上报读位置 0=退回 4 轮阻塞读 */
    uint32_t pos_active      : 2;   /* 0空闲 1导航中 2暂停 */
    uint32_t trim_enable     : 1;   /* 1=启用视觉纠偏 (Watch 写 0 关掉) */
    uint32_t trim_within     : 1;   /* 是否矫正成功: 视觉 within (1=到位, 只读) */
    uint32_t rotate_active   : 1;   /* 1=原地旋转中 (只读) */
    uint32_t qr_scan_active  : 1;   /* 1=扫码直行(沿当前车头锁航向), mission 清 0 停 */
    uint32_t lock            : 1;   /* 1=锁定底盘(忽略视觉纠偏), 塔吊作业时置 1 */
    uint32_t rc_override     : 1;   /* 1=遥控优先: 导航/视觉微调/定位/原地旋转/扫码全让路, 底盘归遥控直控 */
    uint32_t en              : 1;   /* 1=四轮使能(锁轴); 上电默认 0(松轴没力), 长按 key 置 1 */
    uint32_t trim_skip       : 3;   /* 工作点纠偏关闭位图: bit0物料 bit1粗码 bit2暂存 */
    uint32_t trim_hold       : 1;   /* 1=暂不应用视觉纠偏 (塔吊还在摆探查姿态, 等姿态到位再放开) */
    uint32_t trim_armed      : 1;   /* 1=本次进站已武装: 只认武装之后收到的 within, 不吃上一站残留 */
    uint32_t trim_fresh      : 1;   /* 只读: 武装后已收到过 0x04/0x05 帧 */
    uint32_t trim_conv       : 1;   /* 只读: 底盘自判到位 (要纠的量已落进底盘自己的死区, 再纠车也不会动,
                                       连续 CHASSIS_TRIM_CONV_FRAMES 帧成立才置1);
                                       视觉 within 是它的容差(叠加位 2mm), 比底盘死区(10mm)紧, 光等它会永远等不到 */
    uint32_t trim_block      : 1;   /* 1=本站纠偏彻底封死 (电机6 已伸出去会挡摄像头, 再来帧一律不收);
                                       由 Chassis_TrimBlock() 置1, 下一次 Chassis_TrimArm() (进新站) 清0 */
    uint32_t autorep_rearm_en: 1;   /* 1=每秒给四个电机全量重发上膛命令; 0=默认关。
                                       实测: 全量重发会把四轮上报相位对齐 -> 每 20ms 一起撞仲裁,
                                       ID 小的永远赢, 3/4 号停报 (stale 13% vs 0.2%)。见 docs/decisions/010。
                                       日常靠 Chassis_Odom_Update 里"谁 100ms 没新帧就单独补一条"兜底 */
    uint32_t odom_poll       : 1;   /* 1=里程计改"主动轮询读位置": 每 10ms 轮流问一个轮子(0x36),
                                       不用电机自己的定时上报 (它经实测会自己停), 见 Chassis_Odom_PollTick */
} ChassisFlag_t;

/* 整定/标定参数 (现场 Watch 改) */
typedef struct {
    float    test_speed;             /* 轮速 RPM */
    float    yaw_kp, yaw_ki, yaw_kd, yaw_kd_rate;
    float    yaw_damp_max, yaw_rate_dz;
    float    min_omega, yaw_max_out, yaw_max_i, yaw_deadzone, yaw_iband;
    int8_t   yaw_invert;
    float    yaw_rate_lpf_alpha;
    uint16_t rot_settle_ticks;       /* 原地旋转到位稳定拍数 (10ms/拍): 进死区后连续这么拍都保持才判转完 */
    float    rc_speed, rc_rotate_rate, rc_settle_ms;
    int8_t   rc_rx_invert, rc_ry_invert, rc_lx_invert;
    float    wheel_max;
    float    odom_wheel_radius_mm, odom_gear_ratio;
    int8_t   odom_yaw_invert;
    uint16_t odom_update_cycles;
    float    pos_kp, pos_max_speed, pos_deadzone_mm, pos_fwd_mm;
    float    pos_decel, pos_taper_mm;  /* 定位速度规划 (car_config.h 的 CHASSIS_POS_*):
                                          pos_decel = 刹车减速度 mm/s² (0=退回老 P 控, 用上面的 pos_kp);
                                          pos_taper_mm = 末端线性收口半径 mm (须 > pos_deadzone_mm) */
    float    pos_min_speed;          /* 定位速度下限 (轮速 RPM): 0=关(纯 P 控, 到点前自然减速);
                                        >0 时误差还大就按等比放大保到至少这个速度, 方向不变。
                                        给"不停车通过"用 (mission 开/关); 2026-11-09 起启动区改成
                                        到点停车, 目前没有流程在用, 机制保留 */
    float    pos_creep;              /* 末端蠕动下限 (轮速 RPM): 还在死区外就保证至少这个速度,
                                        防收口段命令被截断成 0/1RPM 后车停在死区外不动。
                                        0=关; car_config.h 的 CHASSIS_POS_CREEP_RPM */
    float    trim_gain_x, trim_gain_y, trim_gain_yaw;
    float    qr_scan_speed;
    uint8_t  acc;                    /* 速度命令加速度档位 0~255 */
} ChassisCfg_t;

/* 运行状态/调试值 (只读为主, Watch 看) */
typedef struct {
    float    yaw_target, yaw_cur, yaw_out;
    float    yaw_p_term, yaw_damp, yaw_rate_f;
    float    vbus;                   /* 电池电压 (V) */
    float    odom_x_mm, odom_y_mm, odom_yaw_deg, odom_vx_mmps, odom_vy_mmps;
    float    pos_target_x_mm, pos_target_y_mm;
    float    trim_err_x, trim_err_y, trim_err_yaw;
    float    trim_corr_x, trim_corr_y, trim_corr_yaw;
    float    dock_corr_x, dock_corr_y;   /* 自动停靠时快照的视觉 corr, 供塔吊卸货落点补偿 */
    float    rotate_target;          /* 旋转目标 yaw (度, 世界系) */
    float    yaw_ref;                /* 车头基准: 长按 key 记录的 Imu_GetYaw() */
} ChassisVal_t;

extern ChassisFlag_t chassis_flag;
extern ChassisCfg_t  chassis_cfg;
extern ChassisVal_t  chassis_val;

/* ================= 全自动流程 (mission) 用底盘接口 ================= */
void Chassis_RotateTo(float yaw_deg);   /* 原地旋转到世界系 yaw(度, 与 Imu_GetYaw 同系), 到位后 chassis_flag.rotate_active=0 */

float   Chassis_GetHeading(void);        /* 相对车头航向 = Normalize(Imu_GetYaw() - chassis_val.yaw_ref) */
void    Chassis_ResetHeading(void);      /* 记录当前 yaw 为车头基准 + 里程计清零 */
uint8_t Chassis_TrimSkipStation(NavPointId id); /* 该工作点是否跳过视觉纠偏(bit 位图) */

/* 进站视觉微调武装 (mission MS_TRIM / kin_flag.cmd=11 共用):
   Arm -> 清 within/fresh/block, 挂起纠偏(姿态期间不收帧)
   Release -> 姿态到位, 放开纠偏, 丢掉积压旧帧
   Done -> 1 = 放开之后真的收到 within=1 的新帧
   Block -> 微调已到位、电机6 要伸出去作业: 本站彻底不再收任何纠偏帧, 直到下一次 Arm
   Disarm -> 离站/停止, 退回原来的"收到就自动纠偏" */
void    Chassis_TrimArm(void);
void    Chassis_TrimRelease(void);
void    Chassis_TrimDisarm(void);
void    Chassis_TrimBlock(void);
uint8_t Chassis_TrimDone(void);

/* 立即失能四轮(松轴) 并同步 chassis_flag.en=0: 运行中出问题长按一键键中止时用,
   让操作员能直接把车搬回启动区; 下次 IDLE 长按启动会 chassis_flag.en=1 重新锁轴。 */
void    Chassis_DisableMotors(void);

#endif
