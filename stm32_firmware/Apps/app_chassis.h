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

/* 标志位: 位域打包, 15 个标志叠进 1 个 uint32 */
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
    uint32_t en              : 1;   /* 1=四轮使能(锁轴); 上电默认 0(松轴没力), 长按 key 置 1 */
    uint32_t trim_skip       : 3;   /* 工作点纠偏关闭位图: bit0物料 bit1粗码 bit2暂存 */
} ChassisFlag_t;

/* 整定/标定参数 (现场 Watch 改) */
typedef struct {
    float    test_speed;             /* 轮速 RPM */
    float    yaw_kp, yaw_ki, yaw_kd, yaw_kd_rate;
    float    yaw_damp_max, yaw_rate_dz;
    float    min_omega, yaw_max_out, yaw_max_i, yaw_deadzone, yaw_iband;
    int8_t   yaw_invert;
    float    yaw_rate_lpf_alpha;
    float    rc_speed, rc_rotate_rate, rc_settle_ms;
    int8_t   rc_rx_invert, rc_ry_invert, rc_lx_invert;
    float    wheel_max;
    float    odom_wheel_radius_mm, odom_gear_ratio;
    int8_t   odom_yaw_invert;
    uint16_t odom_update_cycles;
    float    pos_kp, pos_max_speed, pos_deadzone_mm, pos_fwd_mm;
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

#endif
