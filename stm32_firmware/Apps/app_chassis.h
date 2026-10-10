#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

/**
  * @brief  麦轮底盘: 遥控直控 + 陀螺仪航向保持 + 点对点定位 + 里程计 + 电池电压
  *         电机地址: 1=BL(后左) 2=BR(后右) 3=FL(前左) 4=FR(前右)
  */
void App_Chassis_Init(void);        /* 上电初始化 (懒初始化, 预留) */
void App_Chassis_Task(void);        /* 10ms: 遥控直控底盘 (含定位/里程计/电压) */
void App_Chassis_Test_Task(void);   /* 10ms: 底盘单项测试 (Watch 改 chassis_test_mode) */
void Chassis_ApplyVisionTrim(void); /* 视觉底盘微调: 0x04/0x05 corr -> 底盘目标位姿 */

/* ================= 全自动流程 (mission) 用底盘接口 ================= */
void Chassis_RotateTo(float yaw_deg);   /* 原地旋转到世界系 yaw(度, 与 Imu_GetYaw 同系), 到位后 chassis_rotate_active=0 */
extern uint8_t chassis_rotate_active;   /* 只读: 1=原地旋转中 */
extern float   chassis_rotate_target;   /* 只读: 旋转目标 yaw (度) */
extern uint8_t chassis_qr_scan_active;  /* 1=扫码直行(沿当前车头锁航向), mission 清 0 停 */
extern float   chassis_qr_scan_speed;   /* 扫码直行轮速 (RPM) */
extern uint8_t chassis_lock;            /* 1=锁定底盘(忽略 0x04/0x05 视觉纠偏), 塔吊作业时置 1 */

/* 视觉底盘纠偏 (0x04/0x05): 偏了多少(err) / 要矫正多少(corr) / 是否矫正成功(within) */
extern uint8_t chassis_trim_enable;    /* 1=启用纠偏 (Watch 写 0 关掉) */
extern float   chassis_trim_gain_x;    /* corr_x 增益, 可负/调大小 */
extern float   chassis_trim_gain_y;    /* corr_y 增益 */
extern float   chassis_trim_gain_yaw;  /* corr_yaw 增益 */
extern float   chassis_trim_err_x;     /* 偏了多少: 视觉 err_x (mm, 只读) */
extern float   chassis_trim_err_y;     /* 偏了多少: 视觉 err_y (mm, 只读) */
extern float   chassis_trim_err_yaw;   /* 偏了多少: 视觉 dyaw (deg, 只读) */
extern float   chassis_trim_corr_x;    /* 要矫正多少: corr_x*增益 (mm, 只读) */
extern float   chassis_trim_corr_y;    /* 要矫正多少: corr_y*增益 (mm, 只读) */
extern float   chassis_trim_corr_yaw;  /* 要矫正多少: corr_yaw*增益 (deg, 只读) */
extern uint8_t chassis_trim_within;    /* 是否矫正成功: 视觉 within (1=到位, 只读) */
extern float   chassis_dock_corr_x;    /* 自动停靠时快照的视觉 corr_x (mm), 供塔吊卸货落点补偿 */
extern float   chassis_dock_corr_y;    /* 自动停靠时快照的视觉 corr_y (mm), 供塔吊卸货落点补偿 */

#endif
