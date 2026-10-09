#ifndef __CAR_CONFIG_H
#define __CAR_CONFIG_H

/**
  ******************************************************************************
  * @file    car_config.h
  * @brief   整车参数集中配置 (电机地址 / 遥控映射 / PID / 舵机角度)
  *          现场调车只改这个文件, 不要散落到各模块。
  ******************************************************************************
  */

/* ================= 步进电机 CAN 地址 ================= */
#define MOTOR_FL        1    /* 前左轮 */
#define MOTOR_FR        2    /* 前右轮 */
#define MOTOR_RL        3    /* 后左轮 */
#define MOTOR_RR        4    /* 后右轮 */
#define MOTOR_HOIST     5    /* 塔吊升降 */
#define MOTOR_EXTEND    6    /* 塔吊伸出 */
#define MOTOR_COUNT     6

/* ================= 底盘 (麦轮) ================= */
#define CHASSIS_MAX_SPEED     120.0f  /* 平移最大轮速 (RPM) */
#define CHASSIS_ACC           300     /* 加减速 (RPM/s) */
#define CHASSIS_ROTATE_RATE   40.0f   /* 偏航 PID 输出上限 (RPM) */
#define CHASSIS_LX_SENS       3.0f    /* Lx 航向积分灵敏度 (度/周期) */

/* 单轮方向修正: 实车若某轮反转, 把对应宏改成 -1 */
#define WHEEL_FL_DIR          1
#define WHEEL_FR_DIR          1
#define WHEEL_RL_DIR          1
#define WHEEL_RR_DIR          1

/* 原地旋转方向修正: 实车若旋转方向反了, 改成 -1 */
#define CHASSIS_ROTATE_INVERT 1

/* 偏航 PID 参数 (现场整定) */
#define CHASSIS_YAW_KP        5.0f
#define CHASSIS_YAW_KI        0.05f
#define CHASSIS_YAW_KD        0.0f
#define CHASSIS_YAW_MAX_I     30.0f
#define CHASSIS_YAW_DEADZONE  0.5f   /* 度 */
#define CHASSIS_YAW_IBAND     5.0f   /* 度 */

/* ================= 里程计 (麦轮 + 陀螺仪 yaw) ================= */
#define ODOM_WHEEL_RADIUS_MM  48.5f  /* 麦轮半径 (mm): 外径97/2, 5mm联轴器直连 */
#define ODOM_GEAR_RATIO       1.0f   /* 电机:轮 减速比, 直驱=1 */
#define ODOM_YAW_INVERT       1      /* 陀螺仪 yaw 方向反了改 -1 */
#define ODOM_UPDATE_CYCLES    5      /* 每 N 个 10ms 周期更新一次 (5=50ms) */

/* ================= 塔吊 (速度模式) ================= */
#define HOIST_MAX_SPEED       150.0f  /* 升降最大速度 (RPM) */
#define HOIST_ACC             200     /* RPM/s */
#define EXTEND_MAX_SPEED      100.0f  /* 伸出最大速度 (RPM) */
#define EXTEND_ACC            200     /* RPM/s */

/* ================= 舵机 ================= */
/* 各舵机角度行程 (度), CCR = angle/range*2000+500 */
#define SERVO_GRIPPER_RANGE   270.0f
#define SERVO_PLATFORM_RANGE  270.0f
#define SERVO_GIMBAL_RANGE    360.0f

/* 夹爪 */
#define GRIPPER_CLOSE_ANGLE   10.0f
#define GRIPPER_OPEN_ANGLE    80.0f

/* 物料平台 (3 档, 现场标定) */
#define PLATFORM_ANGLE_A      51.0f
#define PLATFORM_ANGLE_B      168.0f
#define PLATFORM_ANGLE_C      270.0f

/* 云台 */
#define GIMBAL_MIN_ANGLE      0.0f
#define GIMBAL_MAX_ANGLE      360.0f
#define GIMBAL_MAX_STEP       1.0f    /* 云台每 10ms 周期最大转角(度), 越小越慢 */

#endif /* __CAR_CONFIG_H */
