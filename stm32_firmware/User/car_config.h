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

/* 麦轮辊子布局 (O/X): 只影响左右横移(vy)方向, 前后/旋转不受影响
   +1 = O 型, -1 = X 型; 换布局只改这一个宏 (逆解 + 正运动学都引用它) */
#define MECANUM_VY_SIGN       (-1)   /* 当前: X 型 */

/* 偏航 PID 参数 (现场整定; 唯一真值, app_chassis.c 变量用这些宏初始化)
   参考 yyb pid_choose=1 (Kp=3.3 / maxOut=30): 原 Kp=0.6 太弱, 旋转时 P 输出压不住摩擦,
   车卡在死区外几度 -> chassis_rotate_active 不清零 -> mission 停在 MS_QR_ROTATE_MAT;
   平移车头也跟不上 -> 晃晃。Kp 负责到位, KD_RATE 阻尼负责消左右晃。 */
#define CHASSIS_YAW_KP        3.5f
#define CHASSIS_YAW_KI        0.02f    /* yyb pid_choose=1 为 0; 稳态还跟不住再试 0.03~0.05 */
#define CHASSIS_YAW_KD        0.0f
#define CHASSIS_YAW_KD_RATE   0.0f    /* 0 = 四点实测定案(阻尼净有害: 压低环增益+LPF相位滞后+绕过死区); 原理由见 docs/decisions/003 */
#define CHASSIS_YAW_DAMP_MAX   8.0f   /* 阻尼项输出上限 (RPM): 防被打转时越过 max_out */
#define CHASSIS_YAW_RATE_DZ    3.0f   /* 阻尼速率死区 (°/s): 陀螺噪声不进电机 */
#define CHASSIS_MIN_OMEGA_EN   0      /* 1=启用最小修正量 (现场实测后再开, 默认关) */
#define CHASSIS_MIN_OMEGA      2.0f   /* 最小修正量 (RPM), 必须 >= EMM 速度量化台阶 */
#define CHASSIS_YAW_MAX_OUT   45.0f   /* 修正量上限 (RPM; yyb 同款, 原 25) */
#define CHASSIS_YAW_MAX_I     30.0f
#define CHASSIS_YAW_DEADZONE  0.8f    /* 度 (旋转完成判据也用它, 先保持别收太紧) */
#define CHASSIS_ROT_SETTLE_TICKS 10   /* 原地旋转到位稳定拍数 (10ms/拍): 进死区后连续这么拍都保持才判转完, 过滤"擦边判到位" */
#define CHASSIS_YAW_IBAND     5.0f    /* 度 */
#define CHASSIS_YAW_INVERT    (-1)    /* 修正方向反了改 1 */

/* 旋转优先限幅: 车头矫正(omega)优先, 平移(vx,vy)让位。
   每轮命令 = 平移分量(最大|vx|+|vy|) + 旋转分量(|omega|)。当平移+旋转之和
   超过每轮预算 CHASSIS_WHEEL_MAX 时, 等比压缩平移、保旋转全额 —— 高速直行
   时车头矫正始终有余量不被平移吃掉(跑不直就靠它兜)。0=关(退回裸混控)。 */
#define CHASSIS_ROT_PRIO_EN    1
#define CHASSIS_WHEEL_MAX      120.0f   /* 每轮速度预算 (RPM): 平移+旋转合计上限 */

/* ================= 里程计 (麦轮 + 陀螺仪 yaw) ================= */
#define ODOM_WHEEL_RADIUS_MM  48.5f  /* 麦轮半径 (mm): 外径97/2, 5mm联轴器直连 */
#define ODOM_GEAR_RATIO       1.0f   /* 电机:轮 减速比, 直驱=1 */
#define ODOM_YAW_INVERT       1      /* 陀螺仪 yaw 方向反了改 -1 */
#define ODOM_UPDATE_CYCLES    5      /* 每 N 个 10ms 周期更新一次 (5=50ms) */

/* 里程计位置读取方式: 1=用 EMM 定时主动上报快照替代 4 轮串行阻塞读, 0=退回阻塞读。
   现场验证 EMM 支持 0x11/0x18 后保持 1; 不支持就在 Keil Watch 把 chassis_odom_use_autorep 写 0 回退。 */
#define CHASSIS_AUTOREP_EN     1
#define CHASSIS_AUTOREP_MS     20     /* 上报间隔 (ms) */

/* ================= 塔吊 (速度模式) ================= */
#define HOIST_MAX_SPEED       150.0f  /* 升降最大速度 (RPM) */
#define HOIST_ACC             200     /* RPM/s */
#define EXTEND_MAX_SPEED      100.0f  /* 伸出最大速度 (RPM) */
#define EXTEND_ACC            200     /* RPM/s */

/* 塔吊升降(电机5) 各工位高度 (单位: 电机度, 现场标定) */
#define HOIST_HEIGHT_PLATFORM 540.0f  /* 放置/抓取 物料平台 */
#define HOIST_HEIGHT_UNLOAD_GRAB 530.0f /* 卸货抓取 物料平台 (比放置600低50, 更贴物料) */
#define HOIST_HEIGHT_ROTATE   460.0f  /* 抓回 回转平台 = 物料区抓取高 (视觉装货 + cmd=10 固定位装货 唯一真值) */
#define HOIST_HEIGHT_PALLET   370.0f  /* 暂存码垛 落点 */
#define HOIST_HEIGHT_DROP_GROUND  70.0f  /* 卸货放置 平地落点高度 */
#define HOIST_HEIGHT_GRAB_GROUND  60.0f  /* 回装抓取 平地高度 */
#define HOIST_HEIGHT_GROUND   20.0f  /* 电机5 默认位(低) */
#define HOIST_CLEARANCE_HEIGHT 635.0f /* 过渡/清障高度 (抓放后升到这再转云台/伸缩) */

/* ================= 姿态 (云台 / 电机5 / 电机6) ================= */
#define POSTURE_MOTION_GIMBAL  310.0f  /* 运动姿态: 行驶/换工作点 */
#define POSTURE_MOTION_HOIST   600.0f
#define POSTURE_MOTION_EXTEND  500.0f
#define POSTURE_PROBE_GIMBAL   134.0f  /* 探查姿态: 到工作点给视觉看 + 抓放间停留 */
#define POSTURE_PROBE_HOIST    600.0f
#define POSTURE_PROBE_EXTEND   0.0f
/* 物料区专属探查姿态: 只在物料区用, 粗加工/暂存仍用 POSTURE_PROBE_*
   电机5 直接站在抓取高(不再单独下降), 夹爪张开等指令 -> 收到指令直接闭合 -> 升640去放置 */
#define POSTURE_MATERIAL_GIMBAL 135.0f  /* 云台: 正前方 (θ=0) */
#define POSTURE_MATERIAL_HOIST  HOIST_HEIGHT_ROTATE  /* 电机5: = 抓取高 460 (探查即抓取位, 无下降步) */
#define POSTURE_MATERIAL_EXTEND 200.0f  /* 电机6: 预先伸出, 靠近物料 */

/* ================= 全自动流程 (mission) ================= */
/* 进站等底盘视觉微调 within 的上限 (10ms/拍, 500=5s)。到探查姿态后放开纠偏,
   等一次新的 within=1; 视觉在该工作区不发 0x04/0x05 时就等这么长时间后放行, 不卡死流程。 */
#define MISSION_TRIM_WAIT_10MS  500

/* 物料区连抓 (kin_flag.cmd=12): 放完一件回探查姿态后继续等下一个 0x09, 一共连抓这么多个。
   只给原地/手动测试用 —— mission 的物品节拍是"0x08 给一件 -> 抓 -> 0x0F 报一件", 不要用 cmd=12。 */
#define MATERIAL_CHAIN_GRABS    3

/* ================= 舵机 ================= */
/* 各舵机角度行程 (度), CCR = angle/range*2000+500 */
#define SERVO_GRIPPER_RANGE   270.0f
#define SERVO_PLATFORM_RANGE  270.0f
#define SERVO_GIMBAL_RANGE    360.0f

/* 上电门控: Servo_Init 写停放位初值后, 再挡 N 个 10ms 拍才放行任何舵机动作 (避开舵机上电自检) */
#define SERVO_ARM_TICKS       50

/* 夹爪 */
#define GRIPPER_CLOSE_ANGLE   105.0f
#define GRIPPER_OPEN_ANGLE    200.0f

/* 物料平台 (3 档, 现场标定) */
#define PLATFORM_ANGLE_A      43.0f
#define PLATFORM_ANGLE_B      160.0f
#define PLATFORM_ANGLE_C      270.0f

/* 云台 */
#define GIMBAL_MIN_ANGLE      0.0f
#define GIMBAL_MAX_ANGLE      360.0f
#define GIMBAL_MAX_STEP       1.0f    /* 云台每 10ms 周期最大转角(度), 越小越慢 */
#define GIMBAL_PARK_ANGLE     135.0f  /* 上电停放位 (θ=0 时舵机值, 与 kin_gimbal_theta_offset 一致) */

/* ================= 测试任务默认值 (Test_Task 里云台/平台/夹爪调试参数) =================
 * 现场先在 Keil Watch 里改对应变量, 调满意后再回填这里的宏, 保持"唯一真值"在 car_config.h。
 */
#define GIMBAL_TEST_ANGLE      GIMBAL_PARK_ANGLE   /* 云台调试目标角 (0~360) */
#define GIMBAL_TEST_STEP       GIMBAL_MAX_STEP     /* 云台调试斜坡 (度/10ms) */
#define PLATFORM_TEST_ANGLE    PLATFORM_ANGLE_A    /* 平台调试目标角 (0~270) */
#define PLATFORM_TEST_STEP     1.0f                /* 平台调试斜坡 (度/10ms) */
#define GRIPPER_TEST_ANGLE     GRIPPER_OPEN_ANGLE  /* 夹爪调试目标角 (0~270) */

#endif /* __CAR_CONFIG_H */
