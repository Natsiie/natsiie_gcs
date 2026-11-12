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

/* ================= 底盘 (麦轮) 速度总表 ================= */
/* ★ 现场调"车跑多快"只改这一段。app_chassis.c 的 chassis_cfg 初值引用这些宏, 改完编译就生效;
   运行中也能在 Keil Watch 实时改 chassis_cfg.*, 调满意再回填到这里 (保持唯一真值)。
   平移量都是"轮速 RPM"(直驱): 轮径 48.5mm, 1 RPM ≈ 5.1 mm/s (里程计口径), 70 RPM ≈ 0.36 m/s。
   加速度 CHASSIS_ACC 是 EMM 固件的档位 0~255, 不是物理单位 (越小越肉)。 */
#define CHASSIS_SPEED_POS    100.0f   /* ★主旋钮: 导航/点对点定位最大轮速 —— mission 全程都走它
                                         (现和扫码直行 CHASSIS_SPEED_QR 同速; 70 是早期保守值) */
#define CHASSIS_SPEED_RC      60.0f   /* 遥控平移满杆 */
#define CHASSIS_SPEED_QR     100.0f   /* 启动区扫码直行 (沿 X 慢扫找二维码) */
#define CHASSIS_SPEED_TEST    50.0f   /* 底盘单项测试 (chassis_flag.test_mode 1~6) */
#define CHASSIS_SPEED_RC_ROT  30.0f   /* 遥控旋转满杆 (RPM) */
#define CHASSIS_ACC          100      /* EMM 速度命令加速度档位 0~255 */
#define CHASSIS_LX_SENS       3.0f    /* Lx 航向积分灵敏度 (度/周期) */
/* 注: 原地旋转没有单独的速度宏 —— 它由偏航 PID 决定, 旋钮是 CHASSIS_YAW_KP / CHASSIS_YAW_MAX_OUT。
   原 CHASSIS_MAX_SPEED / CHASSIS_ROTATE_RATE 两个宏全工程没人引用(改了也没用), 已删。 */

/* ================= 点对点定位速度规划 (停靠又快又准) ================= */
/* 老代码是纯 P 控 v = kp×误差: 指数收敛, 离点越近越慢 —— 最后 200mm 要爬好几秒, 停靠慢就慢在这。
   现在按刹车曲线给速度: 远处顶满 CHASSIS_SPEED_POS 巡航 -> 进入刹车距离按恒定减速度
   sqrt(2·a·d) 减速 -> 离点 CHASSIS_POS_TAPER_MM 内线性收到死区边上为 0 (触点比老 P 控还软,
   终点速度更低, 精度不掉)。方向仍按误差比例分, 轨迹还是直线。 */
#define CHASSIS_POS_DECEL_MMPS2  300.0f  /* ★刹车减速度 (mm/s²): 300 = 0.3 m/s², 100RPM 刹停约 0.43m。
                                            调大 = 更晚刹车、更快停 (太猛会打滑/冲过头); 调小 = 更早刹车、更肉。
                                            写 0 = 整段退回老 P 控 (chassis_cfg.pos_kp), 现场兜底用 */
#define CHASSIS_POS_TAPER_MM     80.0f   /* 末端线性收口半径 (mm): 这段内速度从刹车曲线线性收到 0。
                                            必须 > 死区(10mm); 太大 = 最后一段又变慢, 太小 = 到点还带着速度冲 */
#define CHASSIS_POS_CREEP_RPM      8.0f  /* ★末端蠕动下限 (轮速 RPM): 还在死区外就至少给这么多。
                                            收口在死区边上归零, 但 EMM 低速+静摩擦会让车停在死区外
                                            10~13mm 处不动 (命令 <1RPM 被 (uint16_t) 截断成 0) ——
                                            加了它一定能走完最后几毫米。8RPM ≈ 41mm/s, 最后 10mm 约 0.25s。
                                            0 = 关闭 (退回旧行为) */

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
#define CHASSIS_YAW_MAX_OUT   60.0f   /* 修正量上限 (RPM) = ★原地旋转速度: 大角度旋转时 P 已饱和,
                                         转速就顶在这个值上, 45->60 约快 33% (yyb 同款是 30, 原 25)。
                                         再想快就加大到 70~80, 但要同步加大 CHASSIS_WHEEL_MAX。 */
#define CHASSIS_YAW_MAX_I     30.0f
#define CHASSIS_YAW_DEADZONE  0.8f    /* 度 (旋转完成判据也用它, 先保持别收太紧) */
#define CHASSIS_ROT_SETTLE_TICKS 10   /* 原地旋转到位稳定拍数 (10ms/拍): 进死区后连续这么拍都保持才判转完, 过滤"擦边判到位" */
#define CHASSIS_TRIM_CONV_FRAMES 10   /* 底盘自判纠偏到位: 视觉 corr 已落进底盘死区(再纠车也不动)后,
                                         连续收到这么多帧 0x04/0x05 才认位(帧数, 不是拍数; 按视觉实际帧率调) */
#define CHASSIS_TRIM_SETTLE_FRAMES 8  /* 纠偏到位/运动结束后丢弃的视觉帧数: 等车身停稳、拖影消失.
                                         帧数, 不是拍数; 10~20Hz 下约 0.4~0.8s, 按现场帧率调 */
#define CHASSIS_YAW_IBAND     5.0f    /* 度 */
#define CHASSIS_YAW_INVERT    (-1)    /* 修正方向反了改 1 */

/* 旋转优先限幅: 车头矫正(omega)优先, 平移(vx,vy)让位。
   每轮命令 = 平移分量(最大|vx|+|vy|) + 旋转分量(|omega|)。当平移+旋转之和
   超过每轮预算 CHASSIS_WHEEL_MAX 时, 等比压缩平移、保旋转全额 —— 高速直行
   时车头矫正始终有余量不被平移吃掉(跑不直就靠它兜)。0=关(退回裸混控)。 */
#define CHASSIS_ROT_PRIO_EN    1
#define CHASSIS_WHEEL_MAX      150.0f   /* 每轮速度预算 (RPM): 平移+旋转合计上限。
                                           平移顶到 CHASSIS_SPEED_POS(100)时还剩 50 给车头修正;
                                           不够的话平移会被压 (旋转优先, 保 omega)。
                                           原 120 是配 70 平移 + 45 修正用的, 提速后同步加大。 */

/* ================= 里程计 (麦轮 + 陀螺仪 yaw) ================= */
#define ODOM_WHEEL_RADIUS_MM  48.5f  /* 麦轮半径 (mm): 外径97/2, 5mm联轴器直连 */
#define ODOM_GEAR_RATIO       1.0f   /* 电机:轮 减速比, 直驱=1 */
#define ODOM_YAW_INVERT       1      /* 陀螺仪 yaw 方向反了改 -1 */
#define ODOM_UPDATE_CYCLES    5      /* 每 N 个 10ms 周期更新一次 (5=50ms) */
/* 单轮单拍位移上限 (电机度): 超过就当"位置计数器不连续"(电机掉电重启/内部多圈计数值复位/坏帧),
   该轮本拍不积分并把基准就地重对。物理上限 120RPM = 720°/s = 36°/50ms(约30mm), 取 400°(约339mm)
   留 10 倍余量, 正常行驶永远打不到。0 或负数 = 关闭该检查。 */
#define ODOM_MAX_WHEEL_STEP_DEG  400.0f

/* 里程计位置读取方式: 1=用 EMM 定时主动上报快照替代 4 轮串行阻塞读, 0=退回阻塞读。
   现场验证 EMM 支持 0x11/0x18 后保持 1; 不支持就在 Keil Watch 把 chassis_odom_use_autorep 写 0 回退。 */
#define CHASSIS_AUTOREP_EN     1
#define CHASSIS_AUTOREP_MS     20     /* 上报间隔 (ms) */

/* ================= 塔吊 速度总表 ================= */
/* ★ 塔吊动作快慢。上面 KIN_* 是 mission/kin 自动流程用的 (app_kin.c 的 kin_motion 初值引用);
   下面 HOIST/EXTEND_* 只有遥控手动操作塔吊 (App_Tower_Task, MyTasks 里当前被注释) 用。
   单位别混: gimbal_step_deg 是舵机斜坡 (度/10ms), 其它是 X_V2 点到点轨迹的目标速度(RPM)/加减速。 */
#define KIN_GIMBAL_STEP_DEG   2.0f    /* ★云台巡航速度 (度/10ms): 2.0 = 200°/s */
#define KIN_GIMBAL_ACC        0.05f   /* 云台梯形加速度 (度/拍², 0.05 ≈ 100ms 加到满速) */
#define KIN_GIMBAL_DEC        0.01f   /* 云台减速度 (比加速慢 = 停得稳) */
#define KIN_HOIST_VEL       150.0f    /* ★电机5 升降目标速度 (RPM) */
#define KIN_HOIST_ACC         100     /* 电机5 加/减速 */
#define KIN_HOIST_DEC         100
#define KIN_EXTEND_VEL      150.0f    /* ★电机6 伸缩目标速度 (RPM) */
#define KIN_EXTEND_ACC        100     /* 电机6 加/减速 */
#define KIN_EXTEND_DEC        100

/* 遥控手动塔吊的杆量程 (当前 MyTasks 里 App_Tower_Task 被注释, 仅调试时启用) */
#define HOIST_MAX_SPEED       150.0f  /* Ly 满杆升降速度 (RPM) */
#define HOIST_ACC             200     /* 遥控升降加减速 */
#define EXTEND_MAX_SPEED      100.0f  /* 旋钮满量程伸出速度 (RPM) */
#define EXTEND_ACC            200     /* 遥控伸缩加减速 */

/* 塔吊升降(电机5) 各工位高度 (单位: 电机度, 现场标定) */
#define HOIST_HEIGHT_PLATFORM 530.0f  /* 放置/抓取 物料平台 */
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
#define POSTURE_MATERIAL_GIMBAL 134.0f  /* 云台: 物料区探查/抓取位 (2026-10-31 用户要求改 134, 和通用探查对齐;
                                          固定位装货不再按视觉坐标转云台, 这里就是抓取角) */
#define POSTURE_MATERIAL_HOIST  HOIST_HEIGHT_ROTATE  /* 电机5: = 抓取高 460 (探查即抓取位, 无下降步) */
#define POSTURE_MATERIAL_EXTEND 260.0f  /* 电机6: 物料区预备位的伸出量 (微调完之后伸到这个值, 靠近物料) */
/* 微调前姿态的电机6: 收回 0。物料区到站先摆 134/460/0 给视觉做底盘微调(电机6 伸出去会挡
   转盘/物料的视场), 等 0x0E within=1 之后再伸到 POSTURE_MATERIAL_EXTEND 进抓取预备位。 */
#define POSTURE_MATERIAL_TRIM_EXTEND 0.0f

/* ================= 全自动流程 (mission) ================= */
/* 进站等底盘视觉微调 within 的上限 (10ms/拍, 500=5s)。到探查姿态后放开纠偏,
   等一次新的 within=1; 视觉在该工作区不发 0x04/0x05 时就等这么长时间后放行, 不卡死流程。
   !! 2026-10-31 临时关闭: app_mission.c 的 MS_TRIM 里超时放行两项已注释掉, 只认 TrimDone,
      这个宏目前不生效; 要恢复超时把 MS_TRIM 里注释掉的两项加回去。 */
#define MISSION_TRIM_WAIT_10MS  500

/* 任务码(0x06)同一内容的正常重发间隔是 1s (视觉持续重发 30s)。收到任务码时, 距上次超过
   这个拍数 (10ms/拍, 500=5s) 就当成视觉开新任务, 把 0x0F 序号/颜色对账清零。
   留了 5 倍余量: 正常重发绝不会被误判成新任务。 */
#define MISSION_QR_NEW_TASK_GAP_10MS  500

/* 启动区扫码 (2026-11-09 改法): 不再"先到 (±200,-200) 不停车通过 + 慢扫到 ±1500",
   改成点对点直接开到扫码停点 (±corner_x, mission.scan_goto_y), 停 1s (mission.scan_pause_10ms)
   后直接回物料区; 半路扫到码就不去停点。原来的 MISSION_START_PASS_MM / MISSION_START_CRUISE_RPM
   已删除 (底盘 pos_min_speed 机制保留, 目前没有流程用它)。 */

/* 物料区连抓 (kin_flag.cmd=12): 放完一件回探查姿态后继续等下一个 0x09, 一共连抓这么多个。
   只给原地/手动测试用 —— mission 的物品节拍是"0x08 给一件 -> 抓 -> 0x0F 报一件", 不要用 cmd=12。 */
#define MATERIAL_CHAIN_GRABS    3

/* ================= 导航离站避让 (物料区) ================= */
/* 物料区抓完 3 件去粗加工: 车头要从 material_yaw(90°, 朝 +Y 对着转盘) 原地转到 0°(朝 +X),
   转这一下会扫到转盘。所以出发前先沿世界 +X 平移这么远, 停稳后再转车头。
   默认 50mm; 0=关闭。启动区2 若是镜像布局, 现场按实际改成负值或 0。
   注意底盘到点死区 pos_deadzone_mm=10, 实走会少几毫米, 要更保险就加大这个值。 */
#define NAV_MATERIAL_LEAVE_DX_MM  50.0f

/* ================= 舵机 ================= */
/* 各舵机角度行程 (度), CCR = angle/range*2000+500 */
#define SERVO_GRIPPER_RANGE   270.0f
#define SERVO_PLATFORM_RANGE  270.0f
#define SERVO_GIMBAL_RANGE    360.0f

/* 上电门控: Servo_Init 写停放位初值后, 再挡 N 个 10ms 拍才放行任何舵机动作 (避开舵机上电自检) */
#define SERVO_ARM_TICKS       50

/* 夹爪 */
#define GRIPPER_CLOSE_ANGLE   105.0f
#define GRIPPER_OPEN_ANGLE    195.0f

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
