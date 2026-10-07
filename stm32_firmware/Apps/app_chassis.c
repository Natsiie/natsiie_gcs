#include "sys.h"

/**
  * @brief  麦轮底盘控制 (4 × X固件闭环步进, 速度模式)
  *         - 遥控直控: sw[0] 锁定, Rx左右 Ry前后, Lx旋转(速率直控+松杆锁航向)
  *         - 点对点定位: btn0 回原点(0,0) / btn1 沿车头往前, 里程计闭环 P 控 + 陀螺仪锁车头
  *         - 麦轮里程计: 读4轮位置 + 陀螺仪 yaw 航迹推算
  *         电机地址: 1=BL(后左) 2=BR(后右) 3=FL(前左) 4=FR(前右)
  */

/* ================= 底盘测试 (全部变量, Keil Watch 里改) ================= */
/* chassis_test_mode: 0停止 1前进 2后退 3左移 4右移 5顺时针 6逆时针 */
uint8_t chassis_test_mode  = 0;
float   chassis_test_speed = 30.0f;     /* 轮速 RPM */

/* 陀螺仪偏航修正 (平移时锁车头走直线) */
uint8_t chassis_yaw_enable = 1;         /* 1=平移时用陀螺仪修正车头 */
float   chassis_yaw_kp = 1.1f;          /* P (参考 yyb pid_choose=1) */
float   chassis_yaw_ki = 0.0f;         /* I (参考为0: 死区内才不会有积分残留一直顶) */
float   chassis_yaw_kd = 0.0f;          /* D: 锁航向不需要; 死区不挡D项, 陀螺仪噪声被放大造成直走左右抖 -> 0 */
float   chassis_yaw_max_out = 40.0f;    /* 修正量上限 (RPM) */
float   chassis_yaw_max_i = 30.0f;      /* 积分限幅 */
float   chassis_yaw_deadzone = 1.5f;    /* 死区 (度): 参考舵轮转向1.0°, 大致到达就行不微调 */
float   chassis_yaw_iband = 5.0f;       /* 积分区间 (度) */
int8_t  chassis_yaw_invert = -1;         /* 修正方向反了改 -1 */
float   chassis_yaw_target = 0.0f;      /* 目标朝向 (只读) */
float   chassis_yaw_cur = 0.0f;         /* 当前朝向 (只读) */
float   chassis_yaw_out = 0.0f;         /* 修正输出 RPM (只读) */

/* 遥控器直控底盘 (陀螺仪航向保持): sw[0]=0锁定/2解锁, Rx左右 Ry前后, Lx旋转(速率直控+松杆锁航向) */
float   chassis_rc_speed = 60.0f;       /* 平移轮速上限 RPM */
float   chassis_rc_rotate_rate = 60.0f; /* Lx 满杆旋转角速度 RPM (跟手: 直接比例) */
float   chassis_rc_settle_ms = 300.0f; /* 松杆后滑行稳定时间(ms): 车自然停稳再锁航向, 防"回弹" */
int8_t  chassis_rc_rx_invert = 1;      /* Rx 左右反了 -> 改 -1 */
int8_t  chassis_rc_ry_invert = 1;      /* Ry 前后反了 -> 改 -1 */
int8_t  chassis_rc_lx_invert = 1;      /* Lx 旋转反了 -> 改 -1 */

/* 电池电压反馈 (电机1母线电压 = 电池电压, 只读, Keil Watch 里看) */
float   chassis_vbus = 0.0f;           /* 电池电压 (V) */

/* 麦轮里程计 (读4轮位置 + 陀螺仪 yaw 航迹推算, Keil Watch 里看) */
float   chassis_odom_wheel_radius_mm = 48.5f;  /* 麦轮半径 (mm): 外径97/2, 5mm联轴器直连 */
float   chassis_odom_gear_ratio = 1.0f;        /* 减速比, 直驱=1 */
int8_t  chassis_odom_yaw_invert = 1;           /* yaw 方向反了改 -1 */
uint16_t chassis_odom_update_cycles = 5;       /* 每 N 个 10ms 周期更新一次 */
float   chassis_odom_x_mm = 0.0f;              /* 世界坐标 X (mm) */
float   chassis_odom_y_mm = 0.0f;              /* 世界坐标 Y (mm) */
float   chassis_odom_yaw_deg = 0.0f;           /* 当前航向 (度) */
float   chassis_odom_vx_mmps = 0.0f;           /* 车体前进速度 (mm/s) */
float   chassis_odom_vy_mmps = 0.0f;           /* 车体左移速度 (mm/s) */

/* 点对点定位 (btn0回原点 / btn1往前): 用里程计闭环 P 控 + 陀螺仪锁车头 */
uint8_t chassis_pos_active = 0;           /* 0=空闲 1=导航中 2=暂停(保留目标, 再按btn继续) */
float   chassis_pos_target_x_mm = 0.0f;   /* 目标点 X (世界系) */
float   chassis_pos_target_y_mm = 0.0f;   /* 目标点 Y (世界系) */
float   chassis_pos_kp = 0.12f;           /* 位置 P (RPM/mm) */
float   chassis_pos_max_speed = 60.0f;    /* 定位最大轮速 (RPM) */
float   chassis_pos_deadzone_mm = 15.0f;  /* 到点死区 (mm) */
float   chassis_pos_fwd_mm = 1200.0f;     /* btn1 前进距离 (mm) */

/* 同步发送 4 轮速度: 先暂存 4 条速度命令到 MMCL 缓冲区, 再 X_V2_Multi_Motor_Cmd(0)
   广播地址 0 一次性发出, 4 个电机同时收到同一条多机命令 -> 同步动作 */
static void Chassis_SendWheels(float w_bl, float w_br, float w_fl, float w_fr)
{
    uint8_t dir;

    dir = (w_bl < 0) ? 1 : 0; if (w_bl < 0) w_bl = -w_bl;
    X_V2_MMCL_Vel_Control(1, dir, 300, w_bl, false);
    dir = (w_br < 0) ? 1 : 0; if (w_br < 0) w_br = -w_br;
    X_V2_MMCL_Vel_Control(2, dir, 300, w_br, false);
    dir = (w_fl < 0) ? 1 : 0; if (w_fl < 0) w_fl = -w_fl;
    X_V2_MMCL_Vel_Control(3, dir, 300, w_fl, false);
    dir = (w_fr < 0) ? 1 : 0; if (w_fr < 0) w_fr = -w_fr;
    X_V2_MMCL_Vel_Control(4, dir, 300, w_fr, false);

    X_V2_Multi_Motor_Cmd(0);
}

/* 读底盘电池电压: 电机1母线电压 (=电池电压, 四轮同一条电源线)
   回包 Data[0]=0x24, Data[1..2]=16位 mV 大端; 阻塞读一次约 1ms */
static void Chassis_ReadVBus(void)
{
    /* 先清掉速度命令残留的回包 + 标志, 避免读到旧的 ACK 帧 */
    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
        CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&can.CAN_RxMsg);
    can.rxFrameFlag = false;

    X_V2_Read_Sys_Params(1, S_VBUS);   /* 读电机1电源电压 */

    uint32_t t = 0;
    while (can.rxFrameFlag == false && ++t < 2000000);
    if (can.rxFrameFlag == false) return;   /* 超时 */

    can.rxFrameFlag = false;
    if (can.CAN_RxMsg.Data[0] == 0x24)
    {
        uint16_t mv = ((uint16_t)can.CAN_RxMsg.Data[1] << 8) |
                      ((uint16_t)can.CAN_RxMsg.Data[2]);
        chassis_vbus = mv / 1000.0f;         /* mV -> V */
    }
}

/* ================= 麦轮里程计 (BL/BR/FL/FR = 电机 1/2/3/4) ================= */
static uint8_t chassis_odom_inited = 0;
static float   chassis_odom_prev_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f}; /* BL BR FL FR 上周期电机角度(度) */

/* 阻塞读单个电机实时位置(度): 成功写 *pos_deg 返回1, 失败返回0 */
static uint8_t Chassis_Odom_ReadPos(uint8_t addr, float *pos_deg)
{
    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
        CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&can.CAN_RxMsg);
    can.rxFrameFlag = false;

    X_V2_Read_Sys_Params(addr, S_CPOS);

    uint32_t t = 0;
    while (can.rxFrameFlag == false && ++t < 2000000);
    if (can.rxFrameFlag == false) return 0;   /* 超时 */
    can.rxFrameFlag = false;

    if (can.CAN_RxMsg.Data[0] == 0x36 && can.CAN_RxMsg.DLC == 7)
    {
        uint32_t mag = ((uint32_t)can.CAN_RxMsg.Data[2] << 24) |
                       ((uint32_t)can.CAN_RxMsg.Data[3] << 16) |
                       ((uint32_t)can.CAN_RxMsg.Data[4] << 8)  |
                       ((uint32_t)can.CAN_RxMsg.Data[5]);
        float p = (float)mag * 0.1f;          /* 0.1度 分辨率 */
        if (can.CAN_RxMsg.Data[1]) p = -p;    /* Data[1]=1 为负 */
        *pos_deg = p;
        return 1;
    }
    return 0;
}

static void Chassis_Odom_Update(void)
{
    float pos[4], new_pos, yaw_deg;

    /* 读 4 轮位置 (电机地址和 Chassis_SendWheels 一致: 1=BL 2=BR 3=FL 4=FR; 失败保持旧值) */
    if (Chassis_Odom_ReadPos(1, &new_pos)) pos[0] = new_pos; else pos[0] = chassis_odom_prev_pos[0];
    if (Chassis_Odom_ReadPos(2, &new_pos)) pos[1] = new_pos; else pos[1] = chassis_odom_prev_pos[1];
    if (Chassis_Odom_ReadPos(3, &new_pos)) pos[2] = new_pos; else pos[2] = chassis_odom_prev_pos[2];
    if (Chassis_Odom_ReadPos(4, &new_pos)) pos[3] = new_pos; else pos[3] = chassis_odom_prev_pos[3];

    yaw_deg = Imu_GetYaw();
    chassis_odom_yaw_deg = yaw_deg;

    if (!chassis_odom_inited)
    {
        for (int i = 0; i < 4; i++) chassis_odom_prev_pos[i] = pos[i];
        chassis_odom_inited = 1;
        return;   /* 首次只记录基准, 不积分 */
    }

    /* 电机角增量(度) -> 轮弧长(mm) */
    float scale = chassis_odom_wheel_radius_mm * (MATH_PI / 180.0f) / chassis_odom_gear_ratio;
    float d_bl = (pos[0] - chassis_odom_prev_pos[0]) * scale;
    float d_br = (pos[1] - chassis_odom_prev_pos[1]) * scale;
    float d_fl = (pos[2] - chassis_odom_prev_pos[2]) * scale;
    float d_fr = (pos[3] - chassis_odom_prev_pos[3]) * scale;

    for (int i = 0; i < 4; i++) chassis_odom_prev_pos[i] = pos[i];

    /* 正运动学 (逆解反函数): a=BL b=BR c=FL d=FR
       vx = (a - b + c - d) / 4   前进
       vy = (a + b + c + d) / 4   左移 */
    float dx_robot = (d_bl - d_br + d_fl - d_fr) / 4.0f;
    float dy_robot = (d_bl + d_br + d_fl + d_fr) / 4.0f;

    /* 车体系 -> 世界系 (陀螺仪 yaw 作航向) */
    float yaw_rad = chassis_odom_yaw_invert * yaw_deg * (MATH_PI / 180.0f);
    float c = cosf(yaw_rad), s = sinf(yaw_rad);
    chassis_odom_x_mm += dx_robot * c - dy_robot * s;
    chassis_odom_y_mm += dx_robot * s + dy_robot * c;

    /* 车体速度 (mm/s): 周期 = chassis_odom_update_cycles * 10ms */
    float cyc = (chassis_odom_update_cycles > 0) ? (float)chassis_odom_update_cycles : 1.0f;
    float dt_s = (10.0f * cyc) / 1000.0f;
    chassis_odom_vx_mmps = dx_robot / dt_s;
    chassis_odom_vy_mmps = dy_robot / dt_s;
}

/* 底盘用懒初始化: 偏航 PID / 里程计基准在任务首次进入时自动建立, 上电无需额外动作 */
void App_Chassis_Init(void)
{
}

/**
  * @brief  底盘单项测试: 按 chassis_test_mode 跑 前进/后退/左右平移 + 顺逆时针旋转
  *         平移(1~4)时用陀螺仪偏航 PID 锁车头走直线
  */
void App_Chassis_Test_Task(void)
{
    static PID_Controller_t yaw_pid;
    static uint8_t yaw_pid_inited = 0;
    float s = chassis_test_speed;
    float vx = 0, vy = 0, base_omega = 0;
    float yaw, yaw_corr, omega;
    float w_bl, w_br, w_fl, w_fr;

    yaw = Imu_GetYaw();
    chassis_yaw_cur = yaw;

    /* PID 初始化(一次) + 每周期把 Watch 里的增益同步进 PID */
    if (!yaw_pid_inited)
    {
        PID_Init(&yaw_pid, chassis_yaw_kp, chassis_yaw_ki, chassis_yaw_kd,
                 chassis_yaw_max_out, chassis_yaw_max_i,
                 chassis_yaw_deadzone, chassis_yaw_iband, 0.5f, 1);   /* is_angle_loop=1 */
        yaw_pid_inited = 1;
    }
    yaw_pid.Kp = chassis_yaw_kp;
    yaw_pid.Ki = chassis_yaw_ki;
    yaw_pid.Kd = chassis_yaw_kd;
    yaw_pid.max_out = chassis_yaw_max_out;
    yaw_pid.max_i = chassis_yaw_max_i;
    yaw_pid.deadzone = chassis_yaw_deadzone;
    yaw_pid.i_band = chassis_yaw_iband;

    /* 速度分量 (按实测映射: 前进=左右反向, 平移=四轮同向, 旋转=前后反向) */
    switch (chassis_test_mode)
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
    if (chassis_test_mode == 0)
    {
        chassis_yaw_target = yaw;
        PID_Clear(&yaw_pid);
    }

    /* 平移(1~4)时用陀螺仪修正车头; 旋转(5/6)不修正 */
    yaw_corr = 0;
    if (chassis_test_mode >= 1 && chassis_test_mode <= 4 && chassis_yaw_enable)
    {
        yaw_corr = chassis_yaw_invert * PID_Calc(&yaw_pid, chassis_yaw_target, yaw);
    }
    chassis_yaw_out = yaw_corr;

    omega = base_omega + yaw_corr;

    /* 麦轮逆运动学 (实测标定): w_bl=+vx+vy+w  w_br=-vx+vy+w  w_fl=+vx+vy-w  w_fr=-vx+vy-w */
    w_bl =  vx + vy + omega;
    w_br = -vx + vy + omega;
    w_fl =  vx + vy - omega;
    w_fr = -vx + vy - omega;

    Chassis_SendWheels(w_bl, w_br, w_fl, w_fr);   /* 四轮同步发 */
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
    float vx, vy, omega, yaw;
    float w_bl, w_br, w_fl, w_fr;

    /* 每 chassis_odom_update_cycles 个 10ms 周期更新一次里程计 (读4轮位置, 约2-4ms) */
    if (++odom_tick >= chassis_odom_update_cycles)
    {
        odom_tick = 0;
        Chassis_Odom_Update();
    }

    /* 读陀螺仪 */
    yaw = Imu_GetYaw();
    chassis_yaw_cur = yaw;

    /* PID 初始化一次 + 每周期同步 Watch 增益 */
    if (!yaw_pid_inited)
    {
        PID_Init(&yaw_pid, chassis_yaw_kp, chassis_yaw_ki, chassis_yaw_kd,
                 chassis_yaw_max_out, chassis_yaw_max_i,
                 chassis_yaw_deadzone, chassis_yaw_iband, 0.5f, 1);  /* is_angle_loop=1 */
        chassis_yaw_target = yaw;
        yaw_pid_inited = 1;
    }
    yaw_pid.Kp = chassis_yaw_kp;
    yaw_pid.Ki = chassis_yaw_ki;
    yaw_pid.Kd = chassis_yaw_kd;
    yaw_pid.max_out = chassis_yaw_max_out;
    yaw_pid.max_i = chassis_yaw_max_i;
    yaw_pid.deadzone = chassis_yaw_deadzone;
    yaw_pid.i_band = chassis_yaw_iband;

    /* 锁定: sw[0] != 2 (含中间档1) 或失联 -> 停 4 轮 + 锁当前朝向 + 清积分
       定位中则转"暂停"(保留目标): 手动定位解锁后需再按 btn0/btn1, 全程导航解锁后自动继续 */
    if (ELRS_RC.sw[0] != 2 || !ELRS_RC.online)
    {
        if (chassis_pos_active == 1) chassis_pos_active = 2;   /* 定位 -> 暂停 */
        chassis_yaw_target = yaw;
        PID_Clear(&yaw_pid);
        Chassis_SendWheels(0, 0, 0, 0);
        return;
    }

    /* 全程导航状态机 (优先于手动 btn0/btn1 定位) */
    Nav_Update();

    /* 按钮边沿: 手动定位 btn0 回原点(0,0) / btn1 沿当前车头往前; 导航中忽略避免抢目标 */
    if (!nav_active)
    {
        static bool last_btn0 = false, last_btn1 = false;

        if (ELRS_RC.btns[0] && !last_btn0)
        {
            chassis_pos_target_x_mm = 0.0f;
            chassis_pos_target_y_mm = 0.0f;
            chassis_pos_active = 1;
            chassis_yaw_target = yaw;         /* 定位期间锁当前车头朝向 */
            PID_Clear(&yaw_pid);
        }
        if (ELRS_RC.btns[1] && !last_btn1)
        {
            /* 暂停中再按 -> 恢复原目标继续; 否则新目标: 沿当前车头往前 chassis_pos_fwd_mm */
            if (chassis_pos_active != 2)
            {
                float th = chassis_odom_yaw_invert * yaw * (MATH_PI / 180.0f);
                chassis_pos_target_x_mm = chassis_odom_x_mm + chassis_pos_fwd_mm * cosf(th);
                chassis_pos_target_y_mm = chassis_odom_y_mm + chassis_pos_fwd_mm * sinf(th);
            }
            chassis_pos_active = 1;
            chassis_yaw_target = yaw;
            PID_Clear(&yaw_pid);
        }
        last_btn0 = ELRS_RC.btns[0];
        last_btn1 = ELRS_RC.btns[1];
    }

    if (chassis_pos_active == 1)
    {
        /* 点对点定位: 里程计闭环 P 控, 航向 PID 锁死 (世界系误差 -> 车体系) */
        float ex = chassis_pos_target_x_mm - chassis_odom_x_mm;
        float ey = chassis_pos_target_y_mm - chassis_odom_y_mm;
        float th = chassis_odom_yaw_invert * yaw * (MATH_PI / 180.0f);
        float ef =  ex * cosf(th) + ey * sinf(th);   /* 前进方向误差 */
        float el = -ex * sinf(th) + ey * cosf(th);   /* 左移方向误差 */

        vx = chassis_pos_kp * ef;
        vy = chassis_pos_kp * el;
        if (vx >  chassis_pos_max_speed) vx =  chassis_pos_max_speed;
        if (vx < -chassis_pos_max_speed) vx = -chassis_pos_max_speed;
        if (vy >  chassis_pos_max_speed) vy =  chassis_pos_max_speed;
        if (vy < -chassis_pos_max_speed) vy = -chassis_pos_max_speed;

        /* 到点: 前进/左移误差都进死区 -> 停 + 退出定位 */
        if (fabsf(ef) < chassis_pos_deadzone_mm && fabsf(el) < chassis_pos_deadzone_mm)
        {
            chassis_pos_active = 0;
            vx = 0.0f;
            vy = 0.0f;
        }

        omega = chassis_yaw_invert * PID_Calc(&yaw_pid, chassis_yaw_target, yaw);
    }
    else
    {
        /* 平移速度 */
        vx = chassis_rc_ry_invert * ELRS_RC.Ry * chassis_rc_speed;   /* 前进 + */
        vy = -chassis_rc_rx_invert * ELRS_RC.Rx * chassis_rc_speed;  /* 左移 + (Rx 右为正) */

        /* Lx 旋转: 摇杆直接比例控角速度 (跟手); 松杆后先滑行停稳, 再 PID 锁航向 */
        {
            float lx = chassis_rc_lx_invert * ELRS_RC.Lx;
            static uint16_t settle_tick = 0;
            uint16_t settle_max = (uint16_t)(chassis_rc_settle_ms / 10.0f);

            if (fabsf(lx) > 0.05f)
            {
                /* 摇杆在动: 直接速率控制 (跟手) */
                settle_tick = 0;
                chassis_yaw_target = yaw;
                PID_Clear(&yaw_pid);
                omega = lx * chassis_rc_rotate_rate;
            }
            else if (settle_tick < settle_max)
            {
                /* 摇杆刚回中: 目标跟随车头滑行停稳, 不加修正 -> 不"回弹" */
                settle_tick++;
                chassis_yaw_target = yaw;
                PID_Clear(&yaw_pid);
                omega = 0.0f;
            }
            else
            {
                /* 停稳后: 陀螺仪 PID 锁住当前航向 (死区内自动停) */
                omega = chassis_yaw_invert * PID_Calc(&yaw_pid, chassis_yaw_target, yaw);
            }
        }
    }
    chassis_yaw_out = omega;

    /* 麦轮逆运动学 (实测标定): w_bl=+vx+vy+w  w_br=-vx+vy+w  w_fl=+vx+vy-w  w_fr=-vx+vy-w */
    w_bl =  vx + vy + omega;
    w_br = -vx + vy + omega;
    w_fl =  vx + vy - omega;
    w_fr = -vx + vy - omega;

    Chassis_SendWheels(w_bl, w_br, w_fl, w_fr);   /* 四轮同步发 */

    /* 每 500ms 读一次电池电压 (不每周期读, 避免和速度命令抢 CAN 总线) */
    if (++vbus_tick >= 50)
    {
        vbus_tick = 0;
        Chassis_ReadVBus();
    }
}
