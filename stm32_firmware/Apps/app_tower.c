#include "sys.h"

/**
  * @brief  塔吊 + 舵机控制
  *         - 升降(电机5): Ly 速度模式
  *         - 伸出(电机6): knobs[0] 速度模式
  *         - 夹爪: btns[0] 切换开/闭
  *         - 物料平台: btns[1] 切换 3 档
  *         - 云台: knobs[1] -> 0~360°
  */
static uint8_t gripper_open = 0;
static uint8_t platform_pos = 0;
static uint8_t last_btn0 = 0, last_btn1 = 0;
static float gimbal_cur = 0.0f;   /* 云台当前角度 (斜坡限速用) */

/* 舵机测试默认值在 car_config.h (GIMBAL/PLATFORM/GRIPPER_TEST_*); 这里用宏初始化,
   现场在 Watch 里改变量即可, 调满意再回填 car_config.h 宏。
   无独立使能开关: Test_Task 每拍把云台/平台/夹爪/电机5/6 一起跑, 目标角/位置(Watch 改)
   等于当前值时自然不动 —— 想一起动就一起动, 不必分开使能。 */
float   gimbal_test_angle    = GIMBAL_TEST_ANGLE;    /* 目标角度 (0~360), Watch 里改 */
float   gimbal_test_step     = GIMBAL_TEST_STEP;     /* 加减速限幅: 每 10ms 最大转角(度), 越小越慢 */
float   gimbal_test_cur      = 0.0f;                 /* 当前角度 (只读, Watch 里看) */

float   platform_test_angle  = PLATFORM_TEST_ANGLE;  /* 目标角度 (0~270), Watch 里改 */
float   platform_test_step   = PLATFORM_TEST_STEP;   /* 加减速限幅: 每 10ms 最大转角(度), 越小越慢 */
float   platform_test_cur    = PLATFORM_TEST_ANGLE;  /* 当前角度 (只读, Watch 里看) */

float   gripper_test_angle   = GRIPPER_TEST_ANGLE;   /* 目标角度 (0~270), Watch 里改 */
float   gripper_test_cur     = GRIPPER_TEST_ANGLE;   /* 当前角度 (只读, Watch 里看) */

/* ================= 上电默认状态 =================
 * 上电自动执行: 云台先保持 135°, 电机5 升到放置高度 600, 电机6 同步伸长到放置长度 500;
 * 等电机5 到位后云台才转 310°(默认位)。
 */
uint8_t boot_enable = 1;              /* 1=上电默认状态接管上层, 0=关闭 */
float   boot_gimbal_angle1 = 135.0f;   /* 云台第一段目标角 */
float   boot_gimbal_angle2 = 310.0f;  /* 云台默认最终角 */
float   boot_gimbal_step   = 1.0f;    /* 云台斜坡限速(度/10ms) */
float   boot_gimbal_tol    = 0.5f;    /* 云台到位判定(度) */
float   boot_hoist_height  = 600.0f;  /* 电机5 放置高度 */
uint16_t boot_hoist_acc    = 100;     /* 电机5 加速 */
uint16_t boot_hoist_dec    = 100;     /* 电机5 减速 */
float   boot_hoist_vel     = 100.0f;  /* 电机5 速度 */
float   boot_hoist_tol     = 2.0f;    /* 电机5 到位判定(度) */
float   boot_extend_pos    = 500.0f;  /* 电机6 放置长度 */
uint8_t  boot_extend_dir   = 1;       /* 电机6 方向: 1=伸出 */
uint16_t boot_extend_acc   = 100;     /* 电机6 加速 */
uint16_t boot_extend_dec   = 100;     /* 电机6 减速 */
float   boot_extend_vel    = 100.0f;  /* 电机6 速度 */

uint8_t      boot_step = 0;            /* 只读: 0=云台135等电机5到位 1=云台去310 2=完成 */
static float boot_gimbal_cur = 135.0f;  /* 云台当前角(斜坡用) */

static void Tower_SendVel(uint8_t addr, float vel, uint16_t acc)
{
    uint8_t dir = 0;
    if (vel < 0) { dir = 1; vel = -vel; }
    X_V2_Vel_Control(addr, dir, acc, vel, false);
}

/* 测试用: 电机5(塔吊升降)实时位置, Keil Watch 里看它 */
float motor5_pos = 0.0f;

/* 读电机5实时位置 (回包 Data[0]=0x36, DLC=7): 非阻塞版。
   只发读请求(不忙等), 回包由 CAN 中断填 can_pos_raw[5]/can_pos_cnt[5];
   本函数取"上一次请求"的回包解析(慢一拍没关系)。绝不在任务里忙等,
   云台舵机才能每 10ms 都按时更新, 不被读电机卡到一顿一顿。 */
void Tower_ReadPos5(void)
{
    static uint16_t last_cnt = 0;   /* 已消费的帧计数 */

    if (can_pos_cnt[MOTOR_HOIST] != last_cnt)
    {
        last_cnt = can_pos_cnt[MOTOR_HOIST];
        motor5_pos = (float)can_pos_raw[MOTOR_HOIST] * 0.1f;          /* X_V2: ×0.1 = 度 */
        if (can_pos_sign[MOTOR_HOIST]) motor5_pos = -motor5_pos;      /* Data[1]=1 为负 */
    }

    X_V2_Read_Sys_Params(MOTOR_HOIST, S_CPOS);   /* 发下一个读请求(非阻塞) */
}

/* 测试用: 电机5母线电压 (V), Keil Watch 里看它 */
float motor5_vbus = 0.0f;

/* 读电机5电源电压 (回包 Data[0]=0x24, Data[1..2]=16位 mV 大端) */
void Tower_ReadVBus5(void)
{
    uint32_t t = 0;

    X_V2_Read_Sys_Params(MOTOR_HOIST, S_VBUS);

    while (++t < 2000000)
    {
        if (!can.rxFrameFlag) continue;
        can.rxFrameFlag = false;

        if ((can.CAN_RxMsg.ExtId >> 8) != MOTOR_HOIST) continue;
        if (can.CAN_RxMsg.Data[0] != 0x24) continue;

        uint16_t mv = ((uint16_t)can.CAN_RxMsg.Data[1] << 8) |
                      ((uint16_t)can.CAN_RxMsg.Data[2]);
        motor5_vbus = mv / 1000.0f;         /* mV -> V */
        return;
    }
}

/* 测试用: 电机6(塔吊伸出)实时位置, Keil Watch 里看它 */
float motor6_pos = 0.0f;

/* 读电机6实时位置 (回包 Data[0]=0x36, DLC=7): 非阻塞版, 同上 */
void Tower_ReadPos6(void)
{
    static uint16_t last_cnt = 0;   /* 已消费的帧计数 */

    if (can_pos_cnt[MOTOR_EXTEND] != last_cnt)
    {
        last_cnt = can_pos_cnt[MOTOR_EXTEND];
        motor6_pos = (float)can_pos_raw[MOTOR_EXTEND] * 0.1f;        /* X_V2: ×0.1 = 度 */
        if (can_pos_sign[MOTOR_EXTEND]) motor6_pos = -motor6_pos;    /* Data[1]=1 为负 */
    }

    X_V2_Read_Sys_Params(MOTOR_EXTEND, S_CPOS);   /* 发下一个读请求(非阻塞) */
}

void App_Tower_Init(void)
{
    gripper_open = 0;
    platform_pos = 0;
    last_btn0 = last_btn1 = 0;
    gimbal_cur = GIMBAL_MIN_ANGLE;
    gimbal_test_cur = gimbal_test_angle;
    platform_test_cur = platform_test_angle;
    gripper_test_cur = gripper_test_angle;

    boot_step = 0;
    boot_gimbal_cur = boot_gimbal_angle1;   /* 上电云台先给第一段目标 135° */

    Servo_SetAngle(SERVO_GRIPPER,  GRIPPER_OPEN_ANGLE);   /* 夹爪默认张开 (值在 car_config.h) */
    Servo_SetAngle(SERVO_PLATFORM, PLATFORM_ANGLE_A);     /* 物料平台默认状态1 = A (值在 car_config.h) */
    Servo_SetAngle(SERVO_GIMBAL,   boot_gimbal_cur);   /* 上电直接给 135°, 之后由 Tower_Boot_Task 斜坡到 310° */
}

void App_Tower_Task(void)
{
    Tower_ReadPos5();     /* 测试: 读电机5位置 */
    Tower_ReadVBus5();    /* 测试: 读电机5电源电压 */

    if (!ELRS_RC.online)
    {
        X_V2_Stop_Now(MOTOR_HOIST,  false);
        X_V2_Stop_Now(MOTOR_EXTEND, false);
        return;
    }

    /* 塔吊升降 (电机5): Ly 速度模式 */
    Tower_SendVel(MOTOR_HOIST, ELRS_RC.Ly * HOIST_MAX_SPEED, HOIST_ACC);

    /* 塔吊伸出 (电机6): knobs[0] 中位 500 -> ±速度 */
    float extend = (ELRS_RC.knobs[0] - 500) / 500.0f * EXTEND_MAX_SPEED;
    Tower_SendVel(MOTOR_EXTEND, extend, EXTEND_ACC);

    /* 夹爪: btns[0] 上升沿切换开/闭 */
    if (ELRS_RC.btns[0] && !last_btn0)
    {
        gripper_open = !gripper_open;
        Servo_SetAngle(SERVO_GRIPPER, gripper_open ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE);
    }
    last_btn0 = ELRS_RC.btns[0];

    /* 物料平台: btns[1] 上升沿切换 3 档 */
    if (ELRS_RC.btns[1] && !last_btn1)
    {
        platform_pos = (platform_pos + 1) % 3;
        float ang = (platform_pos == 0) ? PLATFORM_ANGLE_A :
                    (platform_pos == 1) ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C;
        Servo_SetAngle(SERVO_PLATFORM, ang);
    }
    last_btn1 = ELRS_RC.btns[1];

    /* 云台: knobs[1] -> 目标角度, 斜坡限速, 一点一点逼近 */
    float gimbal_target = (float)ELRS_RC.knobs[1] / 1000.0f * SERVO_GIMBAL_RANGE;
    float d = gimbal_target - gimbal_cur;
    if (d >  GIMBAL_MAX_STEP) d =  GIMBAL_MAX_STEP;
    if (d < -GIMBAL_MAX_STEP) d = -GIMBAL_MAX_STEP;
    gimbal_cur += d;
    Servo_SetAngle(SERVO_GIMBAL, gimbal_cur);
}

/* 云台角度调试任务: 每 10ms 调一次, 斜坡限速统一走 Servo_MoveTo */
void Gimbal_Test_Task(void)
{
    Servo_MoveTo(SERVO_GIMBAL, gimbal_test_angle, gimbal_test_step);
    gimbal_test_cur = Servo_GetAngle(SERVO_GIMBAL);   /* 只读回显 (Watch 看当前角) */
}

/* 物料平台角度调试任务: 每 10ms 调一次, 保留加减速限幅(斜坡), 一点一点逼近目标角 */
void Platform_Test_Task(void)
{
    float d = platform_test_angle - platform_test_cur;
    if (d >  platform_test_step) d =  platform_test_step;
    if (d < -platform_test_step) d = -platform_test_step;
    platform_test_cur += d;
    Servo_SetAngle(SERVO_PLATFORM, platform_test_cur);
}

/* 夹爪角度调试任务: 每 10ms 调一次, 立即到位 (夹爪无斜坡) */
void Gripper_Test_Task(void)
{
    Servo_SetAngle(SERVO_GRIPPER, gripper_test_angle);
    gripper_test_cur = Servo_GetAngle(SERVO_GRIPPER);   /* 只读回显 (Watch 看当前角) */
}

/* 集中测试入口: 每 10ms 调一次。云台/平台/夹爪/电机5/6 每拍一起跑, 无独立使能开关;
   想只动某一路就把其它路的目标值(Watch 里改)设成当前值即可。 */
void Test_Task(void)
{
    Gimbal_Test_Task();
    Platform_Test_Task();
    Gripper_Test_Task();
    Motor56_Test_Task();
}

/* 上电默认状态: 只执行一次
 *   云台: 先保持 135°, 等电机5 升到放置高度 600 后再转 310°(默认位), 带斜坡限速
 *   电机5: 升到放置高度 600
 *   电机6: 同步伸长到放置长度 500
 */
void Tower_Boot_Task(void)
{
    if (!boot_enable || boot_step >= 2) return;   /* 已完成, 不再执行 */

    if (boot_step == 0)
    {
        /* 阶段0: 云台保持 135°, 电机5 升到放置高度, 电机6 伸长到放置长度 */
        static uint8_t boot_phase = 0;   /* 0/1 交错: 发命令 / 读位置 */

        boot_gimbal_cur = boot_gimbal_angle1;
        Servo_SetAngle(SERVO_GIMBAL, boot_gimbal_cur);

        /* 关键: 位置命令和读位置不能同一拍做。发完位置命令紧接着就发读请求,
           读请求会紧跟位置命令上总线, 打断电机还没处理完的位置命令, 结果电机
           "收到命令但不动"(测试代码能走, 是因为它先发5号再发6号, 5号命令和读
           之间天然隔了 2 帧)。改成交错: 这一拍发命令, 下一拍再读位置 */
        boot_phase ^= 1;

        if (boot_phase)
        {
            X_V2_Traj_Pos_Control(MOTOR_HOIST, 0, boot_hoist_acc, boot_hoist_dec,
                                  boot_hoist_vel, boot_hoist_height, 1, 0);
            X_V2_Traj_Pos_Control(MOTOR_EXTEND, boot_extend_dir, boot_extend_acc, boot_extend_dec,
                                  boot_extend_vel, boot_extend_pos, 1, 0);
        }
        else
        {
            Tower_ReadPos5();   /* 读电机5实时位置 */
            {
                float e = motor5_pos - boot_hoist_height;
                if (e < 0) e = -e;
                if (e < boot_hoist_tol) boot_step = 1;   /* 电机5 到位 -> 开始转云台 */
            }
        }
    }
    else   /* boot_step == 1 */
    {
        /* 阶段1: 云台 135 -> 310 斜坡限速 */
        float d = boot_gimbal_angle2 - boot_gimbal_cur;
        if (d >  boot_gimbal_step) d =  boot_gimbal_step;
        if (d < -boot_gimbal_step) d = -boot_gimbal_step;
        boot_gimbal_cur += d;
        Servo_SetAngle(SERVO_GIMBAL, boot_gimbal_cur);

        {
            float e = boot_gimbal_cur - boot_gimbal_angle2;
            if (e < 0) e = -e;
            if (e < boot_gimbal_tol) boot_step = 2;   /* 云台到位 -> 完成 */
        }
    }
}

/* ================= 电机5/6 测试 (Keil Watch 里改) =================
 * 调电机5(升降)和电机6(伸出): 位置模式为主, 也可切速度模式。
 * 在 Watch 里改目标位置/速度/方向, motor5_pos / motor6_pos 只读回显实时位置。
 * 与云台/平台/夹爪测试同拍一起跑(见 Test_Task)。发命令和读位置交错一拍:
 * 避免"位置命令后紧跟读请求"打断电机(同 Tower_Boot_Task, 发送侧冲突)。
 * 读位置已改成非阻塞(见 Tower_ReadPos5/6): 只发请求+取 CAN 中断填好的专用槽
 * can_pos_raw/can_pos_cnt, 一微秒都不忙等 —— 云台才能每 10ms 按时更新、丝滑。
 */
uint8_t  motor5_test_mode = 0;      /* 0=位置模式, 1=速度模式 */
uint8_t  motor6_test_mode = 0;
uint8_t  motor5_test_dir = 0;       /* 位置模式方向: 0=正(升), 1=反 */
uint8_t  motor6_test_dir = 1;       /* 位置模式方向: 1=伸出(和流程一致) */
float    motor5_test_pos = 0.0f;    /* 电机5 目标位置(度) */
float    motor6_test_pos = 0.0f;    /* 电机6 目标位置(度) */
float    motor5_test_vel = 100.0f;  /* 速度(RPM) */
float    motor6_test_vel = 100.0f;
uint16_t motor5_test_acc = 100;     /* 加减速(RPM/s) */
uint16_t motor6_test_acc = 100;

void Motor56_Test_Task(void)
{
    static uint8_t phase = 0;   /* 每拍翻转: 1=发命令 0=读位置, 命令与读错开一拍 */

    phase ^= 1;

    if (phase)
    {
        /* 电机5 升降 */
        if (motor5_test_mode == 0)
            X_V2_Traj_Pos_Control(MOTOR_HOIST, motor5_test_dir, motor5_test_acc, motor5_test_acc,
                                  motor5_test_vel, motor5_test_pos, 1, 0);
        else
            Tower_SendVel(MOTOR_HOIST, motor5_test_vel, motor5_test_acc);

        /* 电机6 伸出 */
        if (motor6_test_mode == 0)
            X_V2_Traj_Pos_Control(MOTOR_EXTEND, motor6_test_dir, motor6_test_acc, motor6_test_acc,
                                  motor6_test_vel, motor6_test_pos, 1, 0);
        else
            Tower_SendVel(MOTOR_EXTEND, motor6_test_vel, motor6_test_acc);
    }
    else
    {
        /* 读回实时位置 (Watch 看) */
        Tower_ReadPos5();
        Tower_ReadPos6();
    }
}

/* ================= 抓取+放置流程 (全部变量, Keil Watch 里改) ================= */

/* 触发: sw[grab_sw] 从 0 拨到 1 -> 执行一次抓取+放置; flow_cmd 写 1~3 也可手动触发(调试) */
uint8_t flow_cmd = 0;
uint8_t flow_step = 0;    /* 只读: 0待命 1准备抓取 2下探 3夹爪闭 4抓回 5升放置 6转放置 7伸出 8夹爪开 9回默认 */
uint8_t grab_sw = 2;      /* 触发开关在 ELRS_RC.sw[] 里的下标 (3位开关, 取值 0/1/2) */
uint8_t grab_count = 0;   /* 只读: 已执行抓取+放置次数 */
uint8_t flow_dir = 0;     /* 0=抓取放置(抓前面->放平台, 平台A->B->C装满); 1=放置抓取(抓平台->放前面, 平台C->B->A清空) */

/* 视觉握手: 把自动流程拆开, 位置对了才能抓取/放置 (上层写命令, 下位机执行后清0) */
uint8_t vision_mode = 0;   /* 0=全自动(视觉未接/联调, 不等待); 1=分步(装货等抓取指令, 卸货等松开指令) */
uint8_t grab_cmd = 0;      /* 上层写1: 装货阶段 电机5 从550降到447 下探抓取 */
uint8_t release_cmd = 0;   /* 上层写1: 卸货阶段 松开夹爪 */
uint8_t wait_grab = 0;     /* 只读: 1=已到550, 等抓取指令 */
uint8_t wait_release = 0;  /* 只读: 1=已到放置位, 等松开指令 */

/* 云台 (SERVO_GIMBAL): 待命 310 / 抓取 135 / 放置 310 */
float default_gimbal_angle = 310.0f;
float grab_gimbal_angle = 135.0f;
float release_gimbal_angle = 310.0f;
float gimbal_max_step = 1.0f;   /* 度/每 10ms, 越小越慢 */
float gimbal_done_tol = 0.5f;   /* 云台到位判定(度) */

/* 电机5 升降 (id=5, 行程 0~648.79): 抓取高度447(装载/卸货相同); 放置高度按阶段: 装载480 / 卸货50 */
float height1 = 447.0f;             /* 抓取高度 (准备抓取/抓回) */
float grab_height_low = 447.0f;     /* 下探抓料高度 (=抓取高度, 不再单独下探) */
float grab_approach_height = 550.0f;/* 装货阶段: 抓取前预降(视觉对准)高度, 抓取指令后才降到447 */
float place_height_load   = 480.0f; /* 装载(flow_dir=0) 放置高度 */
float place_height_unload = 50.0f;  /* 卸货(flow_dir=1) 放置高度 */

/* 电机6 伸出 (id=6, 行程 0~808.8) */
uint8_t  extend_dir = 1;
uint16_t extend_acc = 100;
uint16_t extend_dec = 100;
float    extend_vel = 100.0f;
float    grab_extend_pos = 600.0f;    /* 抓取位 */
float    release_extend_pos = 500.0f; /* 放置位 */

/* 延时 (10ms 单位 = 循环次数, 待细调) */
uint16_t move_wait_10ms = 100;    /* 电机5/6 到位等待 */
uint16_t gripper_delay_10ms = 50; /* 夹爪动作后等待 */

/* 物料平台 (SERVO_PLATFORM): 独立 3 档, platform_sel 0/1/2 -> car_config A/B/C */
uint8_t platform_sel = 0;

/* 夹爪 (SERVO_GRIPPER): 上电张开, 由流程控制 */
float gripper_open_angle  = GRIPPER_OPEN_ANGLE;   /* 值在 car_config.h */
float gripper_close_angle = GRIPPER_CLOSE_ANGLE;  /* 值在 car_config.h */

/* 到位判定: |cur - tgt| < tol (纯本地计算, 只用于云台斜坡) */
static uint8_t PosDone(float cur, float tgt, float tol)
{
    float e = cur - tgt;
    if (e < 0) e = -e;
    return e < tol;
}

/**
  * @brief  抓取+放置流程 (每 10ms 调用一次)
  *         待命(默认): 云台 310, 电机5 放置高度, 电机6 不动, 夹爪保持
  *         触发: sw[grab_sw] 从 0 拨到 1 -> 执行一次抓取+放置, 完事回默认
  *         flow_dir: 0=抓取放置(抓前面->放平台, 平台A->B->C装满);
  *                   1=放置抓取(抓平台->放前面, 平台C->B->A清空, 即动作反一下)
  *         除第一次外, 每次触发先转物料平台到下一档
  *         视觉握手(vision_mode=1): 装货抓取 电机5先到550停住等抓取指令(grab_cmd=1)再降到447;
  *                                  卸货放置 夹爪先不松, 等松开指令(release_cmd=1)再松。
  *         vision_mode=0 时全自动跑通(视觉未接, 联调用)。
  */
void Tower_Test_Task(void)
{
    static float gim_cur = 310.0f;      /* 云台当前角 (与 default_gimbal_angle 一致) */
    static float gripper_angle = 80.0f; /* 夹爪当前角 */
    static uint16_t wait_tick = 0;      /* 延时计数 */
    static uint8_t last_grab_sw = 0;    /* 触发开关上次值(边沿检测) */
    static uint8_t grab_sw_ready = 0;   /* 首拍同步开关值, 防上电误触发 */
    static float pick_gimbal = 135.0f, drop_gimbal = 310.0f;   /* 本次 抓取点/放置点 云台角 */
    static float pick_extend = 600.0f, drop_extend = 500.0f;  /* 本次 抓取点/放置点 伸出长 */
    static uint8_t last_flow_dir = 0xFF;   /* 方向切换检测 (首次=无效值, 保证首拍同步) */
    float hoist_pos;
    float place_h;
    float gim_tgt;
    float d;

    /* 上电默认状态未完成前不接管 (等 Tower_Boot_Task 跑完) */
    if (boot_enable && boot_step < 2) return;

    /* ---- 方向切换 -> 重新计数 (每个方向第一个周期不转平台, 从当前档开始) ---- */
    if (flow_dir != last_flow_dir)
    {
        last_flow_dir = flow_dir;
        grab_count = 0;
    }

    /* ---- 触发: sw[grab_sw] 从 0 拨到 1 (或 flow_cmd 写 1~3 调试) ---- */
    {
        uint8_t swv = ELRS_RC.sw[grab_sw];
        if (!grab_sw_ready) { last_grab_sw = swv; grab_sw_ready = 1; }  /* 首拍同步, 防上电误触发 */
        uint8_t sw_trig  = (swv == 1 && last_grab_sw != 1);
        uint8_t cmd_trig = (flow_cmd >= 1 && flow_cmd <= 3);
        if (flow_step == 0 && (sw_trig || cmd_trig))
        {
            grab_count++;                          /* 第几次执行 */

            /* 按方向定 抓取点/放置点 (云台角 + 伸出长) */
            if (flow_dir == 0)      /* 抓取放置: 抓前面 -> 放平台 */
            {
                pick_gimbal = grab_gimbal_angle;
                pick_extend = grab_extend_pos;
                drop_gimbal = release_gimbal_angle;
                drop_extend = release_extend_pos;
            }
            else                    /* 放置抓取: 抓平台 -> 放前面 */
            {
                pick_gimbal = release_gimbal_angle;
                pick_extend = release_extend_pos;
                drop_gimbal = grab_gimbal_angle;
                drop_extend = grab_extend_pos;
            }

            if (grab_count >= 2)                   /* 除第一次外, 先转平台到下一档 */
            {
                if (flow_dir == 0) platform_sel = (platform_sel + 1) % 3;   /* A->B->C 装满 */
                else               platform_sel = (platform_sel + 2) % 3;   /* C->B->A 清空 */
            }
            flow_cmd = 0;
            wait_tick = 0;
            flow_step = 1;                         /* 从准备抓取开始 */
        }
        last_grab_sw = swv;
    }

    /* ---- 云台: 待命/回默认, 抓取段 pick_gimbal, 放置段 drop_gimbal ---- */
    if (flow_step >= 1 && flow_step <= 5)      gim_tgt = pick_gimbal;
    else if (flow_step >= 6 && flow_step <= 8) gim_tgt = drop_gimbal;
    else                                       gim_tgt = default_gimbal_angle;
    d = gim_tgt - gim_cur;
    if (d >  gimbal_max_step) d =  gimbal_max_step;
    if (d < -gimbal_max_step) d = -gimbal_max_step;
    gim_cur += d;
    Servo_SetAngle(SERVO_GIMBAL, gim_cur);

    /* ---- 物料平台: 独立 3 档直接给 ---- */
    Servo_SetAngle(SERVO_PLATFORM, platform_sel == 0 ? PLATFORM_ANGLE_A :
                                   platform_sel == 1 ? PLATFORM_ANGLE_B : PLATFORM_ANGLE_C);

    /* ---- 电机5 目标 (按步; 换步立即发, 同一步约1s兜底重发一次, 不能每拍发) ---- */
    place_h = (flow_dir == 0) ? place_height_load : place_height_unload;
    if (flow_step == 2 || flow_step == 3)
        hoist_pos = grab_height_low;      /* 下探/夹爪闭 447 (抓取) */
    else if (flow_step == 1)
        hoist_pos = (flow_dir == 0) ? grab_approach_height : height1;  /* 装货550预降 / 卸货447直接抓 */
    else if (flow_step == 4)
        hoist_pos = height1;              /* 抓回 447 */
    else
        hoist_pos = place_h;              /* 待命/升/放置/回默认 480(装载)/50(卸货) */
    {
        static uint8_t  last_hoist_step = 0xFF;
        static uint16_t hoist_send_cnt  = 100;
        if (flow_step != last_hoist_step || hoist_send_cnt >= 100)
        {
            last_hoist_step = flow_step;
            hoist_send_cnt  = 0;
            X_V2_Traj_Pos_Control(MOTOR_HOIST, 0, 100, 100, 100.0f, hoist_pos, 1, 0);
        }
        hoist_send_cnt++;
    }

    /* ---- 电机6: 待命不动; 抓取段 pick_extend, 放置段 drop_extend (换步立即发, 同一步约1s兜底重发) ---- */
    {
        static uint8_t  last_ext_step = 0xFF;
        static uint16_t ext_send_cnt  = 100;
        if (flow_step == 1 || flow_step == 7)
        {
            float ext_pos = (flow_step == 1) ? pick_extend : drop_extend;
            if (flow_step != last_ext_step || ext_send_cnt >= 100)
            {
                X_V2_Traj_Pos_Control(MOTOR_EXTEND, extend_dir, extend_acc, extend_dec, extend_vel,
                                      ext_pos, 1, 0);
                ext_send_cnt = 0;
            }
            ext_send_cnt++;
        }
        else
        {
            ext_send_cnt = 100;   /* 离开 1/7, 下次进入立即发 */
        }
        last_ext_step = flow_step;
    }

    /* ---- 夹爪 + 步骤推进 ---- */
    switch (flow_step)
    {
    case 0:   /* 待命: 夹爪保持当前态 */
        wait_grab = 0;
        wait_release = 0;
        Servo_SetAngle(SERVO_GRIPPER, gripper_angle);
        break;

    case 1:   /* 准备抓取: 云台->抓取点, 电机6->抓取伸出, 电机5->550(装货)/447(卸货); 装货分步等抓取指令 */
        if (PosDone(gim_cur, pick_gimbal, gimbal_done_tol))
        {
            if (flow_dir == 0 && vision_mode)   /* 装货+分步: 停在550等抓取指令 */
            {
                wait_grab = 1;
                if (!grab_cmd) break;
                grab_cmd = 0;
            }
            wait_grab = 0;
            wait_tick = 0;
            flow_step = 2;
        }
        break;

    case 2:   /* 下探(抓取高度) */
        if (++wait_tick >= move_wait_10ms) { wait_tick = 0; flow_step = 3; }
        break;

    case 3:   /* 夹爪闭 */
        gripper_angle = gripper_close_angle;
        Servo_SetAngle(SERVO_GRIPPER, gripper_angle);
        if (++wait_tick >= gripper_delay_10ms) { wait_tick = 0; flow_step = 4; }
        break;

    case 4:   /* 抓回(抓取高度) */
        if (++wait_tick >= move_wait_10ms) { wait_tick = 0; flow_step = 5; }
        break;

    case 5:   /* 升(放置高度) */
        if (++wait_tick >= move_wait_10ms) { wait_tick = 0; flow_step = 6; }
        break;

    case 6:   /* 云台转放置点 (斜坡到位) */
        if (PosDone(gim_cur, drop_gimbal, gimbal_done_tol))
        { wait_tick = 0; flow_step = 7; }
        break;

    case 7:   /* 电机6->500 */
        if (++wait_tick >= move_wait_10ms) { wait_tick = 0; flow_step = 8; }
        break;

    case 8:   /* 夹爪开, 放置完成 (卸货+分步: 等松开指令再松) */
        if (flow_dir == 1 && vision_mode)   /* 卸货+分步: 夹爪先不松, 等松开指令 */
        {
            wait_release = 1;
            if (!release_cmd) break;
            release_cmd = 0;
        }
        wait_release = 0;
        gripper_angle = gripper_open_angle;
        Servo_SetAngle(SERVO_GRIPPER, gripper_angle);
        if (++wait_tick >= gripper_delay_10ms) { wait_tick = 0; flow_step = 9; }
        break;

    case 9:   /* 回默认: 云台->310, 电机5->放置高度, 电机6停发; 等云台到位 */
        if (PosDone(gim_cur, default_gimbal_angle, gimbal_done_tol))
        { wait_tick = 0; flow_step = 0; }
        break;
    }
}
