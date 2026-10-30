#include "sys.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         跑在 DATE_Tasks (10ms)。只做「调度」：读底盘/塔吊/串口状态, 下发 nav_cmd / kin_flag.cmd,
  *         不直接动电机/舵机。mission.enable=0 时完全不介入, 保留原「1功能测试代码」。
  *
  *         赛程 (两个启动区共用, 坐标随 nav_zone):
  *           上电引导位姿 -> 等按键(短按切区/长按启动)
  *           -> 直连物料点(跳过拐角): 先走 X 再走 Y, 二维码沿路后台扫
  *           -> 全程导航: 从物料点起, 之后每去一个工作区都「中点旋转 -> 工作区作业 -> 回中点」
  *           工作区顺序: 物料(装货) -> 粗加工(卸货+回装) -> 暂存(堆叠), 走两轮。
  *
  *         注意: 底盘移动复用 App_Chassis_Task, 受遥控 sw[0]==2(解锁)且在线 门控
  *               (chassis_flag.need_sw_unlock=1 默认开启, 写 0 旁路, 纯视觉/mission 可不用遥控)。
  */

/* ================= 状态 ================= */
enum {
    MS_BOOT = 0,           /* 上电引导位姿: 夹爪开 -> 平台A -> 塔吊 Tower_Boot_Task 姿态 310/600/500 */
    MS_IDLE,               /* 等按键 (短按切区/长按启动) */
    MS_QR_SCAN_GOTO,       /* 启动区: 先到 (±50,-50) 点 */
    MS_QR_SCAN_GOTO_WAIT,  /* 等 (±50,-50) 到位 */
    MS_QR_SCAN_SLOW,       /* 启动区: 沿 ±X 慢慢直走, 扫码/到 ±1500 停 */
    MS_QR_GOTO_MAT_X,      /* 直连物料点: 先走 X (保持当前 y) */
    MS_QR_GOTO_MAT_X_WAIT, /* 等 X 到位 -> 转 material_yaw */
    MS_QR_GOTO_MAT_Y_WAIT, /* 等 Y 到位 */
    MS_QR_ROTATE_MAT,      /* 转 material_yaw (X->转->Y) */
    MS_NAV,                /* 全程导航运行中, 等 nav_wait */
    MS_ROTATE,             /* 中点原地旋转 */
    MS_TRIM,               /* 工作区视觉纠偏 (姿态到位后放开, 等一帧新 within=1 或超时后锁底盘) */
    MS_WAIT_MANUAL,        /* 手动模式: 塔吊作业用 Kin_TestRun, 等 Watch 写 mission.work_done=1 */
    MS_POSE_DEFAULT,       /* 塔吊收探查姿态: 物料区 135/460/200, 其它 135/600/0 (清视觉视场) */
    MS_LOAD,               /* 装货 (物料区) */
    MS_UNLOAD,             /* 卸货 (粗加工: 平台 -> 环, 落高50, 按二维码环序) */
    MS_RELOAD,             /* 回装 (粗加工: 环 -> 平台, 按二维码环序) */
    MS_STACK,              /* 堆叠 (暂存: 平台 -> 环, 落高50/400, 按二维码环序) */
    MS_POSE_BOOT,          /* 塔吊收运动姿态 310/600/500 (离站前) */
    MS_DONE,               /* 收车完成 */
};

/* 每个塔吊操作的模式 (Watch 改): 0=视觉自动(默认), 1=手动debug(停到位等你 Watch 驱动), 2=跳过 */
#define MISSION_OP_VISION 0
#define MISSION_OP_MANUAL 1
#define MISSION_OP_SKIP   2

/* ================= 打包变量 (Keil Watch 只看 mission) ================= */
Mission_t mission = {
    .enable           = 1,
    .mode_load        = MISSION_OP_VISION,  /* 装货(物料区) */
    .mode_unload      = MISSION_OP_VISION,  /* 卸货(粗加工区) */
    .mode_reload      = MISSION_OP_VISION,  /* 回装(粗加工区) */
    .mode_stack       = MISSION_OP_VISION,  /* 堆叠(暂存区) */
    .scan_goto_x      = 200.0f,             /* 启动区1: 先去的点 X (mm); 启动区2 取 -50 */
    .scan_goto_y      = -200.0f,            /* 两区共用: 先去的点 Y (mm) */
    .scan_stop_x      = 1500.0f,            /* 启动区1: 慢扫 |X| 上限 (mm), 到即停; 启动区2 取 -1500 */
    .vision_timeout   = 1000,               /* 工作区视觉超时(10ms/拍), 1000=10s */
    .trim_wait_10ms   = MISSION_TRIM_WAIT_10MS,  /* 进站等底盘微调 within 上限 5s, 等不到放行 */
    .scan_pause_10ms  = 500,                /* 扫码直行到 corner x 暂停时长(10ms/拍), 200=2s */
    .state            = MS_BOOT,
};

/* 手动debug 当前等的是哪个操作 (进入 MS_WAIT_MANUAL 前填, 决定写完 done 后去哪) */
enum { MSOP_LOAD = 0, MSOP_UNLOAD = 1, MSOP_RELOAD = 2, MSOP_STACK = 3 };
static uint8_t ms_manual_op = MSOP_LOAD;

/* 扫码流程/视觉超时参数已打包进 mission (见上方 Mission_t 初始化) */

#define MISSION_ITEM_TOTAL 6             /* 总物料件数: 2 轮 x 3 件 */

/* ================= 物料颜色对账打包变量 (Keil Watch 只看 mission_color) ================= */
/* 二维码解析结果: 第2组(第1轮环序)/第4组(第2轮环序) 给放置用 -> ms_qr_rings (内部, 经 Mission_Ring 读);
   第1组(第1轮颜色)/第3组(第2轮颜色) 存 mission_color.expected, 和实际抓到的颜色逐件对账。
   "实际抓到的颜色" 在物料区每闭一次夹爪时记一件 (见 Mission_NoteGrabColor), 手动 cmd=10/12 也记。
   注意口径: 0x08 的 color_id 是视觉按它自己解析的任务码过滤后的结果, 正常必然和二维码一致;
   这一对账抓的是"视觉任务码/状态机跑偏"(以及 task_mode 关掉时的乱发), 抓不到摄像头把颜色认错。 */
MissionColor_t mission_color = {
    .qr_colors_ok = 0,
    .ok           = 0,
    .err          = 0,
    .total        = MISSION_ITEM_TOTAL,
    .cnt          = 0,
    .clr          = 0,
    .expected     = { {1,2,3}, {1,2,3} },   /* 没扫到码时的占位: 按 1/2/3 记 */
    .seq          = { 0,0,0,0,0,0 },        /* 0 = 这件还没抓到 */
};
static uint8_t ms_count_shown = 0;      /* 1=本次已上屏过对账结果 (新一场长按启动清0) */

static uint8_t ms_qr_rings[2][3] = { {1,2,3}, {1,2,3} };   /* [轮][序号] = 环号(1~3) */
static uint8_t ms_qr_valid = 0;
static uint8_t ms_qr_received = 0;   /* 收到过二维码帧(0x06)标志(慢扫用); 启动/进慢扫时清 0 */
static uint16_t ms_vision_timeout = 0;  /* 视觉超时计数(10ms/拍), 收新视觉指令清零 */
static uint16_t ms_trim_tick = 0;       /* 进站等底盘微调 within 的计时 (10ms/拍), MS_TRIM 里累加 */
static uint8_t  ms_ok_count  = 0;        /* 完成正确的工作区动作数 (正常走完 -> 对) */
static uint8_t  ms_err_count = 0;        /* 视觉超时/失败的工作区动作数 (超时强制完成 -> 错) */
#define MISSION_WORK_TOTAL 6             /* 总工作区动作数: 2 轮 x 3 区(物料/粗加工/暂存) */

/* 扫码直行: 到 corner x 停 2s 的状态 */
static uint8_t  ms_scan_corner_done = 0; /* 1=已到过 corner x 并停过, 之后不再停 */
static uint8_t  ms_scan_paused      = 0; /* 1=正停在 corner x 等 mission.scan_pause_10ms */
static uint32_t ms_scan_pause_tick  = 0; /* 暂停起始拍 (ms_tick) */

static uint32_t ms_tick  = 0;
static uint8_t  ms_round = 0;            /* 当前轮: 0=第1轮, 1=第2轮 */
/* mission.state / mission.sub 已打包进 mission.state / mission.sub */
static uint8_t  ms_key_last = 0;
static uint32_t ms_key_press_tick = 0;

/* ================= 外部接口 ================= */
extern uint8_t        nav_zone;
extern uint8_t        nav_cmd, nav_active, nav_wait;
extern int8_t         nav_step;
extern NavPointId     nav_at_point;
extern uint8_t        nav_round;
extern NavZoneCoord_t nav_zone_coord[2];
void       Nav_Start(void); void Nav_Stop(void); void Nav_Continue(void);
void       Nav_StartAt(NavPointId id);
void       Nav_GetPointCoord(NavPointId id, float *x, float *y);
NavPointId Nav_NextPoint(void);

/* kin_* 已打包进 kin_flag/kin_test/kin_cal/kin_motion/kin_tgt (app_kin.h, 经 sys.h 引入) */
void Kin_BootPosture(void);
void Kin_Stop(void);

/* chassis_* 已打包进 chassis_flag/chassis_cfg/chassis_val (app_chassis.h, 经 sys.h 引入) */
void Chassis_RotateTo(float yaw_deg);
void Chassis_ResetHeading(void);
uint8_t Chassis_TrimSkipStation(NavPointId id);

extern volatile uint8_t Serial_ColorTargetReady;
extern volatile uint8_t Serial_QRReady;
extern volatile uint8_t Serial_VisionCmd;
extern Serial_RxData_t   Serial_RxData;

void Screen_ShowZone(uint8_t zone);
void Screen_ShowCount(uint8_t ok, uint8_t err, uint8_t total);

/* ================= 二维码解析 ================= */
/* "156+231+516+312" -> 4组3位: 第2/4组(d[3..5]/d[9..11])作环序, 第1/3组(d[0..2]/d[6..8])作颜色顺序 */
static void Mission_ParseQR(const char *s)
{
    uint8_t d[12], n = 0, i, colors_ok = 1;
    while (*s && n < 12)
    {
        if (*s >= '0' && *s <= '9') d[n++] = (uint8_t)(*s - '0');
        s++;
    }
    if (n != 12) return;
    for (i = 0; i < 3; i++)
    {
        uint8_t r1 = d[3 + i], r2 = d[9 + i];
        uint8_t c1 = d[0 + i], c2 = d[6 + i];
        if (r1 < 1 || r1 > 3) r1 = i + 1;
        if (r2 < 1 || r2 > 3) r2 = i + 1;
        ms_qr_rings[0][i] = r1;
        ms_qr_rings[1][i] = r2;
        mission_color.expected[0][i] = c1;
        mission_color.expected[1][i] = c2;
        if (c1 < 1 || c1 > 6 || c2 < 1 || c2 > 6) colors_ok = 0;   /* 颜色组不是 1~6: 不可信, 不参与对账 */
    }
    mission_color.qr_colors_ok = colors_ok;
    ms_qr_valid = 1;
}

static uint8_t Mission_Ring(uint8_t round, uint8_t idx)
{
    return ms_qr_rings[round ? 1 : 0][idx];
}

/* 物料颜色对账: app_kin 在物料区每闭一次夹爪(拿到一件物料)时调一次。
   color = 闭爪那一刻的 0x08 color_id (1=红 2=黄 3=蓝 4=绿 5=黑 6=浅蓝), 期望值取二维码颜色组。
   对不上只记错, 不影响抓取流程。手动 cmd=10/12 连抓也走这里, 所以不需要跑 mission 也能看。 */
void Mission_NoteGrabColor(uint8_t color)
{
    uint8_t i;
    if (color < 1 || color > 6) return;                    /* 0=视觉还没发过, 越界丢, 不占件数 */
    if (mission_color.cnt >= MISSION_ITEM_TOTAL) return;   /* 一场最多记 6 件 */
    i = mission_color.cnt;
    mission_color.seq[i] = color;
    mission_color.cnt = (uint8_t)(i + 1);
    if (!mission_color.qr_colors_ok) return;               /* 没扫到码/颜色组不可信: 这一件不判 */
    if (color == mission_color.expected[i / 3][i % 3]) mission_color.ok++;
    else                                              mission_color.err++;
}

/* 房务: 处理 Watch 写的 mission_color.clr=1 -> 清空重开一场 (10ms 调一次, 在 Kin_Update 开头) */
void Mission_ColorTick(void)
{
    if (!mission_color.clr) return;
    mission_color.clr = 0;
    mission_color.cnt = 0;
    mission_color.ok  = 0;
    mission_color.err = 0;
    memset(mission_color.seq, 0, sizeof(mission_color.seq));
}

/* ================= 抓取完成上报 (0x0F PickDone / 0x11 PickColor / 0x10 NextBatch) ================= */
/* 上位机 pick_task: 期望颜色 -> 发 0x08 目标 -> 等 0x09 授权 -> 抓完必须收到 0x0F 才推进到下一件。
   不发就一直卡在 WAIT_PICK_DONE, 期望颜色永远是 1 -> 3/5 等颜色全被跳过。
   0x0F payload 只能是序号 (上位机 int() 解析后和 expected.sequence 比), 发颜色一定卡在第 2 件。
   所以: 0x0F 发序号保证推进, 0x11 发颜色给视觉日志看 (它不消费, 只打一条节流警告)。 */
uint8_t mission_report_pick = 1;   /* 1=上报 (默认); Watch 写 0 整体关闭 (视觉没升级时的兜底) */
uint8_t mission_pick_seq    = 0;   /* 已上报序号 0~6; 决定下一件报几号和何时发 0x10, Watch 可改写对齐 */

/* 一件物料抓完: color = 闭爪那拍锁存的 0x08 color_id。
   color 无效(视觉没发过 0x08) -> 不发也不计件 (手动 cmd=10/12 没接视觉时就是这种情况)。 */
void Mission_ReportPickDone(uint8_t color)
{
    uint8_t seq;
    if (!mission_report_pick) return;
    if (color < 1 || color > 6) return;
    if (mission_pick_seq >= 6) return;              /* 一场只 6 件, 超了不再报 */
    seq = (uint8_t)(mission_pick_seq + 1);
    Serial_SendPickDone(seq);                       /* 0x0F: 序号 1~6 (视觉据此推进) */
    Serial_SendPickColor(color);                    /* 0x11: 本件颜色 (视觉日志里可见) */
    mission_pick_seq = seq;
    if (seq == 3) Serial_SendNextBatch(1);          /* 第一批 3 件抓完 -> 放行第二批 */
}

/* ================= 塔吊参数准备 ================= */
static void Mission_LoadParams(void)     /* 装货: 等视觉0x09授权下探, 自动夹 */
{
    kin_flag.handshake_en  = 1;      /* 到抓取位等 0x09 ok 再闭爪 (物料区探查已在460, 无实际下降) */
    kin_flag.auto_grip  = 1;      /* 到位后自动闭夹爪 */
    kin_flag.auto_release = 0;
    kin_cal.grab_z_mm         = HOIST_HEIGHT_ROTATE; /* 回转平台抓取高 (唯一真值; 物料区探查也在同一高度)  */
    kin_cal.grab_y_offset_mm  = 0.0f;  /* 装货视觉 y 补偿 (实测 3cm) */
}

static void Mission_ReloadParams(void)   /* 回装: 不握手, 直接降到环上抓, 自动夹 */
{
    kin_flag.handshake_en  = 0;      /* 不等 0x09, 到位直接下探 */
    kin_flag.auto_grip  = 1;
    kin_flag.auto_release = 0;
    kin_cal.grab_z_mm         = kin_cal.reload_grab_hoist_deg;  /* 平地高100 */
    kin_cal.grab_y_offset_mm  = 0.0f;   /* 目标坐标已按卸货落点补好, 不再补 */
    kin_flag.unload_lock       = 1;      /* 回装期间冻结卸货坐标, 防视觉重发 0x0C 中途改坐标 */
}

static void Mission_UnloadParams(float drop_deg)  /* 卸货/堆叠: 平台->环, 自动松, 落高 drop_deg */
{
    kin_flag.auto_release = 1;     /* 到位自动松夹爪 */
    kin_cal.drop_hoist_deg      = drop_deg;
}

/* ================= 按键处理 (IDLE 内) ================= */
static void Mission_KeyIdle(uint8_t k)
{
    if (k && !ms_key_last) { ms_key_press_tick = ms_tick; }   /* 按下沿 */
    if (!k && ms_key_last)                                     /* 松手沿 */
    {
        uint32_t dur = ms_tick - ms_key_press_tick;
        if (dur >= 150)                                       /* >=1.5s: 长按一键启动 */
        {
            Nav_Stop();
            ms_qr_valid = 0;   /* 新一轮, 清旧二维码/底盘锁/直行旋转标志 */
            ms_qr_received = 0;
            mission_color.qr_colors_ok = 0;  /* 颜色组也要重新扫到才可信 */
            mission_color.ok  = 0;           /* 物料颜色对账清零, 一场一统计 */
            mission_color.err = 0;
            mission_color.cnt = 0;           /* 取色件数清零 */
            mission_color.clr = 0;
            memset(mission_color.seq, 0, sizeof(mission_color.seq));   /* 上一场的颜色别留在这 */
            ms_ok_count  = 0;
            ms_err_count = 0;
            ms_count_shown = 0;              /* 结果没上屏前允许再上, 新一场重新允许 */
            mission_pick_seq = 0;            /* 已上报件数从头数 (只管什么时候发 0x10) */
            chassis_flag.lock = 0;
            chassis_flag.qr_scan_active = 0;
            chassis_flag.rotate_active = 0;
            chassis_flag.en = 1;          /* 长按启动: 四轮使能(锁轴) */
            Chassis_ResetHeading();  /* 记录当前 yaw 为车头基准 0°, 清里程计 */
            mission.state = MS_QR_GOTO_MAT_X;
        }
        else                                                  /* 短按: 切换启动区 */
        {
            nav_zone = (nav_zone == 2) ? 1 : 2;
            Screen_ShowZone(nav_zone);
        }
    }
    ms_key_last = k;
}

/* ================= 中点旋转目标 yaw ================= */
static void Mission_RotateAtMid(void)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
    NavPointId next = Nav_NextPoint();
    float nx, ny, target_yaw;

    /* 去工作区: 用该站现场标定的目标朝向; 其他(收车去拐角): atan2 朝下一目标 */
    switch (next)
    {
    case NAV_MATERIAL: target_yaw = z->material_yaw; break;
    case NAV_ROUGH:    target_yaw = z->rough_yaw;    break;
    case NAV_STAGING:  target_yaw = z->staging_yaw;  break;
    default:
        Nav_GetPointCoord(next, &nx, &ny);
        target_yaw = atan2f(ny - z->mid_y, nx - z->mid_x) * (180.0f / MATH_PI);
        break;
    }
    Chassis_RotateTo(target_yaw);
}

/* ================= 工作区操作模式分派 ================= */

/* 当前工作区第一个操作的模式 (粗加工区先卸货, 回装在第2步单独看) */
static uint8_t Mission_FirstOpMode(void)
{
    if (nav_at_point == NAV_MATERIAL) return mission.mode_load;
    if (nav_at_point == NAV_ROUGH)    return mission.mode_unload;
    return mission.mode_stack;   /* NAV_STAGING */
}

/* 手动debug 写完 mission.work_done 后去哪: 卸货手动完还有回装要单独看模式, 其余直接收姿态离站 */
static uint8_t Mission_AfterManual(void)
{
    if (ms_manual_op == MSOP_UNLOAD)   /* 粗加工: 卸货手动完, 回装单独看模式 */
    {
        if (mission.mode_reload == MISSION_OP_SKIP)   return MS_POSE_BOOT;
        if (mission.mode_reload == MISSION_OP_VISION) { Mission_ReloadParams(); return MS_RELOAD; }
        ms_manual_op = MSOP_RELOAD;   /* 回装也是手动: 继续等下一段 */
        return MS_WAIT_MANUAL;
    }
    return MS_POSE_BOOT;   /* 装货/回装/堆叠手动完 -> 收姿态离站 */
}

/* ================= 主状态机 ================= */
void Mission_Update(uint8_t key_pressed)
{
    if (!mission.enable) return;   /* 测试模式: 不介入 */

    ms_tick++;

    /* 收到二维码 -> 解析环序 (后台, 任意状态都收) */
    if (Serial_QRReady) { Serial_QRReady = 0; ms_qr_received = 1; Mission_ParseQR(Serial_RxData.str); }

    /* 手动跳过工作区 (物料/粗加工/暂存): 写 mission.skip_work=1
       -> 跳过当前这个工作区的塔吊作业, 停塔吊、解锁底盘直接离站去下一点 (跳完自动清0)。
       仅在纠偏/作业/收姿态等工作区状态(MS_TRIM~MS_POSE_BOOT)生效, 不影响导航/中点旋转/收车。 */
    if (mission.skip_work && mission.state >= MS_TRIM && mission.state <= MS_POSE_BOOT)
    {
        mission.skip_work = 0;
        Kin_Stop();          /* 立即停塔吊, 不再走当前作业/收姿态 (Kin_Stop 里会解除底盘微调武装) */
        Chassis_TrimDisarm();
        chassis_flag.lock = 0;
        nav_cmd = 3;         /* 解锁底盘, 直接推进导航去下一点 */
        mission.state = MS_NAV;
        return;              /* 本拍不再跑 switch, 防 nav_wait 未清零时重新进入工作区 */
    }

    /* 物料区手动抓取 (Watch 写 mission.mat_grab=1): 不走视觉, 塔吊摆 135/460/200 (已在抓取高, 夹爪张开)
       -> 等执行位 -> 直接闭爪 -> 完整装货 (kin_flag.cmd=10)。
       只在物料区、纠偏已完成(chassis_flag.lock=1, 即车身已定住)、塔吊空闲时执行;
       不满足条件就保留标志, 等到位/空闲再抓, 抓完自动清0。 */
    if (mission.mat_grab && nav_at_point == NAV_MATERIAL && chassis_flag.lock && !kin_flag.active &&
        (mission.state == MS_TRIM || mission.state == MS_WAIT_MANUAL || mission.state == MS_LOAD))
    {
        mission.mat_grab = 0;
        kin_flag.done = 0;           /* 清残留 done, 防上一段手动测试的完成标志被后面的状态误读 */
        kin_flag.mat_grab_cmd = 1;   /* 执行位: 已在抓取高, 收到即直接闭爪 (无下降步) */
        kin_flag.cmd = 10;           /* 物料区固定位完整装货 */
    }

    /* 工作区视觉超时看门狗: 收新视觉指令(0x04/05/08/09/0C)即清零计数;
       塔吊作业状态(MS_TRIM~MS_STACK)超过 mission.vision_timeout 拍(默认1000=10s)
       没收到任何视觉指令 -> 视为当前点动作已完成, 收塔吊默认姿态(134/600/0)离站。 */
    if (Serial_VisionCmd)
    {
        Serial_VisionCmd = 0;
        ms_vision_timeout = 0;
    }
    else if (mission.state != MS_WAIT_MANUAL && mission.state >= MS_TRIM && mission.state <= MS_STACK)
    {
        if (++ms_vision_timeout >= mission.vision_timeout)
        {
            ms_vision_timeout = 0;
            Kin_Stop();              /* 停塔吊, 走收姿态 */
            Chassis_TrimDisarm();    /* 不再等本区微调, 退回自动纠偏 */
            mission.sub = 0;
            ms_err_count++;          /* 超时强制完成 -> 记错 */
            mission.state = MS_POSE_BOOT; /* 与正常完成同一出口: 收运动姿态 310/600/500 -> 解锁离站 */
            return;
        }
    }

    switch (mission.state)
    {
    case MS_BOOT:
        /* 上电引导位姿: 等 Tower_Boot_Task 把塔吊带到运动姿态 310/600/500 (boot_step==2) 后进入等按键 */
        if (boot_step >= 2) { mission.state = MS_IDLE; Screen_ShowZone(nav_zone); }
        break;

    case MS_IDLE:
        Mission_KeyIdle(key_pressed);
        break;

    case MS_QR_GOTO_MAT_X:
        {
            /* 启动区直连物料点(跳过拐角), 两区同流程:
               先去 (±50,-50) 点 -> 沿 ±X 慢扫(扫码/到 ±1500) -> 回物料(X->转->Y) */
            float sign = (nav_zone == 2) ? -1.0f : 1.0f;
            chassis_val.pos_target_x_mm = sign * mission.scan_goto_x;
            chassis_val.pos_target_y_mm = mission.scan_goto_y;
            chassis_flag.pos_active = 1;
            mission.state = MS_QR_SCAN_GOTO_WAIT;
        }
        break;

    case MS_QR_SCAN_GOTO_WAIT:
        if (!chassis_flag.pos_active)
        {
            ms_qr_received = 0;          /* 开始慢扫前清掉历史二维码标志 */
            ms_scan_corner_done = 0;     /* 重置 corner 停车标志 */
            ms_scan_paused = 0;
            mission.state = MS_QR_SCAN_SLOW;  /* 到 (±50,-50) -> 慢扫 */
        }
        break;

    case MS_QR_SCAN_SLOW:
        {
            /* 沿 ±X 慢慢直走扫码: 到 corner 的 x 值停 2s 再继续; 收到二维码或 |x| 到上限就停直回物料 */
            float sign = (nav_zone == 2) ? -1.0f : 1.0f;
            uint8_t hit_stop = (sign > 0) ? (chassis_val.odom_x_mm >= mission.scan_stop_x)
                                          : (chassis_val.odom_x_mm <= -mission.scan_stop_x);
            if (ms_qr_received || hit_stop)
            {
                ms_qr_received = 0;
                ms_scan_paused = 0;              /* 复位暂停 */
                chassis_flag.qr_scan_active = 0;
                NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
                chassis_val.pos_target_x_mm = z->material_x;
                chassis_val.pos_target_y_mm = chassis_val.odom_y_mm;   /* 保持当前 y, 先走 X */
                chassis_flag.pos_active = 1;
                mission.state = MS_QR_GOTO_MAT_X_WAIT;
            }
            else if (ms_scan_paused)
            {
                /* 停在 corner x 等 2s, 到了继续扫 */
                if (ms_tick - ms_scan_pause_tick >= mission.scan_pause_10ms)
                {
                    ms_scan_paused = 0;
                    chassis_flag.qr_scan_active = 1;   /* 继续直行扫码 */
                }
            }
            else
            {
                NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
                uint8_t at_corner = (sign > 0) ? (chassis_val.odom_x_mm >= z->corner_x)
                                               : (chassis_val.odom_x_mm <= z->corner_x);
                if (!ms_scan_corner_done && at_corner)
                {
                    ms_scan_corner_done = 1;
                    ms_scan_paused = 1;
                    ms_scan_pause_tick = ms_tick;
                    chassis_flag.qr_scan_active = 0;   /* 停 2s */
                }
                else if (!chassis_flag.qr_scan_active)
                {
                    chassis_flag.qr_scan_active = 1;   /* 首次进入/停完: 沿车头(±X)慢速直行 */
                }
            }
        }
        break;

    case MS_QR_GOTO_MAT_X_WAIT:
        if (!chassis_flag.pos_active)
        {
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            Chassis_RotateTo(z->material_yaw);   /* X 到位 -> 转 material_yaw */
            mission.state = MS_QR_ROTATE_MAT;
        }
        break;

    case MS_QR_GOTO_MAT_Y_WAIT:
        if (!chassis_flag.pos_active)
        {
            Nav_StartAt(NAV_MATERIAL);   /* Y 到位 -> 以「已到物料区」状态接入全程导航 */
            mission.state = MS_NAV;
        }
        break;

    case MS_QR_ROTATE_MAT:
        if (!chassis_flag.rotate_active)
        {
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            chassis_val.pos_target_x_mm = z->material_x;
            chassis_val.pos_target_y_mm = z->material_y;
            chassis_flag.pos_active = 1;
            mission.state = MS_QR_GOTO_MAT_Y_WAIT;
        }
        break;

    case MS_NAV:
        if (!nav_active) { mission.state = MS_DONE; break; }   /* 全部走完收车 */
        if (nav_wait)
        {
            ms_round = nav_round;   /* 第1轮=0 第2轮=1 (由 nav 按到物料区次数维护) */
            if (nav_at_point == NAV_MID) { Mission_RotateAtMid(); mission.state = MS_ROTATE; }
            else                          { ms_vision_timeout = 0; mission.state = MS_POSE_DEFAULT; }  /* 到点先摆探查姿态 (物料区 135/460/200, 其它 134/600/0) */
        }
        break;

    case MS_ROTATE:
        if (!chassis_flag.rotate_active) { nav_cmd = 3; mission.state = MS_NAV; }   /* 转完 -> 继续导航 */
        break;

    case MS_TRIM:
        /* 探查姿态已就位 -> 放开底盘视觉纠偏, 等一次"新的" within=1 再锁车作业。
           物料区和粗加工/暂存走同一段: 不能靠上一站残留的 trim_within 直接放行;
           视觉在该区根本不发 0x04/0x05 时, 等满 mission.trim_wait_10ms(默认5s) 就放行, 不卡死。 */
        if (mission.sub == 0)
        {
            Chassis_TrimRelease();
            ms_trim_tick = 0;
            mission.sub = 1;
        }
        ms_trim_tick++;
        if ((Chassis_TrimDone() || Chassis_TrimSkipStation(nav_at_point) ||
             ms_trim_tick >= mission.trim_wait_10ms) && !chassis_flag.pos_active)
        {
            chassis_flag.lock = 1;
            mission.sub = 0;
            switch (Mission_FirstOpMode())
            {
            case MISSION_OP_SKIP:
                /* 跳过该工作区塔吊作业: 停塔吊、解锁底盘直接离站 */
                Kin_Stop();
                chassis_flag.lock = 0;
                nav_cmd = 3;
                mission.state = MS_NAV;
                return;
            case MISSION_OP_MANUAL:
                ms_manual_op = (nav_at_point == NAV_MATERIAL) ? MSOP_LOAD :
                               (nav_at_point == NAV_ROUGH)    ? MSOP_UNLOAD : MSOP_STACK;
                mission.state = MS_WAIT_MANUAL;   /* 手动debug: 停到位等你 Watch 驱动(用 Kin_TestRun 或 kin_flag.cmd), 写完 mission.work_done=1 推进 */
                break;
            default:   /* MISSION_OP_VISION: 视觉自动 */
                /* 停靠完成 -> 通知视觉开始放置阶段(0x0D); 物料区是抓取流程不发 */
                if (nav_at_point == NAV_ROUGH)        Serial_SendPlaceControlStart(1, ms_round + 1);
                else if (nav_at_point == NAV_STAGING) Serial_SendPlaceControlStart(2, ms_round + 1);
                if (nav_at_point == NAV_MATERIAL) { Mission_LoadParams();    mission.state = MS_LOAD; }
                else if (nav_at_point == NAV_ROUGH) { Mission_UnloadParams(100.0f);  mission.state = MS_UNLOAD; }
                else { Mission_UnloadParams((ms_round == 0) ? 100.0f : 400.0f); mission.state = MS_STACK; }
                break;
            }
        }
        break;

    case MS_WAIT_MANUAL:
        /* 手动debug 完成: Watch 写 mission.work_done=1 -> 按当前操作决定下一步(卸货后还可能接回装) */
        if (mission.work_done)
        {
            mission.work_done = 0;
            mission.sub = 0;
            mission.state = Mission_AfterManual();
        }
        break;

    case MS_POSE_DEFAULT:
        /* 导航到点停下 -> 先摆探查姿态, 到位后进视觉纠偏;
           物料区用专属 135/460/200 (电机6 预伸出, 夹爪张开就在抓取高), 其它工作区 134/600/0 (夹爪合上清视场)。
           同时武装底盘微调: 姿态期间 hold=1 不收 0x04/0x05, 姿态到位后在 MS_TRIM 放开,
           保证"先摆姿态, 再底盘微调"是新的 within, 不是上一站残留。 */
        if (mission.sub == 0)
        {
            kin_flag.cmd = (nav_at_point == NAV_MATERIAL) ? 9 : 4;
            Chassis_TrimArm();
            mission.sub = 1;
        }
        if (kin_flag.done)
        {
            kin_flag.done = 0;
            mission.sub = 0;
            mission.state = MS_TRIM;
        }
        break;

    case MS_LOAD:
        /* 装货 x3: 视觉发 0x08 坐标 -> 锁存坐标过去 -> 等 0x09 ok -> 自动夹 -> 收平台。
           0x08 可能周期性带抖动, 抓完清标志只认下一个物料的新坐标(旧的 0x08 不管)。 */
        if (mission.sub < 3)
        {
            if (kin_flag.done) { kin_flag.done = 0; mission.sub++; Serial_ColorTargetReady = 0; }   /* 抓+放+回收完成, 清残留抖动帧 */
            if (mission.sub < 3 && !kin_flag.active && Serial_ColorTargetReady)
            {
                Serial_ColorTargetReady = 0;
                kin_flag.cmd = 1;               /* 用 Serial.c 已更新的 kin_tgt.target_x/y 去抓 */
                /* 颜色不在这里记: 由 app_kin 闭夹爪时调 Mission_NoteGrabColor() 统一记,
                   这样手动 cmd=10/12 连抓也能进同一份记录。
                   抓取完成上报(0x0F PickDone / 0x10 NextBatch)也不在这里发:
                   由 app_kin 在"回到探查位"那拍调 Mission_ReportPickDone(闭爪时锁存的颜色) 统一发,
                   mission / cmd=10 / cmd=12 共用一条 (见 app_kin.c KIN_STEP_POSE_HOIST)。 */
            }
        }
        else
        {
            mission.sub = 0;
            ms_ok_count++;            /* 物料区装货完成 -> 对 */
            mission.state = MS_POSE_BOOT;
        }
        break;

    case MS_UNLOAD:
        /* 卸货: 平台抓 -> 按二维码环序依次放 3 个环 (kin_flag.cmd=3 走完), 落高 50 */
        if (mission.sub == 0)
        {
            kin_tgt.drop_ring[0] = Mission_Ring(ms_round, 0);   /* 槽A -> 环 QR[0] */
            kin_tgt.drop_ring[1] = Mission_Ring(ms_round, 1);   /* 槽B -> 环 QR[1] */
            kin_tgt.drop_ring[2] = Mission_Ring(ms_round, 2);   /* 槽C -> 环 QR[2] */
            kin_flag.cmd = 3; mission.sub = 1;   /* Kin_UnloadStart */
        }
        if (kin_flag.done)
        {
            kin_flag.done = 0;
            mission.sub = 0;
            switch (mission.mode_reload)
            {
            case MISSION_OP_SKIP:
                mission.state = MS_POSE_BOOT;   /* 跳过回装, 直接收姿态离站 */
                break;
            case MISSION_OP_MANUAL:
                ms_manual_op = MSOP_RELOAD;
                mission.state = MS_WAIT_MANUAL; /* 回装手动: 等你 Watch 驱动 */
                break;
            default:
                Mission_ReloadParams();
                mission.state = MS_RELOAD;      /* 回装视觉自动 */
                break;
            }
        }
        break;

    case MS_RELOAD:
        /* 回装 x3: 按二维码第2/4组环序从环上抓回放平台 (非固定1->2->3) */
        if (mission.sub < 3)
        {
            if (kin_flag.done) { kin_flag.done = 0; mission.sub++; }
            if (mission.sub < 3 && !kin_flag.active)
            {
                uint8_t ring = Mission_Ring(ms_round, mission.sub);
                kin_tgt.target_x = kin_tgt.unload_x[ring - 1] + kin_cal.unload_x_offset_mm;   /* 与卸货落点同一点 */
                kin_tgt.target_y = kin_tgt.unload_y[ring - 1] - kin_cal.unload_y_offset_mm;
                kin_flag.cmd = 1;
            }
        }
        else
        {
            kin_flag.unload_lock = 0;   /* 回装完恢复坐标更新, 下一站重新收 0x0C */
            mission.sub = 0;
            ms_ok_count++;         /* 粗加工区卸货+回装完成 -> 对 */
            mission.state = MS_POSE_BOOT;
        }
        break;

    case MS_STACK:
        /* 堆叠: 平台抓 -> 按二维码环序依次放 3 个环 (落高已按第1轮50/第2轮400设好) */
        if (mission.sub == 0)
        {
            kin_tgt.drop_ring[0] = Mission_Ring(ms_round, 0);   /* 槽A -> 环 QR[0] */
            kin_tgt.drop_ring[1] = Mission_Ring(ms_round, 1);   /* 槽B -> 环 QR[1] */
            kin_tgt.drop_ring[2] = Mission_Ring(ms_round, 2);   /* 槽C -> 环 QR[2] */
            kin_flag.cmd = 3; mission.sub = 1;
        }
        if (kin_flag.done)
        {
            kin_flag.done = 0;
            mission.sub = 0;
            ms_ok_count++;         /* 暂存区堆叠完成 -> 对 */
            mission.state = MS_POSE_BOOT;
        }
        break;

    case MS_POSE_BOOT:
        /* 作业完成: 塔吊切运动姿态 310/600/500 (舵机310/电机5 600/电机6 500), 到位后解锁底盘离站 */
        if (mission.sub == 0) { Chassis_TrimDisarm(); kin_flag.cmd = 5; mission.sub = 1; }
        if (kin_flag.done)
        {
            kin_flag.done = 0;
            mission.sub = 0;
            chassis_flag.lock = 0;
            nav_cmd = 3;
            mission.state = MS_NAV;
        }
        break;

    case MS_DONE:
        /* 收车完成: t1 显示"物料颜色对账"结果 OK=对上的件数 / ERR=对不上的件数 (只发一次)。
           口径: 每采用一帧 0x08 去抓算一件, 共 MISSION_ITEM_TOTAL 件(2 轮 x 3 件)。
           原来的工作区动作统计仍在 ms_ok_count/ms_err_count 里, Watch 可看, 不再上屏。 */
        {
            if (!ms_count_shown)
            {
                ms_count_shown = 1;
                Screen_ShowCount(mission_color.ok, mission_color.err, mission_color.total);
            }
        }
        break;
    }
}
