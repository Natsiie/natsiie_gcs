#include "sys.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         跑在 DATE_Tasks (10ms)。只做「调度」：读底盘/塔吊/串口状态, 下发 nav_cmd / kin_flag.cmd,
  *         不直接动电机/舵机。mission.enable=0 时完全不介入, 保留原「1功能测试代码」。
  *
  *         赛程 (两个启动区共用, 坐标随 nav_zone):
  *           上电引导位姿 -> 等按键(短按切区/长按启动)
  *           -> 直行到扫码停点(原来"停 2s"那个位置) 停 1s -> 回物料区: 先走 X 再走 Y, 二维码沿路后台扫
  *              (半路就扫到码的话不去停点, 直接回物料区)
  *           -> 全程导航: 从物料点起, 之后每去一个工作区都「中点旋转 -> 工作区作业 -> 回中点」
  *           工作区顺序: 物料(装货) -> 粗加工(卸货+回装) -> 暂存(堆叠), 走两轮。
  *
  *         注意: 底盘移动复用 App_Chassis_Task, 受遥控 sw[0]==2(解锁)且在线 门控
  *               (chassis_flag.need_sw_unlock=1 默认开启, 写 0 旁路, 纯视觉/mission 可不用遥控)。
  */

/* ================= 状态 ================= */
enum {
    MS_BOOT = 0,           /* 上电引导位姿: 夹爪开 -> 平台A -> 塔吊 Tower_Boot_Task 姿态 270/600/550 */
    MS_IDLE,               /* 等按键 (短按切区/长按启动) */
    MS_QR_SCAN_GOTO,       /* 旧: 启动区先到的点 (已不用, 保留占号) */
    MS_QR_GOTO_STOP_WAIT,  /* 启动区: 等开到扫码停点; 半路收到二维码就提前回物料区 */
    MS_QR_STOP_PAUSE,      /* 启动区: 停在扫码停点等 scan_pause_10ms(1s), 停完直接回物料区 */
    MS_QR_GOTO_STOP,       /* 启动区: 直接开到扫码停点 (±corner_x, scan_goto_y) = 原来那个"停 2s"的位置 */
    MS_QR_GOTO_MAT_X_WAIT, /* 等 X 到位 -> 转 material_yaw */
    MS_QR_GOTO_MAT_Y_WAIT, /* 等 Y 到位 */
    MS_QR_ROTATE_MAT,      /* 转 material_yaw (X->转->Y) */
    MS_NAV,                /* 全程导航运行中, 等 nav_wait */
    MS_ROTATE,             /* 中点原地旋转 */
    MS_TRIM,               /* 工作区视觉纠偏 (姿态到位后放开, 等一帧新 within=1 或超时后锁底盘) */
    MS_WAIT_MANUAL,        /* 手动模式: 塔吊作业用 Kin_TestRun, 等 Watch 写 mission.work_done=1 */
    MS_POSE_DEFAULT,       /* 塔吊收探查姿态: 物料区 134/460/0 (微调后再伸到 260), 其它 134/600/0 (清视觉视场) */
    MS_LOAD,               /* 装货 (物料区) */
    MS_UNLOAD,             /* 卸货 (粗加工: 平台 -> 环, 落高 HOIST_HEIGHT_DROP_GROUND, 按二维码环序) */
    MS_RELOAD,             /* 回装 (粗加工: 环 -> 平台, 按二维码环序) */
    MS_STACK,              /* 堆叠 (暂存: 平台 -> 环, 落高 第1轮 DROP_GROUND / 第2轮 PALLET, 按二维码环序) */
    MS_POSE_BOOT,          /* 塔吊收运动姿态 310/600/500 (离站前) */
    MS_DONE,               /* 收车完成 */
    MS_RESTART_POSE,       /* 调试重启: 等塔吊回引导姿态 (Kin_BootPosture) 后回 IDLE (追加在最后, 不改旧值) */
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
    .scan_goto_y      = -200.0f,            /* 扫码停点的 Y (mm), 两区共用; 停点 X 取 nav 表的 corner_x */
    .vision_timeout   = 1000,               /* 工作区视觉超时(10ms/拍), 1000=10s */
    .trim_wait_10ms   = MISSION_TRIM_WAIT_10MS,  /* 进站等底盘微调 within 上限 5s, 等不到放行 */
    .scan_pause_10ms  = 100,                /* 停在扫码停点的时长(10ms/拍), 100=1s */
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
/* 任务码(0x06)内容 + 到达时刻: 用来判断"视觉开新任务了"(见 Mission_ParseQR)。
   视觉的任务码每 1s 重发一次持续 30s, 所以只能按内容/间隔判定, 不能一见 0x06 就重置。 */
static char     ms_qr_code[13];          /* 上次接受的任务码 (12 位数字) */
static uint8_t  ms_qr_code_valid = 0;
static uint32_t ms_qr_code_tick  = 0;    /* 上次收到任务码的拍 */
static uint16_t ms_vision_timeout = 0;  /* 视觉超时计数(10ms/拍), 收新视觉指令清零 */
static uint16_t ms_trim_tick = 0;       /* 进站等底盘微调 within 的计时 (10ms/拍), MS_TRIM 里累加 */
static uint8_t  ms_ok_count  = 0;        /* 完成正确的工作区动作数 (正常走完 -> 对) */
static uint8_t  ms_err_count = 0;        /* 视觉超时/失败的工作区动作数 (超时强制完成 -> 错) */
#define MISSION_WORK_TOTAL 6             /* 总工作区动作数: 2 轮 x 3 区(物料/粗加工/暂存) */

/* 启动区扫码停点: 停在 (±corner_x, mission.scan_goto_y) 等 scan_pause_10ms */
static uint32_t ms_scan_pause_tick  = 0; /* 停在扫码停点的起始拍 (ms_tick) */

static uint32_t ms_tick  = 0;
static uint16_t ms_idle_tick = 0;        /* IDLE 里刷电池电压的计时(10ms/拍), 100=1s */
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
void Screen_ShowZoneBattery(uint8_t zone, float volts);
void Screen_Clear(void);

/* ================= 二维码解析 ================= */
/* "156+231+516+312" -> 4组3位: 第2/4组(d[3..5]/d[9..11])作环序, 第1/3组(d[0..2]/d[6..8])作颜色顺序 */
static void Mission_ParseQR(const char *s)
{
    uint8_t d[12], n = 0, i, colors_ok = 1;
    char code[13];
    while (*s && n < 12)
    {
        if (*s >= '0' && *s <= '9') d[n++] = (uint8_t)(*s - '0');
        s++;
    }
    if (n != 12) return;

    /* 视觉开新任务了 -> 把 0x0F 序号(和颜色对账)清零。
       视觉的序号是「每个任务码从 1 数到 6」(pick_task expected.sequence), 而 mission_pick_seq
       是跨任务累加的, 只有长按一键启动才清 0。视觉重启任务节点开新任务时车不一定长按过,
       序号就会接着上一场数: 视觉等 1, 车报 4 -> 它 complete() 比对失败, 卡在等 0x0F 不推进,
       车在等 0x09, 双方死锁。这里按"新任务码"对齐两边的计数。
       判定必须看内容: 视觉任务码每 1s 重发一次持续 30s, 一收到就清会把正在跑的任务一秒清一次。
       同一个码重跑也算新任务 —— 长按启动时 ms_qr_code_valid 会清 0, 另外间隔超过
       MISSION_QR_NEW_TASK_GAP_10MS 也当新任务。 */
    for (i = 0; i < 12; i++) code[i] = (char)('0' + d[i]);
    code[12] = '\0';
    if (!ms_qr_code_valid || strcmp(code, ms_qr_code) != 0 ||
        (uint32_t)(ms_tick - ms_qr_code_tick) > MISSION_QR_NEW_TASK_GAP_10MS)
    {
        mission_pick_seq = 0;                    /* 已上报件数从头数 */
        mission_color.ok  = 0;
        mission_color.err = 0;
        mission_color.cnt = 0;
        mission_color.clr = 0;
        memset(mission_color.seq, 0, sizeof(mission_color.seq));
        ms_ok_count  = 0;
        ms_err_count = 0;
    }
    strcpy(ms_qr_code, code);
    ms_qr_code_valid = 1;
    ms_qr_code_tick  = ms_tick;

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

/* 测试用 (kin_flag.cmd=15/16/17 台架整链): 按二维码把 3 个平台槽位的落环填进 kin_tgt.drop_ring。
   槽A/B/C 就是物料区的抓取顺序, 对应二维码第 1/2/3 件; round 0=第1轮(第2组数字) 1=第2轮(第4组)。
   cmd=15 由 Watch 的 kin_test_qr_round 选轮次; cmd=16(平地)/cmd=17(码垛) 在 app_kin 里自动绑定
   第1轮/第2轮。没扫到码时填 1/2/3 (drop_ring 默认值), 不阻断测试。mission 自己的
   MS_UNLOAD/MS_STACK 是直接赋值, 不走这里。 */
void Mission_ApplyQRDropRing(uint8_t round)
{
    uint8_t i;
    for (i = 0; i < 3; i++)
        kin_tgt.drop_ring[i] = ms_qr_valid ? Mission_Ring(round, i) : (uint8_t)(i + 1);
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
   序号必须无条件报: 视觉对一件物料只发一次 0x08 (发完立刻进 WAIT_PICK_DONE, 之后不再补发),
   那一帧要是丢了/坏了, color 就是 0。原来 color 无效直接 return 不报 -> 视觉永远等 0x0F
   (且本件 grab_cmd_sent 已置位, 不会再发 0x09), 车永远等 0x09, 两边互锁。
   现在: 0x0F 照发序号 (它本来就是视觉唯一消费的东西), 只有 color 有效才补发 0x13 给视觉日志看;
   颜色对账 (Mission_NoteGrabColor) 自己会丢掉无效颜色, 不用在这拦。 */
void Mission_ReportPickDone(uint8_t color)
{
    uint8_t seq;
    if (!mission_report_pick) return;
    if (mission_pick_seq >= 6) return;              /* 一场只 6 件, 超了不再报 */
    seq = (uint8_t)(mission_pick_seq + 1);
    Serial_SendPickDone(seq);                       /* 0x0F: 序号 1~6 (视觉据此推进) */
    if (color >= 1 && color <= 6)
        Serial_SendPickColor(color);                /* 0x13: 本件颜色 (视觉日志里可见; 0x11 已给 AlignDone) */
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

/* 清空视觉这一路给过的所有信息 (运行中重启/整场重来时调):
   二维码原文 + 解析结果、颜色目标、抓取授权、环工位纠偏、卸货坐标、转盘数据,
   以及各"新帧"标志, 防止上一场的残留被新一场消费。
   注意: grab_cmd_cnt / Serial_RxData.type 是 Watch 诊断量, 不清。
   视觉节点是常开的, 清掉后它 1s 内会把二维码等重发过来, 不影响新一轮。 */
static void Mission_VisionReset(void)
{
    Serial_RxData.str[0] = '\0';                 /* 0x06 二维码原文 */
    Serial_RxData.target_color_id = 0;           /* 0x08 颜色目标点 */
    Serial_RxData.target_x_mm = 0.0f;
    Serial_RxData.target_y_mm = 0.0f;
    Serial_RxData.target_z_mm = 0.0f;
    Serial_RxData.grab_cmd = 0;                  /* 0x09 抓取授权 */
    grab_cmd = 0;                                /* 塔吊侧同一授权的副本 */
    Serial_RxData.ring_err_x  = 0.0f;            /* 0x04/0x05 环工位偏移 + 纠偏 + within */
    Serial_RxData.ring_err_y  = 0.0f;
    Serial_RxData.ring_ang_err = 0.0f;
    Serial_RxData.ring_corr_x = 0.0f;
    Serial_RxData.ring_corr_y = 0.0f;
    Serial_RxData.ring_corr_yaw = 0.0f;
    Serial_RxData.ring_within = 0;
    Serial_RxData.unload_count = 0;              /* 0x0C 卸货落点 */
    memset(Serial_RxData.unload_x, 0, sizeof(Serial_RxData.unload_x));
    memset(Serial_RxData.unload_y, 0, sizeof(Serial_RxData.unload_y));
    memset(Serial_RxData.color_id, 0, sizeof(Serial_RxData.color_id));
    memset(Serial_RxData.color_cx, 0, sizeof(Serial_RxData.color_cx));
    memset(Serial_RxData.color_cy, 0, sizeof(Serial_RxData.color_cy));
    Serial_RxData.color_count = 0;               /* 0x01 色块像素列表 */
    Serial_RxData.motion_state = 0;              /* 0x02 转盘运动 */
    Serial_RxData.motion_speed = 0.0f;
    Serial_RxData.turntable_cx = 0.0f;           /* 0x03 转盘圆心 */
    Serial_RxData.turntable_cy = 0.0f;
    Serial_RxData.turntable_r  = 0.0f;
    Serial_RxData.turntable_stable = 0;

    Serial_QRReady = 0;                          /* 待解析/待消费标志全清 */
    Serial_ColorTargetReady = 0;
    Serial_UnloadCoordReady = 0;
    Serial_ChassisTrimReady = 0;
    Serial_TurntableTrimReady = 0;
    Serial_VisionCmd = 0;

    kin_flag.unload_have = 0;                    /* 塔吊里锁存的 0x0C 落点也作废 */
}

/* ================= 一整场重来: 清状态 (长按启动 / 运行中重启 共用) =================
   目标: 新一场和刚上电跑第一场完全一样。这里只清"这一场跑出来的东西",
   标定/整定/启停区选择(nav_zone)/视觉节点本身都不动。
   不动塔吊/底盘/里程计 —— 那三样由调用方按场景处理 (重启要停电机+回引导姿态,
   启动则本来就是停着的)。 */
static void Mission_RunReset(void)
{
    Mission_VisionReset();   /* 视觉给过的信息(含二维码)全作废, 新一场重新收 */
    Kin_ResetRun();          /* 塔吊: 抓取授权/落点环序/内部步全清 (见 app_kin.c) */
    Nav_ResetRun();          /* 导航: 停 + 轮次/计数/Watch 待触发清 (见 app_nav.c) */

    ms_qr_valid = 0;   /* 新一轮, 清旧二维码/底盘锁/直行旋转标志 */
    ms_qr_received = 0;
    ms_qr_code_valid = 0;  /* 同一个任务码重跑也要当新任务, 下次收到码重新对齐 0x0F 序号 */
    ms_qr_code[0] = '\0';
    /* 二维码解析结果退回上电默认 (1/2/3): 新码没到之前不沿用上一场的环序/颜色组。
       mission 的 MS_UNLOAD/MS_STACK 直接用 Mission_Ring() 赋值, 不看 ms_qr_valid,
       所以这两组数必须自己清, 否则新一场会按上一场的码放环。 */
    ms_qr_rings[0][0] = 1; ms_qr_rings[0][1] = 2; ms_qr_rings[0][2] = 3;
    ms_qr_rings[1][0] = 1; ms_qr_rings[1][1] = 2; ms_qr_rings[1][2] = 3;
    mission_color.qr_colors_ok = 0;  /* 颜色组也要重新扫到才可信 */
    mission_color.expected[0][0] = 1; mission_color.expected[0][1] = 2; mission_color.expected[0][2] = 3;
    mission_color.expected[1][0] = 1; mission_color.expected[1][1] = 2; mission_color.expected[1][2] = 3;
    mission_color.ok  = 0;           /* 物料颜色对账清零, 一场一统计 */
    mission_color.err = 0;
    mission_color.cnt = 0;           /* 取色件数清零 */
    mission_color.clr = 0;
    memset(mission_color.seq, 0, sizeof(mission_color.seq));   /* 上一场的颜色别留在这 */
    ms_ok_count  = 0;
    ms_err_count = 0;
    ms_count_shown = 0;              /* 结果没上屏前允许再上, 新一场重新允许 */
    mission_pick_seq = 0;            /* 已上报件数从头数 (只管什么时候发 0x10) */
    ms_round     = 0;                /* 从第 1 轮重新开始 (第 2 轮落高/环序不同) */
    chassis_flag.lock = 0;
    chassis_flag.qr_scan_active = 0;
    chassis_flag.rotate_active = 0;
    chassis_cfg.pos_min_speed = 0.0f;   /* 上一场可能停在"启动区巡航"那一段, 速度下限别留给后面的点 */
    /* 状态机残留 (运行中重启回来时最要紧; 从 IDLE 启动时本来就是干净的) */
    mission.sub         = 0;
    mission.work_done   = 0;
    mission.skip_work   = 0;
    mission.mat_grab    = 0;
    mission.restart     = 0;   /* 待处理的 Watch 重启请求也作废, 别在新一场里再触发一次 */
    ms_vision_timeout   = 0;
    ms_trim_tick        = 0;
    mission.done_at_pose   = 0;   /* 诊断: 新一场从干净开始数 */
    mission.pose_early_cnt = 0;
    ms_manual_op        = MSOP_LOAD;
    ms_scan_pause_tick  = 0;
}

/* 运行中调试重启 (长按一键键 / Watch 写 mission.restart=1):
   前提是操作员已经把车搬回启动区。停底盘并四轮失能(松轴, 方便人手搬车) -> 停塔吊 -> 封死纠偏
   -> 清整场状态 -> 里程计清零 -> 塔吊回引导姿态 -> MS_RESTART_POSE 等姿态到位后回 IDLE 重新一键启动。
   下次 IDLE 长按启动会 chassis_flag.en=1 重新锁轴。 */
static void Mission_Restart(void)
{
    /* 顺序要紧: 先物理停 (停底盘/停塔吊), 再清状态, 最后下发引导姿态。
       Kin_BootPosture 放在 Mission_RunReset 之后, 否则 Kin_ResetRun 会把这条姿态命令清掉。 */
    Chassis_DisableMotors();    /* 四轮立即失能(松轴): 车要靠人手搬回启动区, 轮子不能锁着 */
    Kin_Stop();                 /* 停电机5/6 + 清卸/回装链 + 解除底盘微调武装 */
    Chassis_TrimBlock();        /* 清视觉纠偏就绪标志: 到下一站 TrimArm 前一帧不收, 防行车中被拽 */
    Mission_RunReset();         /* 内含 Nav_ResetRun/Kin_ResetRun: 导航轮次、塔吊 cmd/done、整场统计全清 */
    Screen_Clear();             /* 串口屏也清: 上一场的任务码/次数不能还留在屏上。回 IDLE 后 t1 会刷成 ZONE+电量 */
    Chassis_ResetHeading();     /* 搬到启动区了: 当前 yaw 记 0°, 里程计清零 */
    Kin_BootPosture();          /* 夹爪开 + 平台A + 和 Tower_Boot_Task 同一姿态 270/600/550 */
    mission.state = MS_RESTART_POSE;
}

/* ================= 按键处理 (所有状态都扫) =================
   IDLE 里: 长按=启动, 短按=切启动区 (同原来)。
   运行中: 长按=中止回 IDLE (车要已被搬回启动区), 短按忽略 (防误触切区)。 */
static void Mission_KeyScan(uint8_t k)
{
    if (k && !ms_key_last) { ms_key_press_tick = ms_tick; }   /* 按下沿 */
    if (!k && ms_key_last)                                     /* 松手沿 */
    {
        uint32_t dur = ms_tick - ms_key_press_tick;
        if (dur >= 150)                                       /* >=1.5s: 长按 */
        {
            if (mission.state == MS_IDLE)
            {
                Mission_RunReset();           /* 含 Nav_ResetRun/Kin_ResetRun: 和刚上电跑第一场一致 */
                Screen_Clear();               /* 屏也回到上电占位: 别让上一场的任务码/次数挂到新一场 */
                chassis_flag.en = 1;          /* 长按启动: 四轮使能(锁轴) */
                Chassis_ResetHeading();  /* 记录当前 yaw 为车头基准 0°, 清里程计 */
                mission.state = MS_QR_GOTO_STOP;
            }
            else if (mission.state == MS_RESTART_POSE)
            {
                /* 塔吊回姿态卡住时的逃生: 再长按一次直接回 IDLE */
                mission.state = MS_IDLE;
                ms_idle_tick = 0;
                Screen_ShowZoneBattery(nav_zone, chassis_val.vbus);
            }
            else if (mission.state != MS_BOOT)
            {
                Mission_Restart();            /* 运行中: 中止回 IDLE */
            }
        }
        else if (mission.state == MS_IDLE)    /* 短按: 切换启动区 */
        {
            nav_zone = (nav_zone == 2) ? 1 : 2;
            Screen_ShowZoneBattery(nav_zone, chassis_val.vbus);
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

/* ================= 放置完成回传 (0x0B) ================= */
/* 2026-11-09 视觉改成 0x0C 批量模型 (origin/main bba2e1d) 后 0x0B 的含义变了:
 *   - 识别源不再靠 0x0B 切: pixel_to_base 改为收到 0x11 AlignDone (stage2=三圆环 /
 *     stage3=叠料) 才切源并累积 5 帧像素均值, 然后只发一次 0x0C;
 *   - race_supervisor 把"本阶段一次成功 0x0B"当成整批放置完成 (placements_per_phase=1)。
 * 所以到站时一帧 0x0B 都不能发: 早发会被当成整批完成, 视觉提前切下一个识别源,
 * 正在累积的 5 帧被打断 -> 0x0C 永远不来 (塔吊卡在等落点)。只在整批卸/放完那一刻回。
 * task_id 必须是 32 位小写 hex (视觉端严格校验), 用 ms_tick/ms_round 拼, 每次到站都不同。 */
static char ms_place_task_id[33];

/* 只换新 task_id, 不发帧 (到站时用) */
void Mission_PlaceFeedNewTask(void)
{
    static const char hex[] = "0123456789abcdef";
    uint8_t i;
    for (i = 0; i < 32; i++) ms_place_task_id[i] = '0';
    ms_place_task_id[32] = '\0';
    for (i = 0; i < 8; i++)                      /* 低 32 位写 ms_tick (10ms/拍, 单调) */
        ms_place_task_id[31 - i] = hex[(ms_tick >> (4 * i)) & 0x0F];
    ms_place_task_id[22] = hex[ms_round & 0x0F]; /* 第几轮 */
}

/* 旧接口 (到站发 1 帧): 新视觉下会打断 0x0C 累积, 不要再用, 留作台架/回退排查 */
void Mission_PlaceFeedFirstLayer(void)
{
    Mission_PlaceFeedNewTask();
    Serial_SendPlaceResultRaw(ms_place_task_id, 1, 1);
}

/* 整批卸/放完成: 发 2 帧成功. 第一条就够 supervisor 计满; 第二条冗余, 防第一条丢帧 */
void Mission_PlaceFeedDone(void)
{
    if (ms_place_task_id[0] == '\0') Mission_PlaceFeedNewTask();
    Serial_SendPlaceResultRaw(ms_place_task_id, 2, 1);
    Serial_SendPlaceResultRaw(ms_place_task_id, 3, 1);
}

/* 旧接口 (台架 cmd16/17 切叠料源): 新视觉的识别源由 AlignDone stage3 决定, 不要再调用 */
void Mission_PlaceFeedStacked(void)
{
    Mission_PlaceFeedNewTask();
    Serial_SendPlaceResultRaw(ms_place_task_id, 2, 1);
    Serial_SendPlaceResultRaw(ms_place_task_id, 3, 1);
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

/* 微调(物料区还要等电机6 伸到抓取预备位 260)全部做完 -> 按当前工作区的操作模式进入作业 */
static void Mission_EnterWork(void)
{
    switch (Mission_FirstOpMode())
    {
    case MISSION_OP_SKIP:
        /* 跳过该工作区塔吊作业: 停塔吊、封死本站纠偏、解锁底盘直接离站。
           必须封: 武装(trim_armed)还挂着的话, 行车途中转盘节点(常开)来一帧 0x0E within=1,
           就会把已经开走的车拽回那个点。下一次进站的 Chassis_TrimArm() 自动解封。 */
        Kin_Stop();
        Chassis_TrimBlock();
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
        /* 落高统一取 car_config.h (现场标定唯一真值), 不再写死 100/400:
           粗加工 = 平地卸货; 暂存第1轮 = 平地, 第2轮 = 码垛 */
        else if (nav_at_point == NAV_ROUGH) { Mission_UnloadParams(HOIST_HEIGHT_DROP_GROUND); mission.state = MS_UNLOAD; }
        else { Mission_UnloadParams((ms_round == 0) ? HOIST_HEIGHT_DROP_GROUND : HOIST_HEIGHT_PALLET); mission.state = MS_STACK; }
        break;
    }
}

/* ================= 启动区扫码结束: 回物料区 ================= */
/* 三个入口共用 (半路扫到码 / 停点 1s 停完 / 停在停点时扫到码):
   清扫码标志 -> 保持当前 y 先走 X 回 material_x -> 转 material_yaw -> 走 Y。
   2026-11-09 改法: 不再继续往前扫到 ±1500; 停点之后不管有没有收到码都直接回。 */
static void Mission_LeaveScanStop(void)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
    ms_qr_received = 0;
    chassis_flag.qr_scan_active = 0;   /* 保险: 旧的扫码直行标志不要留着 */
    chassis_val.pos_target_x_mm = z->material_x;
    chassis_val.pos_target_y_mm = chassis_val.odom_y_mm;   /* 保持当前 y, 先走 X */
    chassis_flag.pos_active = 1;
    mission.state = MS_QR_GOTO_MAT_X_WAIT;
}

/* ================= 定位目标"暂停"恢复 ================= */
/* App_Chassis_Task 里遥控门控(need_sw_unlock: CH5 不在解锁位 或 接收机失联)会把
   正在跑的定位从 1 改成 2(暂停), 并停 4 轮。锁着时底盘每拍都会按回 2、轮子不动(安全),
   解锁后要有人把 2 改回 1 才会继续 —— 全程导航里有这句 (app_nav.c Nav_Update),
   启动区这段(nav_active=0)没有, 长按启动那一刻若遥控器没解锁/接收机还没出帧,
   定位目标就被永久暂停, 之后解锁也永远不动。三个等定位的状态里各补一次。 */
static void Mission_PosResume(void)
{
    if (chassis_flag.pos_active == 2) chassis_flag.pos_active = 1;
}

/* ================= 主状态机 ================= */
void Mission_Update(uint8_t key_pressed)
{
    /* 二维码 0x06 解析 + 计时放在 enable 判断之前: 台架用 kin_flag.cmd=16/17 测暂存区时
       mission.enable=0, 但落环顺序要按二维码第1轮(平地)/第2轮(码垛)来取, 所以测试模式也得收码
       (Mission_ApplyQRDropRing 读的就是这里填的 ms_qr_rings)。ms_tick 一起走, 否则
       "新任务码间隔"判断在测试模式下永远是 0。点灯/状态机部分仍然 mission.enable=0 时不动。 */
    ms_tick++;

    /* 收到二维码 -> 解析环序 (后台, 任意状态都收) */
    if (Serial_QRReady) { Serial_QRReady = 0; ms_qr_received = 1; Mission_ParseQR(Serial_RxData.str); }

    if (!mission.enable) return;   /* 测试模式: 不介入 */

    /* Watch 调试重启: mission.restart 写 1 -> 等同运行中长按一键键 (中止回 IDLE, 重启完自动清0)。
       前提同长按: 车已被搬回启动区。IDLE/回姿态/上电引导里写 1 无动作。 */
    if (mission.restart)
    {
        mission.restart = 0;
        if (mission.state != MS_IDLE && mission.state != MS_RESTART_POSE && mission.state != MS_BOOT)
            Mission_Restart();
    }

    /* 一键键所有状态都扫: IDLE 长按=启动 / 短按=切区; 运行中长按=中止回 IDLE (短按忽略) */
    Mission_KeyScan(key_pressed);

    /* 手动跳过工作区 (物料/粗加工/暂存): 写 mission.skip_work=1
       -> 跳过当前这个工作区的塔吊作业, 停塔吊、解锁底盘直接离站去下一点 (跳完自动清0)。
       仅在纠偏/作业/收姿态等工作区状态(MS_TRIM~MS_POSE_BOOT)生效, 不影响导航/中点旋转/收车。 */
    if (mission.skip_work && mission.state >= MS_TRIM && mission.state <= MS_POSE_BOOT)
    {
        mission.skip_work = 0;
        Kin_Stop();          /* 立即停塔吊, 不再走当前作业/收姿态 (Kin_Stop 里会解除底盘微调武装) */
        Chassis_TrimBlock(); /* 马上要开走: 本站纠偏封死, 防行车途中一帧 0x0E/0x05 把车拽回这个点 */
        chassis_flag.lock = 0;
        nav_cmd = 3;         /* 解锁底盘, 直接推进导航去下一点 */
        mission.state = MS_NAV;
        return;              /* 本拍不再跑 switch, 防 nav_wait 未清零时重新进入工作区 */
    }

    /* 物料区手动抓取 (Watch 写 mission.mat_grab=1): 不走视觉, 塔吊摆 134/460/260 (已在抓取高, 夹爪张开)
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
       没收到任何视觉指令 -> 视为当前点动作已完成, 收塔吊默认姿态(134/600/0)离站。
       2026-10-31 临时关闭超时放行: 只计数(Watch 看多久没视觉), 不再自动收姿态离站。
       要恢复就把下面注释掉的 if 块原样放回。 */
    if (Serial_VisionCmd)
    {
        Serial_VisionCmd = 0;
        ms_vision_timeout = 0;
    }
    else if (mission.state != MS_WAIT_MANUAL && mission.state >= MS_TRIM && mission.state <= MS_STACK)
    {
        ++ms_vision_timeout;   /* 只计数, 不参与判断 */
        (void)ms_err_count;    /* 超时放行关掉后它暂时没有写入方; 给编译器一个"已使用", 恢复 if 块时删掉这行 */
        /*
        if (ms_vision_timeout >= mission.vision_timeout)
        {
            ms_vision_timeout = 0;
            Kin_Stop();
            Chassis_TrimBlock();
            mission.sub = 0;
            ms_err_count++;
            mission.state = MS_POSE_BOOT;
            return;
        }
        */
    }

    switch (mission.state)
    {
    case MS_BOOT:
        /* 上电引导位姿: 等 Tower_Boot_Task 把塔吊带到引导姿态 270/600/550 (boot_step==2) 后进入等按键 */
        if (boot_step >= 2) { mission.state = MS_IDLE; ms_idle_tick = 0; Screen_ShowZoneBattery(nav_zone, chassis_val.vbus); }
        break;

    case MS_IDLE:
        /* 按键在 Mission_Update 顶部统一扫 (Mission_KeyScan), 这里不用再处理 */
        /* 电量: 写在 t1 启动区后面 "ZONE:1 24.3V", 空格隔开, 只在 IDLE(等长按启动)期间刷,
           1s 一次; 启动后 t1 不再刷, t0/t2 的二维码不受影响。
           进 IDLE 那拍 ms_idle_tick=0 -> 立即刷一次, 之后每约 1s 刷一次 */
        if (ms_idle_tick == 0)
        {
            ms_idle_tick = 1;
            Screen_ShowZoneBattery(nav_zone, chassis_val.vbus);
        }
        else if (++ms_idle_tick >= 100)
        {
            ms_idle_tick = 1;
            Screen_ShowZoneBattery(nav_zone, chassis_val.vbus);
        }
        break;

    case MS_QR_GOTO_STOP:
        {
            /* 启动区: 一脚直接开到扫码停点 (原来那个"停 2s"的位置), 两区同流程, 方向随 nav_zone 镜像。
               停点 = (±corner_x, mission.scan_goto_y):
                 corner_x 就是原来慢扫判"到 corner"用的那个 x (nav 表, 1070/-1070);
                 y 沿用原来的扫码车道 scan_goto_y, 保证摄像头对着的还是二维码那一带。
               这一段必须正常减速到点 (pos_min_speed=0): 目的地就是要停住, 不再"不停车通过"。 */
            float sign = (nav_zone == 2) ? -1.0f : 1.0f;
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            chassis_val.pos_target_x_mm = sign * z->corner_x;
            chassis_val.pos_target_y_mm = mission.scan_goto_y;
            chassis_cfg.pos_min_speed = 0.0f;
            chassis_flag.pos_active = 1;
            ms_qr_received = 0;          /* 只认出发之后收到的二维码 */
            mission.state = MS_QR_GOTO_STOP_WAIT;
        }
        break;

    case MS_QR_GOTO_STOP_WAIT:
        Mission_PosResume();   /* 被遥控门控暂停过也要能继续 (长按那一刻遥控器还没解锁) */
        if (ms_qr_received)
        {
            Mission_LeaveScanStop();   /* 半路扫到码: 不用去停点了, 直接回物料区 */
        }
        else if (!chassis_flag.pos_active)   /* 到停点 -> 停 scan_pause_10ms (1s) */
        {
            ms_scan_pause_tick = ms_tick;
            mission.state = MS_QR_STOP_PAUSE;
        }
        break;

    case MS_QR_STOP_PAUSE:
        /* 停在扫码停点等 1s; 停完直接回物料区, 不再继续往前扫
           (码没扫到也回, 环序用默认 1/2/3, 后台收到码照样解析) */
        if (ms_qr_received || (ms_tick - ms_scan_pause_tick >= mission.scan_pause_10ms))
        {
            Mission_LeaveScanStop();
        }
        break;

    case MS_QR_GOTO_MAT_X_WAIT:
        Mission_PosResume();
        if (!chassis_flag.pos_active)
        {
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            Chassis_RotateTo(z->material_yaw);   /* X 到位 -> 转 material_yaw */
            mission.state = MS_QR_ROTATE_MAT;
        }
        break;

    case MS_QR_GOTO_MAT_Y_WAIT:
        Mission_PosResume();
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
            else                          { ms_vision_timeout = 0; mission.state = MS_POSE_DEFAULT; }  /* 到点先摆探查姿态 (物料区 134/460/0 微调后再伸到 260, 其它 134/600/0) */
        }
        break;

    case MS_ROTATE:
        if (!chassis_flag.rotate_active) { nav_cmd = 3; mission.state = MS_NAV; }   /* 转完 -> 继续导航 */
        break;

    case MS_TRIM:
        /* 探查姿态已就位 -> 放开底盘视觉纠偏, 等一次"新的" within=1 再锁车作业。
           物料区和粗加工/暂存走同一段: 不能靠上一站残留的 trim_within 直接放行;
           2026-10-31 临时关闭超时兜底: 只认 Chassis_TrimDone() (视觉 within 或 底盘自判 trim_conv),
           等不到就一直等。原来"等满 mission.trim_wait_10ms(5s) 就放行"的两项已注释掉。
           物料区多一段: 微调时电机6 是收着的 (134/460/0, 别挡转盘/物料视场),
           微调完只把电机6 从 0 伸到 260 (云台/电机5 不动, 都在 134/460 上);
           伸到位再进作业, 免得微调把伸出去的电机6 一起带着晃。 */
        Mission_PosResume();   /* 同一道门控: 纠偏途中遥控器锁过一下就变 2, 解锁后不恢复的话永远过不去 */
        if (mission.sub == 0)
        {
            chassis_flag.lock = 0;   /* 姿态已摆好: 松开 MS_POSE_DEFAULT 冻住的底盘, 微调要能动 */
            Chassis_TrimRelease();
            ms_trim_tick = 0;
            mission.sub = 1;
        }
        ms_trim_tick++;   /* 只计时 (Watch 看等了多久), 不再参与放行判断 */
        if (mission.sub == 1)
        {
            uint8_t trim_ok = Chassis_TrimDone();   /* 1=视觉/底盘自判真的纠到位了 (不是跳过本站) */
            if ((trim_ok || Chassis_TrimSkipStation(nav_at_point)
                 /* || ms_trim_tick >= mission.trim_wait_10ms */   /* 超时放行: 临时关 */
                ) && !chassis_flag.pos_active)
            {
                chassis_flag.lock = 1;
                /* 到站坐标矫正 (2026-11-02 用户要求): 粗加工/暂存 视觉微调到底 -> 把里程计坐标
                   直接写成这一点的导航目标。车是被视觉对到真实作业位上的, 所以这一段路攒下的
                   累计漂移到这里整段清零, 后面几段(含"另一个轴锁当前值"的走法)从正确位置起算。
                   函数内部限定: 只粗加工/暂存, 跳过本站纠偏和精度测试都不矫正。 */
                if (trim_ok) Nav_SnapOdomToPoint(nav_at_point);
                if (nav_at_point == NAV_MATERIAL && Mission_FirstOpMode() != MISSION_OP_SKIP)
                {
                    Chassis_TrimBlock();  /* 微调到位, 电机6 要伸出去 -> 本站不再收任何纠偏帧 (挡住摄像头会纠错) */
                    kin_flag.done = 0;
                    kin_flag.cmd  = 14;   /* 微调完 -> 只伸电机6 到 260 (云台/电机5 不动) */
                    ms_trim_tick  = 0;
                    mission.sub   = 2;
                    break;
                }
                mission.sub = 0;
                Mission_EnterWork();
            }
        }
        /* 微调后伸电机6: 只等 kin_flag.done, 超时兜底临时关 (原条件: || ms_trim_tick >= mission.trim_wait_10ms) */
        else if (mission.sub == 2 && kin_flag.done)
        {
            kin_flag.done = 0;   /* 伸出到位 (或超时兜底) -> 进作业 */
            mission.sub = 0;
            Mission_EnterWork();
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
           物料区: 先摆 134/460/0 (电机6 收着, 别挡转盘/物料视场), 微调完在 MS_TRIM 再伸到 260;
           其它工作区: 134/600/0 (夹爪合上清视场)。
           同时武装底盘微调: 姿态期间 hold=1 不收 0x04/0x05/0x0E, 姿态到位后在 MS_TRIM 放开,
           保证"先摆姿态, 再底盘微调"是新的 within, 不是上一站残留。
           姿态期间还 lock=1 把底盘整台冻住 (塔吊甩臂会晃车, 航向保持会拧轮子补) ——
           MS_TRIM 开头松, 微调完再锁回 (2026-11-02)。 */
        if (mission.sub == 0)
        {
            /* 诊断 (只读, 不改流程): 记下这拍 done 的旧值 —— 修好之后如果它还变成 1,
               说明 done 残留确实发生过, 只是下面已经不会再被它带进 MS_TRIM 了。 */
            mission.done_at_pose = kin_flag.done ? 1 : 0;
            if (mission.done_at_pose && mission.pose_early_cnt < 0xFFFF) mission.pose_early_cnt++;

            kin_flag.done = 0;      /* 清残留: 要等的是这条姿态命令自己的 done, 不能是上一段的 */
            kin_flag.cmd = (nav_at_point == NAV_MATERIAL) ? 13 : 4;
            Chassis_TrimArm();
            chassis_flag.lock = 1;  /* 摆姿态期间把底盘冻住: 塔吊抬600->640+云台310->134 的甩臂反力
                                       会把车晃出几度, 不冻的话航向保持会一直拧轮子补 ("到站后又动一下")。
                                       微调要能动底盘, MS_TRIM 开头再松开。 */
            mission.sub = 1;
        }
        else if (kin_flag.done)     /* else if: 发 cmd 那拍不判完成 (必须跨至少一拍) */
        {
            kin_flag.done = 0;
            mission.sub = 0;
            mission.state = MS_TRIM;
        }
        break;

    case MS_LOAD:
        /* 物料区固定位装货 x3 (和 cmd=10 同一条链, 2026-10-31 用户要求: 不跟视觉坐标)。
           站位在 MS_POSE_DEFAULT/MS_TRIM 已经摆好: 134/460/0 -> 底盘微调到位 -> 电机6 伸到 260。
           之后云台/电机5/电机6 都不再为坐标动: 视觉只给两样东西 ——
             0x09 下抓指令 -> 塔吊直接闭爪 -> 升 635 清障 -> 放到自己平台 310/308 -> 回 134/460/260;
             0x08 里的 color_id (颜色指令) -> 闭爪那拍由 app_kin 调 Mission_NoteGrabColor 记对账。
           0x08 的坐标(x,y)不再使用, 收到只清标志, 免得留给下一站/台架测试当新帧。
           一件走完 (kin_flag.done) 自动发下一条 cmd=10, 停回抓取预备位等视觉给下一条 0x09;
           抓满 3 件收姿态离站。抓完反馈(0x0F 序号 + 0x11 颜色)由 app_kin 在回位那拍统一发, 这里不发。 */
        if (mission.sub < 3)
        {
            if (kin_flag.done) { kin_flag.done = 0; mission.sub++; }   /* 本件 抓+放+回位 完成 */
            if (mission.sub < 3 && !kin_flag.active)
            {
                Serial_ColorTargetReady = 0;   /* 固定位装货不用 0x08 坐标: 丢掉新帧标志 (color_id 照常更新) */
                /* 第3件 = 全站最后一件: 放完直接回运动姿态 310/600/500 再进 MS_POSE_BOOT,
                   不再先摆物料区探查姿态 134/460/260 (2026-11-02 用户要求) */
                kin_flag.end_boot = (mission.sub == 2) ? 1 : 0;
                kin_flag.mid_hold = 0;         /* 物料区每件之间必须回 134/460/260 等视觉重看/给 0x09, 不能抬到清障高就停 */
                kin_flag.cmd = 10;             /* 固定位完整装货: 等 0x09 -> 闭爪 -> 放平台 -> 回预备位 */
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
        /* 卸货: 平台抓 -> 按二维码环序依次放 3 个环 (kin_flag.cmd=3 走完), 落高 HOIST_HEIGHT_DROP_GROUND */
        if (mission.sub == 0)
        {
            kin_tgt.drop_ring[0] = Mission_Ring(ms_round, 0);   /* 槽A -> 环 QR[0] */
            kin_tgt.drop_ring[1] = Mission_Ring(ms_round, 1);   /* 槽B -> 环 QR[1] */
            kin_tgt.drop_ring[2] = Mission_Ring(ms_round, 2);   /* 槽C -> 环 QR[2] */
            /* 卸完就离站 (跳过回装) 时, 卸货链的最后一件就是全站最后一件:
               放完直接回运动姿态 310/600/500, 不再先摆探查姿态 134/600/0。
               还要回装 (视觉/手动) 的话这里必须是 0 —— 卸完那一步只是过渡, 不是收工。 */
            kin_flag.end_boot = (mission.mode_reload == MISSION_OP_SKIP) ? 1 : 0;
            /* 后面还有回装 (视觉/手动): 卸完别回探查位 134/600/0, 抬到清障高就交班 ——
               回装的 3 个落点这批 0x0C 已经一次性拿到并锁存, 回去再甩出来是白走 */
            kin_flag.mid_hold = (mission.mode_reload == MISSION_OP_SKIP) ? 0 : 1;
            kin_flag.done = 0;   /* 清残留: 要等的是这条卸货链的 done, 不能是上一段的 */
            /* 到站只换新 task_id, 不发 0x0B: 新视觉的识别源由 0x11 AlignDone stage 决定,
               0x0B 只在整批放完时发一条 (见 Mission_PlaceFeedDone) */
            Mission_PlaceFeedNewTask();
            Kin_SetRegionStage(2);   /* 粗加工区 = 三圆环 */
            kin_flag.cmd = 3; mission.sub = 1;   /* Kin_UnloadStart */
        }
        else if (kin_flag.done)  /* else if: 发 cmd 那拍不判完成 (必须跨至少一拍) */
        {
            kin_flag.done = 0;
            mission.sub = 0;
            /* 卸 3 完成 = 本阶段完成: 视觉 supervisor 要等这条 0x0B 才推进下一阶段 */
            Mission_PlaceFeedDone();
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
                /* 第3个 = 全站最后一件: 放完直接回运动姿态 310/600/500, 不再先摆探查姿态 134/600/0 */
                kin_flag.end_boot = (mission.sub == 2) ? 1 : 0;
                /* 第1/2件: 放完别回探查位, 抬到清障高就交班, 直接从平台转向下一个环 ——
                   三个落点一次性拿到 (回装期间 unload_lock 冻结), 回去看也是白看 */
                kin_flag.mid_hold = (mission.sub == 2) ? 0 : 1;
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
        /* 堆叠: 平台抓 -> 按二维码环序依次放 3 个环 (落高已按第1轮 DROP_GROUND / 第2轮 PALLET 设好) */
        if (mission.sub == 0)
        {
            kin_tgt.drop_ring[0] = Mission_Ring(ms_round, 0);   /* 槽A -> 环 QR[0] */
            kin_tgt.drop_ring[1] = Mission_Ring(ms_round, 1);   /* 槽B -> 环 QR[1] */
            kin_tgt.drop_ring[2] = Mission_Ring(ms_round, 2);   /* 槽C -> 环 QR[2] */
            /* 堆叠是这个区唯一的一步: 放满 3 个就是收工, 直接回运动姿态 310/600/500, 不摆探查姿态 */
            kin_flag.end_boot = 1;
            kin_flag.mid_hold = 0;   /* 放满 3 个就是收工 (end_boot), 不走中途交班 */
            kin_flag.done = 0;   /* 清残留: 要等的是这条堆叠链的 done, 不能是上一段的 */
            /* 到站只换新 task_id, 不发 0x0B (识别源由 AlignDone stage3 切) */
            Mission_PlaceFeedNewTask();
            Kin_SetRegionStage(3);   /* 暂存叠料区 */
            kin_flag.cmd = 3; mission.sub = 1;
        }
        else if (kin_flag.done)  /* else if: 发 cmd 那拍不判完成 (必须跨至少一拍) */
        {
            kin_flag.done = 0;
            mission.sub = 0;
            Mission_PlaceFeedDone();   /* 本阶段堆叠完成 -> 0x0B, 视觉 supervisor 推进下一阶段 */
            ms_ok_count++;         /* 暂存区堆叠完成 -> 对 */
            mission.state = MS_POSE_BOOT;
        }
        break;

    case MS_POSE_BOOT:
        /* 作业完成: 塔吊切运动姿态 310/600/500 (舵机310/电机5 600/电机6 500), 到位后解锁底盘离站。
           这里用 Block 不用 Disarm: 收姿态+回程=塔吊在动、车要开走, 0x04/0x05 不吃 trim_armed,
           只 Disarm 的话来一帧照样把车拽一下; 封死到下一站 Chassis_TrimArm() 为止。 */
        if (mission.sub == 0)
        {
            Chassis_TrimBlock();
            if (Kin_AtBootPose())
            {
                /* 收工那一步 kin 已经直接回了运动姿态 (kin_flag.end_boot), 这里不必再发 cmd=5:
                   再发一次只会把电机5 白抬到 640 再降回 600, 臂/镜头晃一下才离站。 */
                mission.sub = 0;
                chassis_flag.lock = 0;
                nav_cmd = 3;
                mission.state = MS_NAV;
                break;
            }
            kin_flag.done = 0;   /* 清残留: 要等的是这条 cmd=5 的 done, 不能是上一段的 */
            kin_flag.cmd = 5; mission.sub = 1;
        }
        else if (kin_flag.done)  /* else if: 发 cmd 那拍不判完成 (必须跨至少一拍) */
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
           原来的工作区动作统计仍在 ms_ok_count/ms_err_count 里, Watch 可看, 不再上屏。
           收车了也别再被纠偏帧拽动: 这里封死 (下次跑流程 MS_POSE_DEFAULT 的 TrimArm 会解封)。 */
        {
            if (!chassis_flag.trim_block) Chassis_TrimBlock();
            if (!ms_count_shown)
            {
                ms_count_shown = 1;
                Screen_ShowCount(mission_color.ok, mission_color.err, mission_color.total);
            }
        }
        break;

    case MS_RESTART_POSE:
        /* 调试重启后的过渡: 等塔吊回引导姿态 (Kin_BootPosture) 再让操作员启动,
           免得臂还伸着就开走。到位/停住 -> IDLE; 卡住时长按一键键可直接跳回 IDLE。 */
        if (kin_flag.done || !kin_flag.active)
        {
            kin_flag.done = 0;
            mission.state = MS_IDLE;
            ms_idle_tick = 0;
            Screen_ShowZoneBattery(nav_zone, chassis_val.vbus);
        }
        break;
    }
}
