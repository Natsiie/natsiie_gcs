#include "sys.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         跑在 DATE_Tasks (10ms)。只做「调度」：读底盘/塔吊/串口状态, 下发 nav_cmd / kin_cmd,
  *         不直接动电机/舵机。mission_enable=0 时完全不介入, 保留原「1功能测试代码」。
  *
  *         赛程 (两个启动区共用, 坐标随 nav_zone):
  *           上电引导位姿 -> 等按键(短按切区/长按启动)
  *           -> 直连物料点(跳过拐角): 先走 X 再走 Y, 二维码沿路后台扫
  *           -> 全程导航: 从物料点起, 之后每去一个工作区都「中点旋转 -> 工作区作业 -> 回中点」
  *           工作区顺序: 物料(装货) -> 粗加工(卸货+回装) -> 暂存(堆叠), 走两轮。
  *
  *         注意: 底盘移动复用 App_Chassis_Task, 仍受遥控 sw[0]==2(解锁)且在线 门控,
  *              比赛时遥控需保持解锁并在线, 否则底盘不动(塔吊仍会动, 会不同步)。
  */

/* ================= 状态 ================= */
enum {
    MS_BOOT = 0,           /* 上电引导位姿: 夹爪开 -> 平台A -> 塔吊 Tower_Boot_Task 姿态 220/600/500 */
    MS_IDLE,               /* 等按键 (短按切区/长按启动) */
    MS_QR_GOTO_MAT_X,      /* 启动区直连物料点: 先走 X (保持当前 y) */
    MS_QR_GOTO_MAT_X_WAIT, /* 等 X 到位 */
    MS_QR_GOTO_MAT_Y,      /* 再走 Y (到 material_y) */
    MS_QR_GOTO_MAT_Y_WAIT, /* 等 Y 到位 */
    MS_QR_ROTATE_MAT,      /* 到物料点转正朝向 material_yaw */
    MS_NAV,                /* 全程导航运行中, 等 nav_wait */
    MS_ROTATE,             /* 中点原地旋转 */
    MS_TRIM,               /* 工作区视觉纠偏 (等 within=1 后锁底盘) */
    MS_POSE_DEFAULT,       /* 塔吊回默认姿态 42/500/0 (清视觉视场) */
    MS_LOAD,               /* 装货 (物料区) */
    MS_UNLOAD,             /* 卸货 (粗加工: 平台 -> 环, 落高50) */
    MS_RELOAD,             /* 回装 (粗加工: 环 -> 平台, 按二维码环序) */
    MS_STACK,              /* 堆叠 (暂存: 平台 -> 环, 落高400) */
    MS_POSE_BOOT,          /* 塔吊回 Tower_Boot_Task 姿态 220/600/500 (离站前) */
    MS_DONE,               /* 收车完成 */
};

uint8_t mission_enable = 0;   /* 0=保留单功能测试代码; 1=全自动比赛流程 (Watch 改) */
uint8_t mission_skip_work = 0; /* 写1=跳过当前工作区(物料/粗加工/暂存)塔吊作业直接离站, 跳完自动清0 (Watch) */

/* 二维码解析结果: 只用到第2组(第1轮环序)/第4组(第2轮环序), 第1/3组颜色由视觉自己把握 */
static uint8_t ms_qr_rings[2][3] = { {1,2,3}, {1,2,3} };   /* [轮][序号] = 环号(1~3) */
static uint8_t ms_qr_valid = 0;

static uint8_t  ms_state = MS_BOOT;
static uint32_t ms_tick  = 0;
static uint8_t  ms_round = 0;            /* 当前轮: 0=第1轮, 1=第2轮 */
static uint8_t  ms_sub   = 0;            /* 工作区内第几个物料 (0/1/2) */
static uint8_t  ms_key_last = 0;
static uint32_t ms_key_press_tick = 0;

/* ================= 外部接口 ================= */
extern uint8_t        nav_zone;
extern uint8_t        nav_cmd, nav_active, nav_wait;
extern int8_t         nav_step;
extern NavPointId     nav_at_point;
extern NavZoneCoord_t nav_zone_coord[2];
void       Nav_Start(void); void Nav_Stop(void); void Nav_Continue(void);
void       Nav_StartAt(NavPointId id);
void       Nav_GetPointCoord(NavPointId id, float *x, float *y);
NavPointId Nav_NextPoint(void);

extern uint8_t kin_cmd, kin_active, kin_done;
extern uint8_t kin_handshake_enable;
extern uint8_t kin_auto_grip_enable;
extern uint8_t kin_auto_release_enable;
extern float   kin_target_x, kin_target_y;
extern float   kin_unload_x[3], kin_unload_y[3];
extern uint8_t kin_unload_lock;
extern float   kin_grab_z_mm, kin_grab_approach_deg, kin_grab_y_offset_mm;
extern float   kin_drop_hoist_deg, kin_reload_grab_hoist_deg;
extern float   kin_unload_x_offset_mm, kin_unload_y_offset_mm;
void Kin_BootPosture(void);
void Kin_Stop(void);

extern uint8_t chassis_pos_active;
extern float   chassis_pos_target_x_mm, chassis_pos_target_y_mm;
extern float   chassis_odom_x_mm, chassis_odom_y_mm;
extern uint8_t chassis_trim_within;
extern uint8_t chassis_lock;
extern uint8_t chassis_rotate_active;
extern uint8_t chassis_qr_scan_active;
extern float   chassis_qr_scan_speed;
void Chassis_RotateTo(float yaw_deg);

extern volatile uint8_t Serial_ColorTargetReady;
extern volatile uint8_t Serial_QRReady;
extern Serial_RxData_t   Serial_RxData;

void Screen_ShowZone(uint8_t zone);

/* ================= 二维码解析 ================= */
/* "156+231+516+312" -> 4组3位, 只取第2组(下标3~5)/第4组(下标9~11)作环序 */
static void Mission_ParseQR(const char *s)
{
    uint8_t d[12], n = 0, i;
    while (*s && n < 12)
    {
        if (*s >= '0' && *s <= '9') d[n++] = (uint8_t)(*s - '0');
        s++;
    }
    if (n != 12) return;
    for (i = 0; i < 3; i++)
    {
        uint8_t r1 = d[3 + i], r2 = d[9 + i];
        if (r1 < 1 || r1 > 3) r1 = i + 1;
        if (r2 < 1 || r2 > 3) r2 = i + 1;
        ms_qr_rings[0][i] = r1;
        ms_qr_rings[1][i] = r2;
    }
    ms_qr_valid = 1;
}

static uint8_t Mission_Ring(uint8_t round, uint8_t idx)
{
    return ms_qr_rings[round ? 1 : 0][idx];
}

/* ================= 塔吊参数准备 ================= */
static void Mission_LoadParams(void)     /* 装货: 等视觉0x09授权下探, 自动夹 */
{
    kin_handshake_enable  = 1;      /* 到抓取位上方等 0x09 ok 再降 */
    kin_auto_grip_enable  = 1;      /* 下探到 447 后自动闭夹爪 */
    kin_auto_release_enable = 0;
    kin_grab_approach_deg = 550.0f;
    kin_grab_z_mm         = 447.0f;
    kin_grab_y_offset_mm  = 30.0f;  /* 装货视觉 y 补偿 (实测 3cm) */
}

static void Mission_ReloadParams(void)   /* 回装: 不握手, 直接降到环上抓, 自动夹 */
{
    kin_handshake_enable  = 0;      /* 不等 0x09, 到位直接下探 */
    kin_auto_grip_enable  = 1;
    kin_auto_release_enable = 0;
    kin_grab_z_mm         = kin_reload_grab_hoist_deg;  /* 环高约50 */
    kin_grab_y_offset_mm  = 0.0f;   /* 目标坐标已按卸货落点补好, 不再补 */
    kin_unload_lock       = 1;      /* 回装期间冻结卸货坐标, 防视觉重发 0x0C 中途改坐标 */
}

static void Mission_UnloadParams(float drop_deg)  /* 卸货/堆叠: 平台->环, 自动松, 落高 drop_deg */
{
    kin_auto_release_enable = 1;     /* 到位自动松夹爪 */
    kin_drop_hoist_deg      = drop_deg;
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
            chassis_lock = 0;
            chassis_qr_scan_active = 0;
            chassis_rotate_active = 0;
            ms_state = MS_QR_GOTO_MAT_X;
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

/* ================= 主状态机 ================= */
void Mission_Update(uint8_t key_pressed)
{
    if (!mission_enable) return;   /* 测试模式: 不介入 */

    ms_tick++;

    /* 收到二维码 -> 解析环序 (后台, 任意状态都收) */
    if (Serial_QRReady) { Serial_QRReady = 0; Mission_ParseQR(Serial_RxData.str); }

    /* 手动跳过工作区 (物料/粗加工/暂存): 写 mission_skip_work=1
       -> 跳过当前这个工作区的塔吊作业, 停塔吊、解锁底盘直接离站去下一点 (跳完自动清0)。
       仅在纠偏/作业/收姿态等工作区状态(MS_TRIM~MS_POSE_BOOT)生效, 不影响导航/中点旋转/收车。 */
    if (mission_skip_work && ms_state >= MS_TRIM && ms_state <= MS_POSE_BOOT)
    {
        mission_skip_work = 0;
        Kin_Stop();          /* 立即停塔吊, 不再走当前作业/收姿态 */
        chassis_lock = 0;
        nav_cmd = 3;         /* 解锁底盘, 直接推进导航去下一点 */
        ms_state = MS_NAV;
        return;              /* 本拍不再跑 switch, 防 nav_wait 未清零时重新进入工作区 */
    }

    switch (ms_state)
    {
    case MS_BOOT:
        /* 引导位姿: 夹爪开 + 走到平台抓取位(220/500/447), 到位后进入等按键 */
        {
            static uint8_t boot_sent = 0;
            if (!boot_sent) { boot_sent = 1; Kin_BootPosture(); }
            else if (kin_done) { kin_done = 0; boot_sent = 0; ms_state = MS_IDLE; Screen_ShowZone(nav_zone); }
        }
        break;

    case MS_IDLE:
        Mission_KeyIdle(key_pressed);
        break;

    case MS_QR_GOTO_MAT_X:
        {
            /* 启动区直连物料点(跳过拐角): 先走 X, 保持当前 y (麦轮横移, 不转向) */
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            chassis_pos_target_x_mm = z->material_x;
            chassis_pos_target_y_mm = chassis_odom_y_mm;
            chassis_pos_active = 1;
            ms_state = MS_QR_GOTO_MAT_X_WAIT;
        }
        break;

    case MS_QR_GOTO_MAT_X_WAIT:
        if (!chassis_pos_active) ms_state = MS_QR_GOTO_MAT_Y;   /* X 到位 -> 走 Y */
        break;

    case MS_QR_GOTO_MAT_Y:
        {
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            chassis_pos_target_x_mm = z->material_x;
            chassis_pos_target_y_mm = z->material_y;
            chassis_pos_active = 1;
            ms_state = MS_QR_GOTO_MAT_Y_WAIT;
        }
        break;

    case MS_QR_GOTO_MAT_Y_WAIT:
        if (!chassis_pos_active)
        {
            /* 已到物料点: 转正到物料区朝向(替代原"中点旋转"), 再接入全程导航 */
            NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
            Chassis_RotateTo(z->material_yaw);
            ms_state = MS_QR_ROTATE_MAT;
        }
        break;

    case MS_QR_ROTATE_MAT:
        if (!chassis_rotate_active)
        {
            /* 转完 -> 以「已到物料区」状态接入全程导航 (跳过拐角+第一个中点) */
            Nav_StartAt(NAV_MATERIAL);
            ms_state = MS_NAV;
        }
        break;

    case MS_NAV:
        if (!nav_active) { ms_state = MS_DONE; break; }   /* 全部走完收车 */
        if (nav_wait)
        {
            ms_round = (nav_step >= 8) ? 1 : 0;   /* 工作区在第2段(下标>=8)=第2轮 */
            if (nav_at_point == NAV_MID) { Mission_RotateAtMid(); ms_state = MS_ROTATE; }
            else                          ms_state = MS_TRIM;
        }
        break;

    case MS_ROTATE:
        if (!chassis_rotate_active) { nav_cmd = 3; ms_state = MS_NAV; }   /* 转完 -> 继续导航 */
        break;

    case MS_TRIM:
        /* 视觉纠偏到位后锁底盘(忽略后续纠偏), 先把塔吊收到默认姿态再作业 */
        if (chassis_trim_within && !chassis_pos_active)
        {
            chassis_lock = 1;
            ms_sub = 0;
            ms_state = MS_POSE_DEFAULT;
        }
        break;

    case MS_POSE_DEFAULT:
        /* 塔吊回默认姿态 42/500/0 (清出视觉视场), 到位后按工作区类型分派作业 */
        if (ms_sub == 0) { kin_cmd = 4; ms_sub = 1; }
        if (kin_done)
        {
            kin_done = 0;
            ms_sub = 0;
            if (nav_at_point == NAV_MATERIAL) { Mission_LoadParams();    ms_state = MS_LOAD; }
            else if (nav_at_point == NAV_ROUGH) { Mission_UnloadParams(50.0f);  ms_state = MS_UNLOAD; }
            else { Mission_UnloadParams(400.0f); ms_state = MS_STACK; }
        }
        break;

    case MS_LOAD:
        /* 装货 x3: 视觉发 0x08 坐标 -> 锁存坐标过去 -> 等 0x09 ok -> 自动夹 -> 收平台。
           0x08 可能周期性带抖动, 抓完清标志只认下一个物料的新坐标(旧的 0x08 不管)。 */
        if (ms_sub < 3)
        {
            if (kin_done) { kin_done = 0; ms_sub++; Serial_ColorTargetReady = 0; }   /* 抓+放+回收完成, 清残留抖动帧 */
            if (ms_sub < 3 && !kin_active && Serial_ColorTargetReady)
            {
                Serial_ColorTargetReady = 0;
                kin_cmd = 1;               /* 用 Serial.c 已更新的 kin_target_x/y 去抓 */
            }
        }
        else
        {
            ms_sub = 0;
            ms_state = MS_POSE_BOOT;
        }
        break;

    case MS_UNLOAD:
        /* 卸货: 平台抓 -> 依次放 3 个环 (kin_cmd=3 走完), 落高 50 */
        if (ms_sub == 0) { kin_cmd = 3; ms_sub = 1; }   /* Kin_UnloadStart */
        if (kin_done)
        {
            kin_done = 0;
            ms_sub = 0;
            Mission_ReloadParams();
            ms_state = MS_RELOAD;
        }
        break;

    case MS_RELOAD:
        /* 回装 x3: 按二维码第2/4组环序从环上抓回放平台 (非固定1->2->3) */
        if (ms_sub < 3)
        {
            if (kin_done) { kin_done = 0; ms_sub++; }
            if (ms_sub < 3 && !kin_active)
            {
                uint8_t ring = Mission_Ring(ms_round, ms_sub);
                kin_target_x = kin_unload_x[ring - 1] + kin_unload_x_offset_mm;   /* 与卸货落点同一点 */
                kin_target_y = kin_unload_y[ring - 1] - kin_unload_y_offset_mm;
                kin_cmd = 1;
            }
        }
        else
        {
            kin_unload_lock = 0;   /* 回装完恢复坐标更新, 下一站重新收 0x0C */
            ms_sub = 0;
            ms_state = MS_POSE_BOOT;
        }
        break;

    case MS_STACK:
        /* 堆叠: 平台抓 -> 依次放 3 个环, 落高 400 (电机5 度) */
        if (ms_sub == 0) { kin_cmd = 3; ms_sub = 1; }
        if (kin_done)
        {
            kin_done = 0;
            ms_sub = 0;
            ms_state = MS_POSE_BOOT;
        }
        break;

    case MS_POSE_BOOT:
        /* 作业完成: 塔吊回 Tower_Boot_Task 姿态 220/600/500, 到位后解锁底盘离站 */
        if (ms_sub == 0) { kin_cmd = 5; ms_sub = 1; }
        if (kin_done)
        {
            kin_done = 0;
            ms_sub = 0;
            chassis_lock = 0;
            nav_cmd = 3;
            ms_state = MS_NAV;
        }
        break;

    case MS_DONE:
        break;
    }
}
