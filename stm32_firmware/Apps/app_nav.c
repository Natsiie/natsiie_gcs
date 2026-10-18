#include "sys.h"

/**
  * @brief  全程导航: 按赛程走一串路径点, 复用底盘的点对点定位 (chassis_flag.pos_active / chassis_val.pos_target_*)
  *         坐标先占位 0, 现场测完填下面的 nav_*_x / nav_*_y 即可, 逻辑不用改。
  */

/* 底盘定位接口已打包进 chassis_flag/chassis_cfg/chassis_val (app_chassis.h, 经 sys.h 引入) */

/* ================= 路径点坐标 (mm, 世界系) — TODO 现场测量后填入 ================= */
uint8_t nav_zone = 1;    /* 当前启动区: 1=启动区1, 2=启动区2; Nav_Start 前设好 */

NavZoneCoord_t nav_zone_coord[2] = {
    /* [0] 启动区1 (当前默认, 带上原来的值) */
    {
        .start_x    = -30.0f,    .start_y    = 0.0f,
        .corner_x   = 1070.0f, .corner_y   = -10.0f,
        .mid_x      = 1090.0f, .mid_y      = -1040.0f,
        .material_x = 100.0f,  .material_y = -1100.0f,
        .rough_x    = 1915.0f, .rough_y    = -1040.0f,
        .staging_x  = 1000.0f, .staging_y  = -1915.0f,
        /* 三工作区目标朝向: 先填"朝该站"的 atan2 方向占位, 现场标定后改 */
        .material_yaw = 90.0f, .rough_yaw = -90.0f, .staging_yaw = 180.0f,
    },
    /* [1] 启动区2 — TODO 现场测量后填入 */
    {
        .start_x    = 0.0f,     .start_y    = 0.0f,
        .corner_x   = -1070.0f, .corner_y   = -10.0f,
        .mid_x      = -1090.0f, .mid_y      = -1040.0f,
        .material_x = -1915.0f, .material_y = -1040.0f,
        .rough_x    = -243.0f,  .rough_y    = -1040.0f,
        .staging_x  = -1090.0f, .staging_y  = -1915.0f,
        .material_yaw = 90.0f, .rough_yaw = -90.0f, .staging_yaw = 180.0f,
    },
};

float nav_staging_mid_dx = 0.0f;   /* 里程计漂移修正量 X (共用; 现场微调) */
float nav_staging_mid_dy = 0.0f;      /* 里程计漂移修正量 Y (共用; 现场微调) */

/* 暂存区坐标偏移 (直接加在 staging 坐标上; 现场微调)。与 nav_staging_mid_dx/dy 不同,
   那两个是到「粗加工->暂存」中点时修里程计, 这两个是直接改暂存区目标坐标。 */
float nav_staging_dx = 0.0f;
float nav_staging_dy = 0.0f;

/* ================= 导航状态 ================= */
uint8_t nav_cmd    = 0;    /* 写 1=启动, 2=中止, 3=继续; 处理后自动回 0 */
uint8_t nav_active = 0;    /* 只读: 1=导航中 */
uint8_t nav_wait   = 0;    /* 只读: 1=已到工作区/中点, 停原地等「继续」指令 */
int8_t  nav_step   = -1;   /* 只读: 当前路径点序号 (-1=未启动) */
NavPointId nav_at_point = NAV_POINT_COUNT;  /* 只读: 暂停点 ID (nav_wait==1 时有效) */

/* 精度测试: 写 nav_test_run=1 启动连续导航(不停工作区/中点、自动关视觉), 走完全程自动清 0 */
uint8_t nav_test_run    = 0;
uint8_t nav_test_active = 0;    /* 1=测试进行中 */
float   nav_test_err_x  = 0.0f; /* 最近到点误差 X (里程计-目标) */
float   nav_test_err_y  = 0.0f; /* 最近到点误差 Y */
float   nav_test_err_max = 0.0f;/* 全程最大到点误差 */

/* 纯底盘自跑选项 (无视觉/无上层, Keil Watch):
   nav_skip_corner=1: 启动时跳过拐角, 直连物料点(先走X再走Y, 转 material_yaw 后接入全程);
   nav_auto_rotate =1: 到中点自动原地旋转朝向下一个工作区, 转完自动继续(不停中点)。 */
uint8_t nav_skip_corner = 0;
uint8_t nav_auto_rotate  = 0;
uint8_t nav_auto_run     = 0;   /* 写 1: 一键纯跑底盘(跳拐角+中点旋转+全程不停, 无视觉/无上层), 走完自动清 0 */
uint8_t nav_auto_active  = 0;   /* 只读: 1=一键纯跑进行中 */

static int8_t nav_index = -1;   /* 当前在 nav_route[] 里的下标 */
static uint8_t nav_staging_mid_count = 0;  /* 已到「粗加工->暂存」段中点次数 (第二次/最后一次时修正里程计 x) */

/* 跳过拐角直连物料点的子阶段 + 中点原地旋转状态 */
#define NAV_SKIP_X       1   /* 先走 X (保持当前 y) */
#define NAV_SKIP_Y       2   /* 再走 Y */
#define NAV_SKIP_ROTATE  3   /* 转 material_yaw */
static uint8_t nav_skip_phase = 0;   /* 0=未启用 */
static uint8_t nav_rotating   = 0;   /* 1=中点原地旋转中 */

/* ============ 直连作业路线 (两个启动区共用, 坐标随 nav_zone; 工作区之间直连, 不走中点旋转) ============ */
uint8_t nav_round = 0;               /* 只读: 0=第1轮 1=第2轮 (第2次到物料区后=1), 供 mission 用 */
static uint8_t nav_material_visits = 0;   /* 已到物料区次数 */

static const uint8_t nav_direct_route[] = {
    NAV_MATERIAL, NAV_ROUGH, NAV_STAGING,   /* 第1轮 */
    NAV_MATERIAL, NAV_ROUGH, NAV_STAGING,   /* 第2轮 */
    NAV_START,                              /* 收车: 暂存 -> 起点直连 */
};
#define NAV_DIRECT_ROUTE_LEN ((int8_t)(sizeof(nav_direct_route) / sizeof(nav_direct_route[0])))

/* 直连路线进场子阶段: 转 -> 走第一轴 -> 转 -> 走第二轴(到点) -> 转 */
#define NAV_LEG_IDLE     0
#define NAV_LEG_PRE_ROT  1   /* 出发前旋转 */
#define NAV_LEG_FIRST    2   /* 走第一轴 */
#define NAV_LEG_MID_ROT  3   /* 中途旋转 */
#define NAV_LEG_SECOND   4   /* 走第二轴(到点) */
#define NAV_LEG_DIRECT   5   /* 直连(收车: 不拆轴, 直接点对点) */
#define NAV_LEG_POST_ROT 6   /* 到点旋转 */
#define NAV_NAN_YAW      10000.0f   /* 哨兵: 该段不旋转 */
static uint8_t    nav_leg_phase = NAV_LEG_IDLE;
static NavPointId nav_leg_from = NAV_START;
static NavPointId nav_leg_to   = NAV_START;
static float nav_leg_pre_yaw  = 0.0f;
static float nav_leg_mid_yaw  = 0.0f;
static float nav_leg_post_yaw = 0.0f;
static char  nav_leg_first_axis = 'x';

static const uint8_t *Nav_ActiveRoute(int8_t *len);
static void Nav_GetLegApproach(NavPointId from, NavPointId to, float *pre_yaw, char *first_axis, float *mid_yaw, float *post_yaw);
static void Nav_LegSetAxisTarget(uint8_t first);
static void Nav_BeginLeg(NavPointId from, NavPointId to);
static void Nav_NoteArrive(NavPointId id);
static void Nav_LegArrive(const uint8_t *route, int8_t route_len);
static void Nav_LegUpdate(void);

/* 全程路径 (展平后的顺序), 两个启动区共用:
   启动区起点 -> 拐角 -> 中点, 之后每个工作区都是「中点 -> 工作区 -> 中点」,
   访问顺序: 物料 -> 粗加工 -> 暂存 -> 物料 -> 粗加工 -> 暂存 (工作区之间必过中点),
   最后收车: 中点 -> 拐角 -> 启动区起点 (所有路径点坐标随 nav_zone 切换) */
static const uint8_t nav_route[] = {
    NAV_CORNER, NAV_MID,               /* 启动区起点 -> 拐角 -> 中点 */
    NAV_MATERIAL, NAV_MID,             /* 中点 -> 物料 -> 中点 */
    NAV_ROUGH,    NAV_MID,             /* 中点 -> 粗加工 -> 中点 */
    NAV_STAGING,  NAV_MID,             /* 中点 -> 暂存 -> 中点 */
    NAV_MATERIAL, NAV_MID,             /* 中点 -> 物料 -> 中点 */
    NAV_ROUGH,    NAV_MID,             /* 中点 -> 粗加工 -> 中点 */
    NAV_STAGING,  NAV_MID,             /* 中点 -> 暂存 -> 中点 */
    NAV_CORNER,   NAV_START,           /* 中点 -> 拐角 -> 启动区起点 收车 */
};
#define NAV_ROUTE_LEN ((int8_t)(sizeof(nav_route) / sizeof(nav_route[0])))

/* 把路径点 ID 的坐标写进底盘定位目标 (坐标随 nav_zone 切换) */
static void Nav_LoadWaypoint(uint8_t id)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];

    switch (id)
    {
    case NAV_START:    chassis_val.pos_target_x_mm = z->start_x;    chassis_val.pos_target_y_mm = z->start_y;    break;
    case NAV_CORNER:   chassis_val.pos_target_x_mm = z->corner_x;   chassis_val.pos_target_y_mm = z->corner_y;   break;
    case NAV_MID:      chassis_val.pos_target_x_mm = z->mid_x;      chassis_val.pos_target_y_mm = z->mid_y;      break;
    case NAV_MATERIAL: chassis_val.pos_target_x_mm = z->material_x; chassis_val.pos_target_y_mm = z->material_y; break;
    case NAV_ROUGH:    chassis_val.pos_target_x_mm = z->rough_x;    chassis_val.pos_target_y_mm = z->rough_y;    break;
    case NAV_STAGING:  chassis_val.pos_target_x_mm = z->staging_x + nav_staging_dx;  chassis_val.pos_target_y_mm = z->staging_y + nav_staging_dy;  break;
    default: break;
    }
}

/* 启动全程: 装载第一个路径点, 交给底盘定位 */
void Nav_Start(void)
{
    nav_index = 0;
    nav_active = 1;
    nav_wait = 0;
    nav_step = 0;
    nav_at_point = NAV_POINT_COUNT;
    nav_staging_mid_count = 0;   /* 重新计数: 最后一次「粗加工->暂存」的中点到达才修正里程计 x */
    nav_skip_phase = 0;
    nav_rotating   = 0;
    nav_leg_phase  = NAV_LEG_IDLE;   /* 清掉上一轮可能残留的进场状态机 */
    nav_material_visits = 0;         /* 重新计轮次 */
    nav_round = 0;

    if (nav_skip_corner || nav_auto_active)
    {
        /* 跳过拐角: 直连物料点, 先走 X (保持当前 y), 再走 Y, 转 material_yaw 后接入全程 */
        NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
        nav_skip_phase = NAV_SKIP_X;
        chassis_val.pos_target_x_mm = z->material_x;
        chassis_val.pos_target_y_mm = chassis_val.odom_y_mm;
        chassis_flag.pos_active = 1;
        return;
    }

    {
        const uint8_t *route; int8_t route_len;
        route = Nav_ActiveRoute(&route_len);
        Nav_LoadWaypoint(route[0]);
    }
    chassis_flag.pos_active = 1;
}

/* 中止导航: 交还遥控/手动控制 (不主动停轮, 下一拍由遥控分支接管) */
void Nav_Stop(void)
{
    nav_active = 0;
    nav_wait = 0;
    nav_index = -1;
    nav_step = -1;
    nav_at_point = NAV_POINT_COUNT;
    chassis_flag.pos_active = 0;
    nav_test_active = 0;   /* 结束(含走完/中止)同时结束精度测试 */
    nav_auto_active = 0;   /* 一键纯跑结束(走完/中止) */
    nav_skip_phase = 0;
    nav_rotating   = 0;
    nav_leg_phase  = NAV_LEG_IDLE;
}

/* 从暂停点继续: 推进到下一个路径点 (仅 nav_wait==1 时有效) */
void Nav_Continue(void)
{
    const uint8_t *route; int8_t route_len;
    if (!nav_active || !nav_wait) return;

    nav_wait = 0;
    nav_at_point = NAV_POINT_COUNT;

    route = Nav_ActiveRoute(&route_len);
    if (nav_index + 1 >= route_len) { Nav_Stop(); return; }
    nav_index++;
    nav_step = nav_index;

    Nav_BeginLeg((NavPointId)route[nav_index - 1], (NavPointId)route[nav_index]);   /* 两区共用直连进场 */
}

/* 从指定路径点直接接入全程导航 (跳过前面的路径点):
   用在启动区直连物料点后 —— 车已在物料点, 以「已到该工作区(nav_wait=1)」状态接入,
   之后按正常流程给「继续」指令走下一个点 (物料 -> 中点)。
   找 nav_route 里第一个 == id 的下标; 找不到则退化为从头启动。 */
void Nav_StartAt(NavPointId id)
{
    const uint8_t *route; int8_t route_len, i;
    route = Nav_ActiveRoute(&route_len);
    for (i = 0; i < route_len; i++)
        if (route[i] == id) break;

    if (i >= route_len) { Nav_Start(); return; }

    nav_index = i;
    nav_active = 1;
    nav_wait = 1;               /* 已在该点, 停原地等「继续」 */
    nav_step = i;
    nav_at_point = id;
    nav_staging_mid_count = 0;
    nav_material_visits = 0;
    nav_round = 0;
    nav_leg_phase = NAV_LEG_IDLE;
    chassis_flag.pos_active = 0;     /* 车已到位(直连段到达), 不重新定位 */

    if (id == NAV_MATERIAL) nav_material_visits = 1;   /* 已在物料区(第1轮) */
}

/* 取某路径点坐标 (随 nav_zone 切换) */
void Nav_GetPointCoord(NavPointId id, float *x, float *y)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
    switch (id)
    {
    case NAV_START:    *x = z->start_x;    *y = z->start_y;    break;
    case NAV_CORNER:   *x = z->corner_x;   *y = z->corner_y;   break;
    case NAV_MID:      *x = z->mid_x;      *y = z->mid_y;      break;
    case NAV_MATERIAL: *x = z->material_x; *y = z->material_y; break;
    case NAV_ROUGH:    *x = z->rough_x;    *y = z->rough_y;    break;
    case NAV_STAGING:  *x = z->staging_x + nav_staging_dx;  *y = z->staging_y + nav_staging_dy;  break;
    default:           *x = 0.0f;          *y = 0.0f;          break;
    }
}

/* 当前暂停点的下一个路径点 (供中点旋转算朝向; 无则回 START) */
NavPointId Nav_NextPoint(void)
{
    const uint8_t *route; int8_t route_len;
    route = Nav_ActiveRoute(&route_len);
    if (nav_index < 0 || nav_index + 1 >= route_len) return NAV_START;
    return (NavPointId)route[nav_index + 1];
}

/* ================= 启动区1 直连路线进场 (转 -> 走轴 -> 转 -> 走轴 -> 转) ================= */

static const uint8_t *Nav_ActiveRoute(int8_t *len)
{
    *len = NAV_DIRECT_ROUTE_LEN; return nav_direct_route;   /* 两个启动区都走直连作业路线 */
}

static void Nav_GetLegApproach(NavPointId from, NavPointId to,
                               float *pre_yaw, char *first_axis, float *mid_yaw, float *post_yaw)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
    *pre_yaw = NAV_NAN_YAW; *mid_yaw = NAV_NAN_YAW; *post_yaw = NAV_NAN_YAW;
    *first_axis = 'x';

    if (from == NAV_MATERIAL && to == NAV_ROUGH)
    {
        *pre_yaw     = 0.0f;             /* 车头转 +X */
        *first_axis  = 'x';              /* 直走 X (rough 与 material 同 y) */
        *post_yaw    = z->rough_yaw;     /* 到点转粗加工朝向 */
    }
    else if (from == NAV_ROUGH && to == NAV_STAGING)
    {
        *pre_yaw     = z->rough_yaw;     /* -90, 车头朝 -Y */
        *first_axis  = 'y';              /* 先走 Y */
        *mid_yaw     = z->staging_yaw;   /* -180, 车头朝 -X */
    }
    else if (from == NAV_STAGING && to == NAV_MATERIAL)
    {
        *first_axis  = 'x';              /* 先走 X (车头已朝 -X) */
        *mid_yaw     = z->material_yaw;  /* 90, 车头朝 +Y */
    }
    else if (from == NAV_STAGING && to == NAV_START)
    {
        *first_axis  = 'x';              /* 收车: 先走 X 直回起点 X */
        *mid_yaw     = z->material_yaw;  /* 90, 车头朝 +Y, 再走 Y 回起点 Y */
    }
    else
    {
        /* 兜底 (未预期的点对): 转朝下一目标, 直线过去 */
        float cx, cy, fx, fy;
        Nav_GetPointCoord(from, &cx, &cy);
        Nav_GetPointCoord(to,   &fx, &fy);
        *pre_yaw    = atan2f(fy - cy, fx - cx) * (180.0f / MATH_PI);
        *first_axis = 0;                 /* 0 = 不拆轴, 直接点对点 */
    }
}

static void Nav_LegSetAxisTarget(uint8_t first)
{
    float dx, dy;
    Nav_GetPointCoord(nav_leg_to, &dx, &dy);
    if (first && nav_leg_first_axis != 0)
    {
        if (nav_leg_first_axis == 'x') { chassis_val.pos_target_x_mm = dx; chassis_val.pos_target_y_mm = chassis_val.odom_y_mm; }
        else                            { chassis_val.pos_target_x_mm = chassis_val.odom_x_mm; chassis_val.pos_target_y_mm = dy; }
    }
    else
    {
        chassis_val.pos_target_x_mm = dx;
        chassis_val.pos_target_y_mm = dy;
    }
    chassis_flag.pos_active = 1;
}

static void Nav_BeginLeg(NavPointId from, NavPointId to)
{
    nav_leg_from = from;
    nav_leg_to   = to;
    Nav_GetLegApproach(from, to, &nav_leg_pre_yaw, &nav_leg_first_axis, &nav_leg_mid_yaw, &nav_leg_post_yaw);

    if (nav_leg_pre_yaw != NAV_NAN_YAW)
    {
        nav_leg_phase = NAV_LEG_PRE_ROT;
        Chassis_RotateTo(nav_leg_pre_yaw);
    }
    else if (nav_leg_first_axis != 0)
    {
        nav_leg_phase = NAV_LEG_FIRST;
        Nav_LegSetAxisTarget(1);
    }
    else
    {
        nav_leg_phase = NAV_LEG_DIRECT;
        Nav_LegSetAxisTarget(0);
    }
}

static void Nav_NoteArrive(NavPointId id)
{
    if (id == NAV_MATERIAL) { nav_material_visits++; nav_round = (nav_material_visits >= 2) ? 1 : 0; }
}

static void Nav_LegArrive(const uint8_t *route, int8_t route_len)
{
    nav_leg_phase = NAV_LEG_IDLE;
    Nav_NoteArrive(nav_leg_to);

    /* 精度测试: 每次到点记录误差 (里程计-目标), 纯里程计不修正漂移 */
    if (nav_test_active)
    {
        float tx, ty, e;
        Nav_GetPointCoord(nav_leg_to, &tx, &ty);
        nav_test_err_x = chassis_val.odom_x_mm - tx;
        nav_test_err_y = chassis_val.odom_y_mm - ty;
        e = sqrtf(nav_test_err_x * nav_test_err_x + nav_test_err_y * nav_test_err_y);
        if (e > nav_test_err_max) nav_test_err_max = e;
    }

    if (!nav_test_active && !nav_auto_active &&
        (nav_leg_to == NAV_MATERIAL || nav_leg_to == NAV_ROUGH || nav_leg_to == NAV_STAGING))
    {
        nav_wait = 1;
        nav_step = nav_index;
        nav_at_point = (NavPointId)nav_leg_to;
    }
    else
    {
        /* 连续模式(精度测试/一键纯跑) 或 收车点(CORNER/START): 不停, 直接推进到下一段 */
        if (nav_index + 1 >= route_len) { Nav_Stop(); return; }
        nav_index++;
        nav_step = nav_index;
        Nav_BeginLeg((NavPointId)route[nav_index - 1], (NavPointId)route[nav_index]);
    }
}

static void Nav_LegUpdate(void)
{
    const uint8_t *route; int8_t route_len;
    route = Nav_ActiveRoute(&route_len);

    switch (nav_leg_phase)
    {
    case NAV_LEG_PRE_ROT:
        if (!chassis_flag.rotate_active)
        {
            if (nav_leg_first_axis != 0) { nav_leg_phase = NAV_LEG_FIRST; Nav_LegSetAxisTarget(1); }
            else                          { nav_leg_phase = NAV_LEG_DIRECT; Nav_LegSetAxisTarget(0); }
        }
        break;
    case NAV_LEG_FIRST:
        if (chassis_flag.pos_active == 0)
        {
            if (nav_leg_mid_yaw != NAV_NAN_YAW) { nav_leg_phase = NAV_LEG_MID_ROT; Chassis_RotateTo(nav_leg_mid_yaw); }
            else                                 { nav_leg_phase = NAV_LEG_SECOND; Nav_LegSetAxisTarget(0); }
        }
        break;
    case NAV_LEG_MID_ROT:
        if (!chassis_flag.rotate_active) { nav_leg_phase = NAV_LEG_SECOND; Nav_LegSetAxisTarget(0); }
        break;
    case NAV_LEG_SECOND:
    case NAV_LEG_DIRECT:
        if (chassis_flag.pos_active == 0)
        {
            if (nav_leg_post_yaw != NAV_NAN_YAW) { nav_leg_phase = NAV_LEG_POST_ROT; Chassis_RotateTo(nav_leg_post_yaw); }
            else                                  Nav_LegArrive(route, route_len);
        }
        break;
    case NAV_LEG_POST_ROT:
        if (!chassis_flag.rotate_active) Nav_LegArrive(route, route_len);
        break;
    }
}

/* 中点原地旋转目标 yaw (纯底盘自跑用, 与 mission 的 Mission_RotateAtMid 同逻辑):
   去工作区用该站现场标定的目标朝向; 收车去拐角/起点用 atan2 朝下一目标 */
static void Nav_RotateAtMid(void)
{
    NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
    NavPointId next = Nav_NextPoint();
    float nx, ny, target_yaw;

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

/**
  * @brief  导航状态机 (每 10ms 调用, 在 App_Chassis_Task 锁检查之后执行)
  *         - btn3 边沿启动导航 (导航中忽略, 防误触重来); nav_cmd 1=启动 2=中止 3=继续 (处理后回 0)
  *         - 底盘定位到达(chassis_flag.pos_active==0)就推进到下一个路径点
  *         - 锁定暂停(chassis_flag.pos_active==2)解锁后自动继续
  *         - 到达工作区(物料/粗加工/暂存)停原地(nav_wait=1), 等「继续」指令再走下一段
  */
void Nav_Update(void)
{
    /* 精度测试触发: 写 nav_test_run=1 -> 关视觉纠偏(由 App_Chassis_Task 看 nav_test_active),
       启动导航全程不停, 每次到点误差记进 nav_test_err_*, 全程最大记进 nav_test_err_max */
    if (nav_test_run)
    {
        nav_test_run = 0;
        nav_test_active = 1;
        nav_test_err_x = 0.0f;
        nav_test_err_y = 0.0f;
        nav_test_err_max = 0.0f;
        Nav_Start();
    }

    /* 一键纯跑底盘: 跳拐角直连物料点 + 中点自动旋转 + 全程不停车, 无视觉/无上层, 走完自动清 0 */
    if (nav_auto_run)
    {
        nav_auto_run = 0;
        nav_auto_active = 1;
        Nav_Start();
    }

    /* btn3 边沿启动导航 */
    {
        static uint8_t last_btn3 = 0;
        if (ELRS_RC.btns[3] && !last_btn3 && !nav_active) Nav_Start();
        last_btn3 = ELRS_RC.btns[3];
    }

    if (nav_cmd == 1)      { Nav_Start();    nav_cmd = 0; }
    else if (nav_cmd == 2) { Nav_Stop();     nav_cmd = 0; }
    else if (nav_cmd == 3) { Nav_Continue(); nav_cmd = 0; }

    if (!nav_active) return;

    /* 锁定期间被底盘置为暂停 -> 解锁后自动继续 */
    if (chassis_flag.pos_active == 2) { chassis_flag.pos_active = 1; return; }

    /* 已到工作区, 停原地等「继续」指令 (抓取/视觉微调), 不推进 */
    if (nav_wait) return;

    /* 直连路线: 进场状态机 (转 -> 走轴 -> 转 -> 走轴 -> 转) */
    if (nav_leg_phase != NAV_LEG_IDLE)
    {
        Nav_LegUpdate();
        return;
    }

    /* 跳过拐角直连物料点: L型先 X 后 Y, 再转 material_yaw 后接入全程 */
    if (nav_skip_phase)
    {
        NavZoneCoord_t *z = &nav_zone_coord[(nav_zone == 2) ? 1 : 0];
        switch (nav_skip_phase)
        {
        case NAV_SKIP_X:
            if (chassis_flag.pos_active == 0)
            {
                nav_skip_phase = NAV_SKIP_Y;
                chassis_val.pos_target_x_mm = z->material_x;
                chassis_val.pos_target_y_mm = z->material_y;
                chassis_flag.pos_active = 1;
            }
            break;
        case NAV_SKIP_Y:
            if (chassis_flag.pos_active == 0)
            {
                nav_skip_phase = NAV_SKIP_ROTATE;
                Chassis_RotateTo(z->material_yaw);
            }
            break;
        case NAV_SKIP_ROTATE:
            if (!chassis_flag.rotate_active)
            {
                nav_skip_phase = 0;
                Nav_StartAt(NAV_MATERIAL);           /* 转完 -> 以「已到物料区」状态接入全程 (nav_wait=1 停原地) */
                if (nav_test_active || nav_auto_active) Nav_Continue();  /* 连续模式(精度测试/一键纯跑): 不停物料点 */
            }
            break;
        }
        return;
    }

    /* 中点原地旋转中: 转完自动继续 (nav_auto_rotate=1 时到中点触发) */
    if (nav_rotating)
    {
        if (!chassis_flag.rotate_active)
        {
            nav_rotating = 0;
            if (nav_index + 1 >= NAV_ROUTE_LEN) { Nav_Stop(); return; }
            nav_index++;
            Nav_LoadWaypoint(nav_route[nav_index]);
            chassis_flag.pos_active = 1;
            nav_step = nav_index;
        }
        return;
    }

    /* 当前点到达 -> 推进到下一个路径点; 全部走完则结束 */
    if (chassis_flag.pos_active == 0)
    {
        const uint8_t *route; int8_t route_len;
        route = Nav_ActiveRoute(&route_len);

        /* 精度测试: 每次到点记录误差 (里程计-目标), 纯里程计不修正漂移 */
        if (nav_test_active)
        {
            float tx, ty;
            Nav_GetPointCoord((NavPointId)route[nav_index], &tx, &ty);
            nav_test_err_x = chassis_val.odom_x_mm - tx;
            nav_test_err_y = chassis_val.odom_y_mm - ty;
            float e = sqrtf(nav_test_err_x * nav_test_err_x + nav_test_err_y * nav_test_err_y);
            if (e > nav_test_err_max) nav_test_err_max = e;
        }

        /* 刚到达当前点: 最后一次「粗加工->暂存」的中点到达时修正里程计世界坐标
           (中点 = 下一路径点是暂存区的中转点; 消除累计漂移, 后续点全部跟着修正)。
           精度测试跳过: 要测的就是纯里程计漂移。 */
        if (!nav_test_active &&
            route[nav_index] == NAV_MID &&
            nav_index + 1 < route_len &&
            route[nav_index + 1] == NAV_STAGING)
        {
            nav_staging_mid_count++;
            if (nav_staging_mid_count >= 2)
            {
                chassis_val.odom_x_mm += nav_staging_mid_dx;
                chassis_val.odom_y_mm += nav_staging_mid_dy;
            }
        }

        /* 中点原地旋转: nav_auto_rotate=1 时到中点自动转朝向下一个工作区, 转完自动继续(不停中点) */
        if ((nav_auto_rotate || nav_auto_active) && route[nav_index] == NAV_MID)
        {
            Nav_RotateAtMid();
            nav_rotating = 1;
            return;
        }

        /* 到达工作区/中点 -> 停下来等「继续」 (不自动推进)
           中点也停: 总控在中点原地旋转朝向下一个工作区后再继续。
           精度测试不停, 直接连续走完全程测误差。 */
        if (!nav_test_active && !nav_auto_active &&
            (route[nav_index] == NAV_MATERIAL ||
             route[nav_index] == NAV_ROUGH    ||
             route[nav_index] == NAV_STAGING  ||
             route[nav_index] == NAV_MID))
        {
            Nav_NoteArrive((NavPointId)route[nav_index]);   /* 到物料区时更新轮次 */
            nav_wait = 1;
            nav_step = nav_index;
            nav_at_point = (NavPointId)route[nav_index];
            return;
        }

        if (nav_index + 1 >= route_len) { Nav_Stop(); return; }
        nav_index++;
        Nav_LoadWaypoint(route[nav_index]);
        chassis_flag.pos_active = 1;
        nav_step = nav_index;
    }
}
