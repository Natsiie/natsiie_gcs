#include "sys.h"

/**
  * @brief  全程导航: 按赛程走一串路径点, 复用底盘的点对点定位 (chassis_pos_*)
  *         坐标先占位 0, 现场测完填下面的 nav_*_x / nav_*_y 即可, 逻辑不用改。
  */

/* 底盘定位接口 (定义在 app_chassis.c) */
extern uint8_t chassis_pos_active;
extern float   chassis_pos_target_x_mm;
extern float   chassis_pos_target_y_mm;
extern float   chassis_odom_x_mm;      /* 里程计世界坐标 X, 最后一次粗加工->暂存的中点到达时修正漂移 */
extern float   chassis_odom_y_mm;      /* 里程计世界坐标 Y */

/* ================= 路径点坐标 (mm, 世界系) — TODO 现场测量后填入 ================= */
uint8_t nav_zone = 1;    /* 当前启动区: 1=启动区1, 2=启动区2; Nav_Start 前设好 */

NavZoneCoord_t nav_zone_coord[2] = {
    /* [0] 启动区1 (当前默认, 带上原来的值) */
    {
        .start_x    = 0.0f,    .start_y    = 0.0f,
        .corner_x   = 1070.0f, .corner_y   = -10.0f,
        .mid_x      = 1090.0f, .mid_y      = -1040.0f,
        .material_x = 243.0f,  .material_y = -1040.0f,
        .rough_x    = 1915.0f, .rough_y    = -1040.0f,
        .staging_x  = 1090.0f, .staging_y  = -1915.0f,
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

float nav_staging_mid_dx = -140.0f;   /* 里程计漂移修正量 X (共用; 现场微调) */
float nav_staging_mid_dy = 0.0f;      /* 里程计漂移修正量 Y (共用; 现场微调) */

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

static int8_t nav_index = -1;   /* 当前在 nav_route[] 里的下标 */
static uint8_t nav_staging_mid_count = 0;  /* 已到「粗加工->暂存」段中点次数 (第二次/最后一次时修正里程计 x) */

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
    case NAV_START:    chassis_pos_target_x_mm = z->start_x;    chassis_pos_target_y_mm = z->start_y;    break;
    case NAV_CORNER:   chassis_pos_target_x_mm = z->corner_x;   chassis_pos_target_y_mm = z->corner_y;   break;
    case NAV_MID:      chassis_pos_target_x_mm = z->mid_x;      chassis_pos_target_y_mm = z->mid_y;      break;
    case NAV_MATERIAL: chassis_pos_target_x_mm = z->material_x; chassis_pos_target_y_mm = z->material_y; break;
    case NAV_ROUGH:    chassis_pos_target_x_mm = z->rough_x;    chassis_pos_target_y_mm = z->rough_y;    break;
    case NAV_STAGING:  chassis_pos_target_x_mm = z->staging_x;  chassis_pos_target_y_mm = z->staging_y;  break;
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
    Nav_LoadWaypoint(nav_route[0]);
    chassis_pos_active = 1;
}

/* 中止导航: 交还遥控/手动控制 (不主动停轮, 下一拍由遥控分支接管) */
void Nav_Stop(void)
{
    nav_active = 0;
    nav_wait = 0;
    nav_index = -1;
    nav_step = -1;
    nav_at_point = NAV_POINT_COUNT;
    chassis_pos_active = 0;
    nav_test_active = 0;   /* 结束(含走完/中止)同时结束精度测试 */
}

/* 从暂停点继续: 推进到下一个路径点 (仅 nav_wait==1 时有效) */
void Nav_Continue(void)
{
    if (!nav_active || !nav_wait) return;

    nav_wait = 0;
    nav_at_point = NAV_POINT_COUNT;

    if (nav_index + 1 >= NAV_ROUTE_LEN) { Nav_Stop(); return; }
    nav_index++;
    Nav_LoadWaypoint(nav_route[nav_index]);
    chassis_pos_active = 1;
    nav_step = nav_index;
}

/* 从指定路径点直接接入全程导航 (跳过前面的路径点):
   用在启动区直连物料点后 —— 车已在物料点, 以「已到该工作区(nav_wait=1)」状态接入,
   之后按正常流程给「继续」指令走下一个点 (物料 -> 中点)。
   找 nav_route 里第一个 == id 的下标; 找不到则退化为从头启动。 */
void Nav_StartAt(NavPointId id)
{
    int8_t i;
    for (i = 0; i < NAV_ROUTE_LEN; i++)
        if (nav_route[i] == id) break;

    if (i >= NAV_ROUTE_LEN) { Nav_Start(); return; }

    nav_index = i;
    nav_active = 1;
    nav_wait = 1;               /* 已在该点, 停原地等「继续」 */
    nav_step = i;
    nav_at_point = id;
    nav_staging_mid_count = 0;
    chassis_pos_active = 0;     /* 车已到位(直连段到达), 不重新定位 */
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
    case NAV_STAGING:  *x = z->staging_x;  *y = z->staging_y;  break;
    default:           *x = 0.0f;          *y = 0.0f;          break;
    }
}

/* 当前暂停点的下一个路径点 (供中点旋转算朝向; 无则回 START) */
NavPointId Nav_NextPoint(void)
{
    if (nav_index < 0 || nav_index + 1 >= NAV_ROUTE_LEN) return NAV_START;
    return (NavPointId)nav_route[nav_index + 1];
}

/**
  * @brief  导航状态机 (每 10ms 调用, 在 App_Chassis_Task 锁检查之后执行)
  *         - btn3 边沿启动导航 (导航中忽略, 防误触重来); nav_cmd 1=启动 2=中止 3=继续 (处理后回 0)
  *         - 底盘定位到达(chassis_pos_active==0)就推进到下一个路径点
  *         - 锁定暂停(chassis_pos_active==2)解锁后自动继续
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
    if (chassis_pos_active == 2) { chassis_pos_active = 1; return; }

    /* 已到工作区, 停原地等「继续」指令 (抓取/视觉微调), 不推进 */
    if (nav_wait) return;

    /* 当前点到达 -> 推进到下一个路径点; 全部走完则结束 */
    if (chassis_pos_active == 0)
    {
        /* 精度测试: 每次到点记录误差 (里程计-目标), 纯里程计不修正漂移 */
        if (nav_test_active)
        {
            float tx, ty;
            Nav_GetPointCoord((NavPointId)nav_route[nav_index], &tx, &ty);
            nav_test_err_x = chassis_odom_x_mm - tx;
            nav_test_err_y = chassis_odom_y_mm - ty;
            float e = sqrtf(nav_test_err_x * nav_test_err_x + nav_test_err_y * nav_test_err_y);
            if (e > nav_test_err_max) nav_test_err_max = e;
        }

        /* 刚到达当前点: 最后一次「粗加工->暂存」的中点到达时修正里程计世界坐标
           (中点 = 下一路径点是暂存区的中转点; 消除累计漂移, 后续点全部跟着修正)。
           精度测试跳过: 要测的就是纯里程计漂移。 */
        if (!nav_test_active &&
            nav_route[nav_index] == NAV_MID &&
            nav_index + 1 < NAV_ROUTE_LEN &&
            nav_route[nav_index + 1] == NAV_STAGING)
        {
            nav_staging_mid_count++;
            if (nav_staging_mid_count >= 2)
            {
                chassis_odom_x_mm += nav_staging_mid_dx;
                chassis_odom_y_mm += nav_staging_mid_dy;
            }
        }

        /* 到达工作区/中点 -> 停下来等「继续」 (不自动推进)
           中点也停: 总控在中点原地旋转朝向下一个工作区后再继续。
           精度测试不停, 直接连续走完全程测误差。 */
        if (!nav_test_active &&
            (nav_route[nav_index] == NAV_MATERIAL ||
             nav_route[nav_index] == NAV_ROUGH    ||
             nav_route[nav_index] == NAV_STAGING  ||
             nav_route[nav_index] == NAV_MID))
        {
            nav_wait = 1;
            nav_step = nav_index;
            nav_at_point = (NavPointId)nav_route[nav_index];
            return;
        }

        if (nav_index + 1 >= NAV_ROUTE_LEN) { Nav_Stop(); return; }
        nav_index++;
        Nav_LoadWaypoint(nav_route[nav_index]);
        chassis_pos_active = 1;
        nav_step = nav_index;
    }
}
