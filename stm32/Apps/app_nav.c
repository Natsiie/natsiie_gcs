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
        .corner_x   = 1070.0f, .corner_y   = 10.0f,
        .mid_x      = 1090.0f, .mid_y      = 1040.0f,
        .material_x = 1915.0f, .material_y = 1040.0f,
        .rough_x    = 243.0f,  .rough_y    = 1040.0f,
        .staging_x  = 1090.0f, .staging_y  = 1915.0f,
    },
    /* [1] 启动区2 — TODO 现场测量后填入 (先全部占位 0, 自己测试时改这里) */
    {
        .start_x    = 0.0f,    .start_y    = 0.0f,
        .corner_x   = 0.0f,    .corner_y   = 0.0f,
        .mid_x      = 0.0f,    .mid_y      = 0.0f,
        .material_x = 0.0f,    .material_y = 0.0f,
        .rough_x    = 0.0f,    .rough_y    = 0.0f,
        .staging_x  = 0.0f,    .staging_y  = 0.0f,
    },
};

float nav_staging_mid_dx = -140.0f;   /* 里程计漂移修正量 X (共用; 现场微调) */
float nav_staging_mid_dy = 0.0f;      /* 里程计漂移修正量 Y (共用; 现场微调) */

/* ================= 导航状态 ================= */
uint8_t nav_cmd    = 0;    /* 写 1=启动, 2=中止, 3=继续; 处理后自动回 0 */
uint8_t nav_active = 0;    /* 只读: 1=导航中 */
uint8_t nav_wait   = 0;    /* 只读: 1=已到工作区, 停原地等「继续」指令 */
int8_t  nav_step   = -1;   /* 只读: 当前路径点序号 (-1=未启动) */

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
    chassis_pos_active = 0;
}

/* 从工作区暂停点继续: 推进到下一个路径点 (仅 nav_wait==1 时有效) */
void Nav_Continue(void)
{
    if (!nav_active || !nav_wait) return;

    nav_wait = 0;

    if (nav_index + 1 >= NAV_ROUTE_LEN) { Nav_Stop(); return; }
    nav_index++;
    Nav_LoadWaypoint(nav_route[nav_index]);
    chassis_pos_active = 1;
    nav_step = nav_index;
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
        /* 刚到达当前点: 最后一次「粗加工->暂存」的中点到达时修正里程计世界坐标
           (中点 = 下一路径点是暂存区的中转点; 消除累计漂移, 后续点全部跟着修正) */
        if (nav_route[nav_index] == NAV_MID &&
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

        /* 到达工作区 -> 停下来等「继续」 (不自动推进) */
        if (nav_route[nav_index] == NAV_MATERIAL ||
            nav_route[nav_index] == NAV_ROUGH    ||
            nav_route[nav_index] == NAV_STAGING)
        {
            nav_wait = 1;
            nav_step = nav_index;
            return;
        }

        if (nav_index + 1 >= NAV_ROUTE_LEN) { Nav_Stop(); return; }
        nav_index++;
        Nav_LoadWaypoint(nav_route[nav_index]);
        chassis_pos_active = 1;
        nav_step = nav_index;
    }
}
