#ifndef __APP_NAV_H
#define __APP_NAV_H

#include "stm32f4xx.h"

/**
  * @brief  全程导航路径 (支持两个启动区)
  *         赛程: 启动区(起点) -> 拐角点 -> 中点, 然后依次访问工作区:
  *               物料 -> 粗加工 -> 暂存 -> 物料 -> 粗加工 -> 暂存
  *         最后收车: 中点 -> 拐角 -> 启动区(起点)。
  *         规则: 工作区(物料/粗加工/暂存)之间移动必须先经过中点(中转)。
  *         两个启动区共用同一条路径结构, 但每一类路径点坐标都各自独立;
  *         用 nav_zone 选 1 或 2, 各自坐标现场测量后填入下方 nav_zone_coord[]。
  *         每到工作区会停原地 (nav_wait=1), 抓取/视觉微调完成后给
  *         「继续」指令 (nav_cmd=3) 再走下一段。
  *         坐标单位 mm (世界系)。
  */

/* 路径点 ID */
typedef enum {
    NAV_START = 0,   /* 启动区起点 (坐标随 nav_zone 切换, 默认里程计原点 0,0) */
    NAV_CORNER,      /* 启动区拐角点 (坐标随 nav_zone 切换) */
    NAV_MID,         /* 中点 (中转枢纽) */
    NAV_MATERIAL,    /* 物料区 */
    NAV_ROUGH,       /* 粗加工区 */
    NAV_STAGING,     /* 暂存区 */
    NAV_POINT_COUNT
} NavPointId;

/* 一个启动区的全部路径点坐标 (mm, 世界系) */
typedef struct {
    float start_x,    start_y;      /* 起点 (通常为里程计原点 0,0) */
    float corner_x,   corner_y;     /* 拐角点 */
    float mid_x,      mid_y;        /* 中点 */
    float material_x, material_y;   /* 物料区 */
    float rough_x,    rough_y;      /* 粗加工区 */
    float staging_x,  staging_y;    /* 暂存区 */
} NavZoneCoord_t;

/* ================= 路径点坐标 (mm, 世界系) — TODO 现场测量后填入 ================= */
extern uint8_t          nav_zone;                 /* 当前启动区: 1=启动区1, 2=启动区2; Nav_Start 前设好 */
extern NavZoneCoord_t   nav_zone_coord[2];        /* [0]=启动区1, [1]=启动区2 */
extern float            nav_staging_mid_dx;       /* 里程计漂移修正量 X (mm), 现场微调 */
extern float            nav_staging_mid_dy;       /* 里程计漂移修正量 Y (mm), 现场微调 */

/* ================= 导航状态 (Keil Watch) ================= */
extern uint8_t nav_cmd;    /* 调试: 写 1=启动, 2=中止, 3=继续; 现场用 btn3 启动 */
extern uint8_t nav_active; /* 只读: 1=导航中 */
extern uint8_t nav_wait;   /* 只读: 1=已到工作区, 停原地等「继续」指令 */
extern int8_t  nav_step;   /* 只读: 当前路径点序号 (-1=未启动) */

void Nav_Update(void);      /* 10ms, 由 App_Chassis_Task 调用 */
void Nav_Start(void);       /* 启动全程导航 */
void Nav_Stop(void);        /* 中止导航 (交还遥控/手动控制) */
void Nav_Continue(void);    /* 从工作区暂停点继续 (nav_wait==1 时有效) */

#endif /* __APP_NAV_H */
