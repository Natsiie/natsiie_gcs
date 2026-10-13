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
    /* 三工作区目标朝向 (度, 世界系, 0=+X, CCW+, 同 Imu_GetYaw) — 到中点朝该工作区转时用它 */
    float material_yaw, rough_yaw, staging_yaw;
} NavZoneCoord_t;

/* ================= 路径点坐标 (mm, 世界系) — TODO 现场测量后填入 ================= */
extern uint8_t          nav_zone;                 /* 当前启动区: 1=启动区1, 2=启动区2; Nav_Start 前设好 */
extern NavZoneCoord_t   nav_zone_coord[2];        /* [0]=启动区1, [1]=启动区2 */
extern float            nav_staging_mid_dx;       /* 里程计漂移修正量 X (mm), 现场微调 */
extern float            nav_staging_mid_dy;       /* 里程计漂移修正量 Y (mm), 现场微调 */

/* ================= 导航状态 (Keil Watch) ================= */
extern uint8_t nav_cmd;    /* 调试: 写 1=启动, 2=中止, 3=继续; 现场用 btn3 启动 */
extern uint8_t nav_active; /* 只读: 1=导航中 */
extern uint8_t nav_wait;   /* 只读: 1=已到工作区/中点, 停原地等「继续」指令 */
extern int8_t  nav_step;   /* 只读: 当前路径点序号 (-1=未启动) */
extern NavPointId nav_at_point;  /* 只读: 暂停点 ID (nav_wait==1 时有效, 否则 NAV_POINT_COUNT) */

/* ================= 导航精度测试 (无视觉, Keil Watch) ================= */
extern uint8_t nav_test_run;     /* 写 1: 启动连续导航精度测试(不停工作区/中点, 自动关视觉纠偏, 走完全程自动清 0) */
extern uint8_t nav_test_active;  /* 只读: 1=精度测试进行中 */
extern float   nav_test_err_x;   /* 只读: 最近一次到点误差 X (里程计-目标, mm) */
extern float   nav_test_err_y;   /* 只读: 最近一次到点误差 Y (里程计-目标, mm) */
extern float   nav_test_err_max; /* 只读: 全程最大到点误差 (mm) */

/* ================= 纯底盘自跑选项 (无视觉/无上层) ================= */
extern uint8_t nav_skip_corner;  /* 写 1: 启动时跳过拐角, 直连物料点(L型先X后Y, 转 material_yaw 后接入全程) */
extern uint8_t nav_auto_rotate;  /* 写 1: 到中点自动原地旋转朝向下一个工作区, 转完自动继续(不停中点) */
extern uint8_t nav_auto_run;     /* 写 1: 一键纯跑底盘(跳拐角+中点旋转+全程不停, 无视觉/无上层), 走完自动清 0 */
extern uint8_t nav_auto_active;  /* 只读: 1=一键纯跑进行中 */

void Nav_Update(void);      /* 10ms, 由 App_Chassis_Task 调用 */
void Nav_Start(void);       /* 启动全程导航 */
void Nav_Stop(void);        /* 中止导航 (交还遥控/手动控制) */
void Nav_Continue(void);    /* 从暂停点继续 (nav_wait==1 时有效) */
void Nav_StartAt(NavPointId id);  /* 从指定路径点直接接入全程导航(跳过前面路径点), 以 nav_wait=1 状态接入 */
void       Nav_GetPointCoord(NavPointId id, float *x, float *y);  /* 取某路径点坐标(随 nav_zone) */
NavPointId Nav_NextPoint(void);  /* 返回当前暂停点的下一个路径点 (供中点旋转算 yaw) */

#endif /* __APP_NAV_H */
