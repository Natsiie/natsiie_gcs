#ifndef __APP_MISSION_H
#define __APP_MISSION_H

#include "stm32f4xx.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         mission.enable=0 时完全不介入, 保留原「1功能测试代码」。
  *         跑在 DATE_Tasks (10ms), 通过 nav_cmd / kin_flag.cmd 协调底盘与塔吊。
  */

/* ================= 全自动流程打包变量 (Keil Watch 只看 mission) ================= */
typedef struct {
    uint8_t  enable;           /* 0=保留单功能测试代码; 1=全自动比赛流程 (Watch 改) */
    uint8_t  mode_load;        /* 装货(物料区) 模式: 0=视觉自动 1=手动debug 2=跳过 */
    uint8_t  mode_unload;      /* 卸货(粗加工区) 模式 */
    uint8_t  mode_reload;      /* 回装(粗加工区) 模式 */
    uint8_t  mode_stack;       /* 堆叠(暂存区) 模式 */
    uint8_t  skip_work;        /* 写1=跳过当前工作区塔吊作业直接离站, 跳完自动清0 */
    uint8_t  mat_grab;         /* 物料区手动抓取: 写1 -> 塔吊走 kin_flag.cmd=10 固定位完整装货(不走视觉), 执行时自动清0 */
    uint8_t  work_done;        /* 手动debug完成标志: MS_WAIT_MANUAL 里等 Watch 写 1, 消费后自动清 0 */
    uint8_t  state;            /* 只读: 主状态机当前状态 (值与 enum 顺序一致) */
    uint8_t  sub;              /* 只读: 当前状态内子步/第几个 */
    float    scan_goto_x;      /* 启动区1: 先去的点 X (mm); 启动区2 取 -50 */
    float    scan_goto_y;      /* 两区共用: 先去的点 Y (mm) */
    float    scan_stop_x;      /* 启动区1: 慢扫 |X| 上限 (mm); 启动区2 取 -1500 */
    uint16_t vision_timeout;   /* 工作区视觉超时(10ms/拍), 1000=10s; 超时视为当前点动作完成 */
    uint16_t trim_wait_10ms;   /* 进站等底盘视觉微调 within 的上限(10ms/拍), 500=5s; 等不到就放行 */
    uint16_t scan_pause_10ms;  /* 扫码直行到 corner x 暂停时长(10ms/拍), 200=2s */
} Mission_t;

extern Mission_t mission;

void Mission_Update(uint8_t key_pressed);   /* 10ms 主状态机, 在 DATE_Tasks 里调 */

#endif /* __APP_MISSION_H */
