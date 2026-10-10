#ifndef __APP_MISSION_H
#define __APP_MISSION_H

#include "stm32f4xx.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         mission_enable=0 时完全不介入, 保留原「1功能测试代码」。
  *         跑在 DATE_Tasks (10ms), 通过 nav_cmd / kin_cmd 协调底盘与塔吊。
  */

extern uint8_t mission_enable;   /* 0=保留单功能测试代码; 1=全自动比赛流程 (Watch 改) */
extern uint8_t mission_skip_work;/* 写1=跳过当前工作区(物料/粗加工/暂存)塔吊作业直接离站, 跳完自动清0 (Watch) */

void Mission_Update(uint8_t key_pressed);   /* 10ms 主状态机, 在 DATE_Tasks 里调 */

#endif /* __APP_MISSION_H */
