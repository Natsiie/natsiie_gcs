#ifndef __APP_TOWER_H
#define __APP_TOWER_H

/**
  * @brief  塔吊 (2 步进升降/伸出) + 3 舵机 (夹爪/物料平台/云台)
  */
/* 抓取/放置流程握手变量 (Serial.c 接收上位机 0x09 GrabCmd 时写) */
extern uint8_t grab_cmd;    /* 上层写1: 装货阶段 电机5 从550降到447 下探抓取 */

void App_Tower_Init(void);
void Tower_ReadPos5(void);
void Tower_ReadPos6(void);
void Tower_ReadVBus5(void);
void App_Tower_Task(void);   /* 10ms 周期调用: 遥控直控塔吊 */
void Tower_Test_Task(void);  /* 10ms 周期调用: 一体化抓取/放置流程 (Watch 写 flow_cmd) */
void Gimbal_Test_Task(void);    /* 10ms 周期调用: 云台角度调试 (Watch 写 gimbal_test_angle) */
void Platform_Test_Task(void);  /* 10ms 周期调用: 物料平台角度调试 (Watch 写 platform_test_angle) */
void Tower_Boot_Task(void);     /* 10ms 周期调用: 上电默认状态 (云台42->220, 电机5升抓取高度, 电机6不动) */
void Motor56_Test_Task(void);   /* 10ms 周期调用: 电机5/6 测试 (位置/速度模式) */

#endif
