#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

/**
  * @brief  麦轮底盘: 遥控直控 + 陀螺仪航向保持 + 点对点定位 + 里程计 + 电池电压
  *         电机地址: 1=BL(后左) 2=BR(后右) 3=FL(前左) 4=FR(前右)
  */
void App_Chassis_Init(void);        /* 上电初始化 (懒初始化, 预留) */
void App_Chassis_Task(void);        /* 10ms: 遥控直控底盘 (含定位/里程计/电压) */
void App_Chassis_Test_Task(void);   /* 10ms: 底盘单项测试 (Watch 改 chassis_test_mode) */

#endif
