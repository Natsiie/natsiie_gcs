/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      sys.c
  * @author    SWUST_ROBOCON_2026_by_浮世叁千
  * @version   V1.0.0
  * @date      2025-11-25
  * @brief     sys
  ******************************************************************************
  * @attention
  * 
  *
  *
  ******************************************************************************
  */
#ifndef _SYS_H
#define _SYS_H

/*官方头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "stm32f4xx.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include "arm_math.h"
	 
/*用户头文件   freertos*/
#include "FreeRTOS_Init.h"
#include "MyTasks.h"
#include "QUEUE.H"

//core文件
#include "crsf_protocol.h"
#include "elrs_Remote.h"

//设备驱动
#include "bsp_can.h"
#include "bsp_usart.h"
#include "Serial.h"
#include "bsp_key.h"
#include "X_V2.h"
#include "servo.h"
#include "imu.h"
#include "tjc_screen.h"
#include "car_config.h"

//应用层
#include "app_chassis.h"
#include "app_tower.h"
#include "app_nav.h"
#include "app_kin.h"

//算法
#include "PID.h"
#include "delay.h"
#include "base.h"
#include "math_utils.h"

void system_init(void);

#endif 


/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 

