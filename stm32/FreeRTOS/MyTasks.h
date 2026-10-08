/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      MyTasks.h
  * @author    SWUST_ROBOCON_2025_by_游子恒
  * @version   V1.0.0
  * @date      2024-11-25
  * @brief     FreeeRTOS_TSAKS
  ******************************************************************************
  * @attention
  * 
  *
  *
  ******************************************************************************
  */
#ifndef __MYTASKS_H
#define __MYTASKS_H

#ifdef __cplusplus
 extern "C" {       
#endif
#include "sys.h"

/******************  任务  ******************/
void Debug_Tasks(void *p_arg);
void Chassis_Tasks(void *p_arg);
void DATE_Tasks(void *p_arg);
void SIGNALING_Tasks(void *p_arg);

/******************  定时器  ******************/	 
void Timer1_Callback(TimerHandle_t xTimer);  


#ifdef __cplusplus
}
#endif

#endif


/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 
