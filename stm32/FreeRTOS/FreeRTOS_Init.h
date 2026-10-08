/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      FreeRTOS_Init.c
  * @author    SWUST_ROBOCON_2025_by_ÓÎ×Óºã
  * @version   V1.0.0
  * @date      2024-11-25
  * @brief     sys
  ******************************************************************************
  * @attention
  * 
  *
  *
  ******************************************************************************
  */
#ifndef __FREERTOS_INIT_H
#define __FREERTOS_INIT_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
	 
typedef struct		
{
	TaskStatus_t Debug_Task_Status;
	TaskStatus_t Button_Task_Status;
}TASK_STATUS;
	 	 
	 
void freertos_init(void);
//TaskHandle_t get_Debug_task_handler(void);
TaskHandle_t get_Chassis_task_handler(void);
TaskHandle_t get_DATE_task_handler(void);
//TimerHandle_t get_Time1_handler(void);


TASK_STATUS * Get_TASK_STATUS(void);


#endif 

/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 
