/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      sys.c
  * @author    SWUST_ROBOCON_2025_by_���Ӻ�
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
#include "sys.h"
static void driver_Init(void);
static void Parameters_Init(void);

/**
  * @brief     ϵͳ��ʼ������
  * @param     None
  * @retval    None
  * @return    None
  * @note      None
  */
void system_init(void)
{
	driver_Init();
	Parameters_Init();
	
	freertos_init();                        //��������ʼ�������к���������ӹ�CPU
			

}

/**
  * @brief     �������ʼ������
  * @param     None
  * @retval    None
  * @return    None
  * @note      None
  */
static void driver_Init(void)
{
	delay_init(168);
	delay_ms(300);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	CAN1_Init();      // 步进电机 CAN (500kbps 扩展帧)
	Imu_Init();       // 陀螺仪 JY901 (USART2, 内部用 delay_ms 扫描波特率, 必须在调度器前)
	Servo_Init();     // 舵机 TIM2 PWM 50Hz
	Key_Init();       // 一键启动开关 (PC9 输入)
}

/**
  * @brief     ������ʼ������
  * @param     None
  * @retval    None
  * @return    None
  * @note      None
  */
static void Parameters_Init(void)
{
//	App_Chassis_Init();
//	App_Lifter_Init();

}


/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 

