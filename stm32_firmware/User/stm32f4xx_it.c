/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "sys.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

///**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{
//}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/**
  * @brief  嵌套中断向量寄存器初始化
  * @param  None
  * @retval None
  */
void nvic_config(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
//	/** TIM2 中断配置 */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	/** TIM3 中断配置 */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	/** CAN1 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/** CAN2 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/** USART1 中断配置 */
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	/** USART2 中断配置 */
//	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	/** USART3 中断配置 */
	
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;       
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/** DMA 中断配置 */
	DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
	 
	// 配置NVIC中断优先级（以USART3_IRQn为例，实际应使用DMA1_StreamX_IRQn）
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn; // 替换为实际的DMA中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 设置抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 设置子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

//void TIM3_IRQHandler(void)                                          //400Hz
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
//	{
//		NRF24L01_Receive_Data();
		
//		get_remote()->X=get_remote()->X/2;
//		get_remote()->Y=get_remote()->Y/2;
//		get_remote()->Z=get_remote()->Z/2;	
//	Usart1_Printf("%d\t\n",get_remote()->X);
//		
//	}
//	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
//}

//void TIM2_IRQHandler(void)                                          //400Hz
//{
//	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
//	{
//    switch(get_remote()->Key_Status){      
//		
//	case 4:                                                          //进入自动找球模式
//		Automatic_ball_finding();
//	break;
//	
//	case 11:
//		Small_top();                                                    //小陀螺
//	break;
//	
//	case 12:
//		coordinate_system_resolves_the_wheel_speed(get_remote()->X,get_remote()->Y);       //遥控  
//    break;	
//}	
//	TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
//	}
//}

/**
  * @brief  USART3中断服务函数
  * @param  None
  * @retval None
  */
	/*
void USART3_IRQHandler(void)
{
	uint16_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{

		USART_ReceiveData(USART3);

		if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == DMA_Memory_0)
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
			this_time_rx_len = DMA_GetCurrDataCounter(DMA1_Stream1);

			DMA_SetCurrDataCounter(DMA1_Stream1, DATA_SIZE);

			DMA_Cmd(DMA1_Stream1, ENABLE);
			if(this_time_rx_len == DATA_SIZE)
			{
				Visual_data_processing(get_usart3_rx_buffer0());
			}
		}
		else
		{
			DMA_Cmd(DMA1_Stream1, DISABLE);
			this_time_rx_len = DMA_GetCurrDataCounter(DMA1_Stream1);

			DMA_SetCurrDataCounter(DMA1_Stream1, DATA_SIZE);

			DMA_Cmd(DMA1_Stream1, ENABLE);

			if( this_time_rx_len== DATA_SIZE)
			{
				Visual_data_processing(get_usart3_rx_buffer1());
			}
		}
	}
}

*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
