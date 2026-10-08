/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/   
/**
  ******************************************************************************
  * @file      delay.h
  * @author    SWUST_ROBOCON_2025_by_游子恒
  * @version   V1.0.0
  * @date      2024-11-25
  * @brief     延时初始化
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 SWUST_ROBOCON.
  * All rights reserved.</center></h2>
  * 
  ******************************************************************************
  */
#ifndef _DELAY_H
#define _DELAY_H

#include "sys.h"

void delay3(int j);
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_xms(u16 nms);
void delay_us(u32 nus);

#endif

/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/ 
