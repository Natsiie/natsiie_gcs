#ifndef __IMU_H
#define __IMU_H

/**
  * @brief  维特智能 JY901 陀螺仪 (USART2 @ PD5/PD6, 自带姿态融合)
  */
void  Imu_Init(void);
float Imu_GetYaw(void);   /* 返回偏航角 (度, -180 ~ 180) */

#endif /* __IMU_H */
