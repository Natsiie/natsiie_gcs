#ifndef _CRSF_PROTOCOL_H__
#define _CRSF_PROTOCOL_H__

#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"

// 暴露出 16 个通道的原始数字 (比如 191~1792)
extern uint16_t CRSF_Channels[16]; 
// 暴露出最后一次收到数据的时间戳 (用于掉线保护)
extern uint32_t CRSF_LastRxTime;   

uint8_t CRC8_Calc(uint8_t *data, uint8_t len);
void CRSF_Init(void);
void CRSF_ProcessByte(uint8_t byte);

#endif
