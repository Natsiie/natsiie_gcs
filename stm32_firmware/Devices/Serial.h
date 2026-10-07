#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdint.h>

/* ============================================================
 *  上位机 <-> STM32 串口通信协议（复刻自 yyb_stm32 / SYSTEM/Serial.c）
 *
 *  物理层 : USART1, PA9=TX, PA10=RX, 115200, 8N1, 无校验
 *  帧格式 : 0xFF 帧头 + 数据 + 0xFE 帧尾（包头包尾共 10 字节, 数据 8 字节）
 *    接收 : 0xFF + 8 字节数据 + 0xFE   （共 10 字节）
 *    发送 : 0xFF + 8 字节数据 + 0xFE   （共 10 字节）
 *  接收方式: RXNE 中断 + 三状态状态机，收满一帧置 Serial_RxFlag = 1
 * ============================================================ */

#define SERIAL_RX_DATA_LEN  8   /* 接收数据长度（字节） */
#define SERIAL_TX_DATA_LEN  8   /* 发送数据长度（字节） */

/* natsiie GCS 帧类型 (与上位机 protocol.py 的 TOPIC_TYPES 保持一致) */
#define SERIAL_TYPE_GIMBAL_YAW  0x07   /* 云台舵机角度反馈 (下位机 -> 上位机, ASCII 度) */

extern uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN]; /* 待发送的 8 字节数据 */
extern uint8_t Serial_RxPacket[SERIAL_RX_DATA_LEN]; /* 收到的 8 字节数据   */
extern uint8_t Serial_RxFlag;                       /* 收到完整一帧 = 1    */

void     Serial_Init(void);                        /* 初始化 USART1            */
void     Serial_SendByte(uint8_t byte);            /* 阻塞发送 1 字节          */
void     Serial_SendArray(uint8_t *array, uint16_t len);
void     Serial_SendString(char *str);
void     Serial_SendPacket(void);                  /* 0xFF + 8 字节 + 0xFE     */
void     Serial_SendGimbalYaw(float angle_deg);    /* 云台舵机角度 -> 上位机 (GCS 0x07 帧) */
uint8_t  Serial_GetRxFlag(void);                   /* 读一次，读完自动清 0     */
void     Serial_SetRxState(uint8_t state);         /* 复位接收状态机（可选）   */

#endif
