/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/
/**
  ******************************************************************************
  * @file      bsp_usart.h
  * @author    SWUST_ROBOCON_2026_by_����
  * @brief     �弶֧�ְ�-�������� (֧��DMA�첽����)
  ******************************************************************************
  */
#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "sys.h"

/* ���崮���豸ö�٣�����ͳһ�ӿ� */
typedef enum {
    UART_3,
    UART_5,
    UART_6
} BSP_UART_ID_e;

/* ================= ����״̬�����ṹ�� ================= */

/* C�� ����״̬�ṹ�� (USART1) */
typedef struct {
    float    err_yaw;          // ���
    float    yaw_out;          // PID ���
    float    yaw;              // ʵ�ʽǶ�
    float    aim_yaw;          // Ŀ��Ƕ�
    uint8_t  online;           // 1���ߣ�0����
    uint32_t last_update_tick; // �����ж� C ���Ƿ�ʧ��
    float    p_term;           /* ch4: P term (RPM) */
    float    rate_filt;        /* ch5: filtered yaw rate (deg/s) */
} Chassis_Feedback_t1;

/* USART6 �ķ����ṹ�� (��CRCУ��) */
typedef struct {
    uint8_t  flag;             // ���յ��ı�־λ
    uint8_t  online;           // 1���ߣ�0����
    uint32_t last_update_tick; // �����ж��Ƿ�ʧ��
} Usart6_Feedback_t;

/* ================= ����/����Э��ṹ�� ================= */

#pragma pack(push, 1)
/* USART6 �շ����ṹ (0x55 0xAA + ���� + CRC8) �� 4 �ֽ� */
typedef struct {
    uint8_t header1;      // �̶�Ϊ 0x55
    uint8_t header2;      // �̶�Ϊ 0xAA
    uint8_t flag;         // ��־λ���� (Payload)
    uint8_t crc8;         // У��λ
} Usart6_Protocol_t;

typedef struct {
    float err_yaw;    // ͨ�� 0
    float yaw_out;    // ͨ�� 1
    float yaw;        // ͨ�� 2
    float aim_yaw;    // ͨ�� 3
    float p_term;     /* ch4: P term (RPM) */
    float rate_filt;  /* ch5: filtered yaw rate (deg/s) */
    uint8_t tail[4];  // ֡β �̶�Ϊ 0x00, 0x00, 0x80, 0x7F
} VOFA_JustFloat_t1;

#pragma pack(pop)

/* ��¶������ʹ�õ�ȫ�ֱ��� */
extern Chassis_Feedback_t1 Chassis_Feedback1;
extern Usart6_Feedback_t  Usart6_Feedback;
extern uint8_t vofa_enable;   /* VOFA 调试开关: 1=USART6 临时切给 VOFA(460800), 0=串口屏(9600) */

/* ================= ͳһ���� API ================= */
void BSP_USART_Init(BSP_UART_ID_e uart_id, uint32_t baudrate);
void BSP_USART_DMA_Send(BSP_UART_ID_e uart_id, uint8_t *data, uint16_t len);

/* ҵ�����ٷ��� API */
void USART6_Send_Command(uint8_t flag);
void USART6_Send_VOFA_JustFloat1(void);
void Vofa_Enable(void);       /* 检测 vofa_enable 跳变, 在串口屏/VOFA 间切换 USART6 */
#endif
/***************(C) COPYRIGHT SWUST_ROBOCON *************END OF FILE********************/
