#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdint.h>

/* ============================================================
 *  上位机(natsiie_gcs/ROS) <-> 下位机(STM32F4) 串口协议
 *
 *  物理层: USART1, PA9=TX, PA10=RX, 115200, 8N1, 无校验
 *  帧格式: 0xFF | TYPE | LEN | PAYLOAD(UTF-8) | CRC8 | 0xFE
 *    - LEN  = payload 字节数 (0~255)
 *    - CRC8 = TYPE ^ LEN ^ PAYLOAD 逐字节异或
 *    - 类型号与上位机 identify_pack/serial/protocol.py 的 TOPIC_TYPES 一致
 *  接收: RXNE 中断 + 帧结构状态机, 收满一帧置 Serial_RxFrameReady=1,
 *        任务里调 Serial_ProcessFrame() 按 type 解析。
 * ============================================================ */

#define SERIAL_HEAD  0xFF
#define SERIAL_TAIL  0xFE

/* 帧类型 (0x07 是下位机->上位机的反馈, 不接收) */
#define SERIAL_TYPE_COLOR         0x01   /* 色块像素列表: id:cx,cy/id:cx,cy/... */
#define SERIAL_TYPE_COLOR_MOTION  0x02   /* 转盘运动: state:speed(px/s) */
#define SERIAL_TYPE_TURNTABLE     0x03   /* 转盘: cx,cy,radius,stable */
#define SERIAL_TYPE_RING_STATION  0x04   /* 三圆环工位: err_x,err_y,ang,corr_x,corr_y,corr_yaw,within */
#define SERIAL_TYPE_STACKED_RING  0x05   /* 叠放位: 同 0x04 */
#define SERIAL_TYPE_QR_DECODE     0x06   /* QR 任务码字符串 */
#define SERIAL_TYPE_GIMBAL_YAW    0x07   /* 云台角度反馈 (下位机->上位机) */
#define SERIAL_TYPE_COLOR_TARGET  0x08   /* 颜色目标点: color_id,x_mm,y_mm,z_mm */
#define SERIAL_TYPE_GRAB_CMD      0x09   /* 夹爪到位授权: payload "1"=授权下降抓取 */
#define SERIAL_TYPE_PLACE_COMMAND 0x0A
#define SERIAL_TYPE_PLACE_RESULT  0x0B
#define SERIAL_TYPE_UNLOAD_COORD  0x0C   /* 卸货坐标: 3 组 x,y (3 个落点, 塔吊系绝对 mm) */
#define SERIAL_TYPE_PLACE_CONTROL 0x0D   /* 放置阶段控制 (下位机->上位机): start,stage,batch 或 retry,command_id */
#define SERIAL_TYPE_PICK_DONE     0x0F   /* 抓取完成序号 (下位机->上位机): ASCII 序号 1~6 */
#define SERIAL_TYPE_NEXT_BATCH    0x10   /* 第一批 3 次抓取完成、回装就绪 (下位机->上位机): "1" */

#define SERIAL_RX_PAYLOAD_MAX     128    /* Includes UUID and placement XYZ. */

typedef struct {
    char task_id[33];
    uint32_t command_id;
    uint8_t stage;      /* 1=ordinary workstation, 2=stacking area */
    uint8_t batch, sequence, color, ring, layer;
    float x_mm, y_mm, z_mm;  /* gimbal_base; not the Kin ground-origin frame */
} Serial_PlaceCommand_t;

/* 旧 TX 数据包 (Serial_SendPacket 用, 8 字节) */
#define SERIAL_TX_DATA_LEN        8
extern uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN];

/* 接收原始帧 (ISR 填, 任务消费) */
typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[SERIAL_RX_PAYLOAD_MAX + 1];   /* +1 存 '\0' */
} Serial_RxFrame_t;

/* 解析结果 (Serial_ProcessFrame 按 type 填充, 业务层读) */
typedef struct {
    uint8_t type;                       /* 最近一帧的 type */

    /* 0x08 ColorTarget */
    uint8_t target_color_id;
    float   target_x_mm, target_y_mm, target_z_mm;

    /* 0x09 GrabCmd */
    uint8_t grab_cmd;

    /* 0x02 ColorMotion */
    uint8_t motion_state;
    float   motion_speed;

    /* 0x03 Turntable */
    float   turntable_cx, turntable_cy, turntable_r;
    uint8_t turntable_stable;

    /* 0x04 / 0x05 圆环工位 (偏移坐标) */
    float   ring_err_x, ring_err_y, ring_ang_err;
    float   ring_corr_x, ring_corr_y, ring_corr_yaw;
    uint8_t ring_within;

    /* 0x0C 卸货坐标 (3 个落点, 塔吊系绝对 mm) */
    float   unload_x[3], unload_y[3];
    uint8_t unload_count;

    /* 0x01 Color 像素列表 (最多 6 段) */
    uint8_t color_count;
    uint8_t color_id[6];
    int16_t color_cx[6], color_cy[6];

    /* 0x06 QR 任务码字符串 */
    char    str[SERIAL_RX_PAYLOAD_MAX + 1];
} Serial_RxData_t;

extern volatile uint8_t Serial_RxFrameReady;  /* 1 = 收到完整一帧待处理 */
extern Serial_RxFrame_t  Serial_RxFrame;
extern Serial_RxData_t   Serial_RxData;

/* 0x0C 卸货坐标(3 组 xy) 帧已解析 -> 塔吊卸货落点; 0x04/0x05 偏移坐标 corr -> 底盘微调。
 * 每收到对应帧置 1, 由业务层消费后清 0 (Kin 清 UnloadCoord, 底盘清 ChassisTrim)。 */
extern volatile uint8_t Serial_UnloadCoordReady;  /* 1 = 新卸货坐标(0x0C 3 组 xy) 待消费 */
extern volatile uint8_t Serial_ChassisTrimReady;  /* 1 = 新底盘微调(0x04/0x05 corr) 待消费 */

/* 0x08 颜色目标点 / 0x06 二维码: 供全自动流程(mission) 消费 */
extern volatile uint8_t Serial_ColorTargetReady;  /* 1 = 新颜色目标点(0x08) 待消费 (mission 清) */
extern volatile uint8_t Serial_QRReady;           /* 1 = 新二维码(0x06) 待解析 (mission 清) */

/* 工作视觉指令(0x04/05/08/09/0C)新帧标志: 供 mission 视觉超时看门狗 (mission 清) */
extern volatile uint8_t Serial_VisionCmd;

void     Serial_Init(void);                        /* 初始化 USART1            */
void     Serial_SendByte(uint8_t byte);            /* 阻塞发送 1 字节          */
void     Serial_SendArray(uint8_t *array, uint16_t len);
void     Serial_SendString(char *str);
void     Serial_SendPacket(void);                  /* 0xFF + 8 字节 + 0xFE     */
void     Serial_SendGimbalYaw(float angle_deg);    /* 云台角度 -> 上位机 (0x07) */
void     Serial_ProcessFrame(void);                /* 任务里调用: 按 type 解析一帧 */
/* Poll once when ready to execute. Complete only after release and retreat.
 * These functions retain/deduplicate commands; they do not drive actuators.
 * Both must run in the same task as Serial_ProcessFrame(). */
uint8_t Serial_TakePlaceCommand(Serial_PlaceCommand_t *out);
void Serial_CompletePlaceCommand(const char *task_id, uint32_t command_id, uint8_t success);
void Serial_SendPlaceControlStart(uint8_t stage, uint8_t batch);  /* 0x0D: start,<stage>,<batch> */
void Serial_SendPlaceControlRetry(uint32_t command_id);           /* 0x0D: retry,<command_id> */
/* 0x0F: 抓取完成序号 (1~6) -> 上位机 pick_task.complete() 依此推进抓取顺序 */
void Serial_SendPickDone(uint8_t seq);
/* 0x10: 第一批(前 3 次抓取)完成、可以进入第二批 -> 上位机只接受 batch=1 */
void Serial_SendNextBatch(uint8_t batch);

#endif
