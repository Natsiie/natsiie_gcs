#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdint.h>

/* ============================================================
 *  涓婁綅鏈(natsiie_gcs/ROS) <-> 涓嬩綅鏈(STM32F4) 涓插彛鍗忚
 *
 *  鐗╃悊灞: USART1, PA9=TX, PA10=RX, 115200, 8N1, 鏃犳牎楠
 *  甯ф牸寮: 0xFF | TYPE | LEN | PAYLOAD(UTF-8) | CRC8 | 0xFE
 *    - LEN  = payload 瀛楄妭鏁 (0~255)
 *    - CRC8 = TYPE ^ LEN ^ PAYLOAD 閫愬瓧鑺傚紓鎴
 *    - 绫诲瀷鍙蜂笌涓婁綅鏈 identify_pack/serial/protocol.py 鐨 TOPIC_TYPES 涓鑷
 *  鎺ユ敹: RXNE 涓鏂 + 甯х粨鏋勭姸鎬佹満, 鏀舵弧涓甯х疆 Serial_RxFrameReady=1,
 *        浠诲姟閲岃皟 Serial_ProcessFrame() 鎸 type 瑙ｆ瀽銆
 * ============================================================ */

#define SERIAL_HEAD  0xFF
#define SERIAL_TAIL  0xFE

/* 甯х被鍨 (0x07 鏄涓嬩綅鏈->涓婁綅鏈虹殑鍙嶉, 涓嶆帴鏀) */
#define SERIAL_TYPE_COLOR         0x01   /* 鑹插潡鍍忕礌鍒楄〃: id:cx,cy/id:cx,cy/... */
#define SERIAL_TYPE_COLOR_MOTION  0x02   /* 杞鐩樿繍鍔: state:speed(px/s) */
#define SERIAL_TYPE_TURNTABLE     0x03   /* 杞鐩: cx,cy,radius,stable */
#define SERIAL_TYPE_RING_STATION  0x04   /* 涓夊渾鐜宸ヤ綅: err_x,err_y,ang,corr_x,corr_y,corr_yaw,within */
#define SERIAL_TYPE_STACKED_RING  0x05   /* 鍙犳斁浣: 鍚 0x04 */
#define SERIAL_TYPE_QR_DECODE     0x06   /* QR 浠诲姟鐮佸瓧绗︿覆 */
#define SERIAL_TYPE_GIMBAL_YAW    0x07   /* 浜戝彴瑙掑害鍙嶉 (涓嬩綅鏈->涓婁綅鏈) */
#define SERIAL_TYPE_COLOR_TARGET  0x08   /* 棰滆壊鐩鏍囩偣: color_id,x_mm,y_mm,z_mm */
#define SERIAL_TYPE_GRAB_CMD      0x09   /* 澶圭埅鍒颁綅鎺堟潈: payload "1"=鎺堟潈涓嬮檷鎶撳彇 */
#define SERIAL_TYPE_PLACE_COMMAND 0x0A
#define SERIAL_TYPE_PLACE_RESULT  0x0B
#define SERIAL_TYPE_UNLOAD_COORD  0x0C   /* 鍗歌揣鍧愭爣: 3 缁 x,y (3 涓钀界偣, 濉斿悐绯荤粷瀵 mm) */
#define SERIAL_TYPE_PLACE_CONTROL 0x0D   /* 鏀剧疆闃舵垫帶鍒 (涓嬩綅鏈->涓婁綅鏈): start,stage,batch 鎴 retry,command_id */
#define SERIAL_TYPE_TURNTABLE_ALIGN 0x0E /* 杞鐩樺渾蹇冨归綈 (涓婁綅鏈->涓嬩綅鏈): corr_x,corr_y,within */
                                         /* 鐗╂枡鍖哄簳鐩樺井璋: 鍙鏈 3 涓瀛楁, 鏃 err/鏃 yaw銆
                                          * 杞村悜涓 0x04/0x05 鐨 corr 瀹屽叏涓鑷 (鍚屼竴寮 chassis_image_to_motion
                                          * 鐭╅樀绠楀嚭鏉ョ殑), corr_yaw 鎭掍负 0 -> 搴曠洏閿佸綋鍓嶈埅鍚戜笉杞銆
                                          * 杞鐩樿妭鐐规槸甯稿紑鐨, 琛岃溅閫斾腑涔熶細鍙, 鎵浠ュ崟鐙鐢
                                          * Serial_TurntableTrimReady + trim_armed 闂ㄦ帶, 鍙鍦ㄧ珯涓婃﹁呭悗鎵嶆敹銆 */
#define SERIAL_TYPE_PICK_DONE     0x0F   /* 鎶撳彇瀹屾垚搴忓彿 (涓嬩綅鏈->涓婁綅鏈): ASCII 搴忓彿 1~6 */
#define SERIAL_TYPE_NEXT_BATCH    0x10   /* 绗涓鎵 3 娆℃姄鍙栧畬鎴愩佸洖瑁呭氨缁 (涓嬩綅鏈->涓婁綅鏈): "1" */
#define SERIAL_TYPE_ALIGN_DONE    0x11   /* 区域矫正完成 (下位机->上位机): stage,success; 1=物料转盘 2=粗加工三环 3=暂存叠料 */
#define SERIAL_TYPE_PICK_COLOR    0x13   /* 抓取颜色 (下位机->上位机): 1~6; 0x11 已给视觉 AlignDone, 颜色挪到 0x13 */

/* ================= AI PID 自动调参专有协议 (0x20~0x23) ================= */
#define SERIAL_TYPE_PID_SET       0x20   /* 上位机->下位机: 设定参数 payload: type,val1,val2,val3 */
#define SERIAL_TYPE_PID_STEP      0x21   /* 上位机->下位机: 触发阶跃测试 payload: type,target,duration_ms */
#define SERIAL_TYPE_PID_DATA      0x22   /* 下位机->上位机: 阶跃打点数据回传 */
#define SERIAL_TYPE_PID_ACK       0x23   /* 下位机->上位机: 命令确认 */
                                         /* 涓婁綅鏈哄綋鍓嶄笉娑堣垂姝ょ被鍨, 鍙鎵撲竴鏉¤妭娴佺殑"蹇界暐鍙嶉堝抚" */
                                         /* 璀﹀憡 -> 浣滅敤鏄鍦ㄨ嗚夋棩蹇楅噷鑳界洿鎺ョ湅鍒版姄鐨勬槸鍝涓棰滆壊 */

#define SERIAL_RX_PAYLOAD_MAX     128    /* Includes UUID and placement XYZ. */

typedef struct {
    char task_id[33];
    uint32_t command_id;
    uint8_t stage;      /* 1=ordinary workstation, 2=stacking area */
    uint8_t batch, sequence, color, ring, layer;
    float x_mm, y_mm, z_mm;  /* gimbal_base; not the Kin ground-origin frame */
} Serial_PlaceCommand_t;

/* 鏃 TX 鏁版嵁鍖 (Serial_SendPacket 鐢, 8 瀛楄妭) */
#define SERIAL_TX_DATA_LEN        8
extern uint8_t Serial_TxPacket[SERIAL_TX_DATA_LEN];

/* 鎺ユ敹鍘熷嬪抚 (ISR 濉, 浠诲姟娑堣垂) */
typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t payload[SERIAL_RX_PAYLOAD_MAX + 1];   /* +1 瀛 '\0' */
} Serial_RxFrame_t;

/* 瑙ｆ瀽缁撴灉 (Serial_ProcessFrame 鎸 type 濉鍏, 涓氬姟灞傝) */
typedef struct {
    uint8_t type;                       /* 鏈杩戜竴甯х殑 type */

    /* 0x08 ColorTarget */
    uint8_t target_color_id;
    float   target_x_mm, target_y_mm, target_z_mm;

    /* 0x09 GrabCmd */
    uint8_t grab_cmd;                   /* 1=鏈変竴甯ф湭娑堣垂鐨勬姄鍙栨巿鏉 (app_kin 闂鐖鏃舵竻0) */
    uint16_t grab_cmd_cnt;              /* 鏀跺埌杩囩殑 0x09 甯ф暟 (鍙澧: Watch 鐪嬪畠娑=瑙嗚夊張鍙戜簡涓娆) */

    /* 0x02 ColorMotion */
    uint8_t motion_state;
    float   motion_speed;

    /* 0x03 Turntable */
    float   turntable_cx, turntable_cy, turntable_r;
    uint8_t turntable_stable;

    /* 0x04 / 0x05 鍦嗙幆宸ヤ綅 (鍋忕Щ鍧愭爣) */
    float   ring_err_x, ring_err_y, ring_ang_err;
    float   ring_corr_x, ring_corr_y, ring_corr_yaw;
    uint8_t ring_within;

    /* 0x0C 鍗歌揣鍧愭爣 (3 涓钀界偣, 濉斿悐绯荤粷瀵 mm) */
    float   unload_x[3], unload_y[3];
    uint8_t unload_count;

    /* 0x01 Color 鍍忕礌鍒楄〃 (鏈澶 6 娈) */
    uint8_t color_count;
    uint8_t color_id[6];
    int16_t color_cx[6], color_cy[6];

    /* 0x06 QR 浠诲姟鐮佸瓧绗︿覆 */
    char    str[SERIAL_RX_PAYLOAD_MAX + 1];
} Serial_RxData_t;

extern volatile uint8_t Serial_RxFrameReady;  /* 1 = 鏀跺埌瀹屾暣涓甯у緟澶勭悊 */
extern Serial_RxFrame_t  Serial_RxFrame;
extern Serial_RxData_t   Serial_RxData;

/* 0x0C 鍗歌揣鍧愭爣(3 缁 xy) 甯у凡瑙ｆ瀽 -> 濉斿悐鍗歌揣钀界偣; 0x04/0x05 鍋忕Щ鍧愭爣 corr -> 搴曠洏寰璋冦
 * 姣忔敹鍒板瑰簲甯х疆 1, 鐢变笟鍔″眰娑堣垂鍚庢竻 0 (Kin 娓 UnloadCoord, 搴曠洏娓 ChassisTrim)銆 */
extern volatile uint8_t Serial_UnloadCoordReady;  /* 1 = 鏂板嵏璐у潗鏍(0x0C 3 缁 xy) 寰呮秷璐 */
extern volatile uint8_t Serial_ChassisTrimReady;  /* 1 = 鏂板簳鐩樺井璋(0x04/0x05 corr) 寰呮秷璐 */
extern volatile uint8_t Serial_TurntableTrimReady;/* 1 = 鏂拌浆鐩樼籂鍋(0x0E corr) 寰呮秷璐 */
                                                  /* 涓 0x04/0x05 鍒嗗紑涓涓鏍囧織: 0x0E 鐨勫彂閫佹柟(杞鐩樿妭鐐)
                                                   * 鏄甯稿紑鐨, 琛岃溅閫斾腑涔熷彂甯, 蹇呴』閰嶅悎 trim_armed 鎵嶆敹;
                                                   * 0x04/0x05 淇濇寔鍘熻屼负涓嶅彈褰卞搷銆 */

/* 0x08 棰滆壊鐩鏍囩偣 / 0x06 浜岀淮鐮: 渚涘叏鑷鍔ㄦ祦绋(mission) 娑堣垂 */
extern volatile uint8_t Serial_ColorTargetReady;  /* 1 = 鏂伴滆壊鐩鏍囩偣(0x08) 寰呮秷璐 (mission 娓) */
extern volatile uint8_t Serial_QRReady;           /* 1 = 鏂颁簩缁寸爜(0x06) 寰呰В鏋 (mission 娓) */

/* 宸ヤ綔瑙嗚夋寚浠(0x04/05/08/09/0C)鏂板抚鏍囧織: 渚 mission 瑙嗚夎秴鏃剁湅闂ㄧ嫍 (mission 娓) */
extern volatile uint8_t Serial_VisionCmd;

void     Serial_Init(void);                        /* 鍒濆嬪寲 USART1            */
void     Serial_SendByte(uint8_t byte);            /* 闃诲炲彂閫 1 瀛楄妭          */
void     Serial_SendArray(uint8_t *array, uint16_t len);
void     Serial_SendString(char *str);
void     Serial_SendPacket(void);                  /* 0xFF + 8 瀛楄妭 + 0xFE     */
void     Serial_SendGimbalYaw(float angle_deg);    /* 浜戝彴瑙掑害 -> 涓婁綅鏈 (0x07) */
void     Serial_ProcessFrame(void);                /* 浠诲姟閲岃皟鐢: 鎸 type 瑙ｆ瀽涓甯 */
/* Poll once when ready to execute. Complete only after release and retreat.
 * These functions retain/deduplicate commands; they do not drive actuators.
 * Both must run in the same task as Serial_ProcessFrame(). */
uint8_t Serial_TakePlaceCommand(Serial_PlaceCommand_t *out);
void Serial_CompletePlaceCommand(const char *task_id, uint32_t command_id, uint8_t success);
/* 0x0B 原始发送 (不受 0x0A 队列约束): 只在"整批卸/放完"那一刻发, 视觉 supervisor
 * 收到一条成功帧就推进下一阶段 (placements_per_phase=1)。task_id 必须 32 位小写 hex。 */
void Serial_SendPlaceResultRaw(const char *task_id, uint32_t command_id, uint8_t success);
void Serial_SendPlaceControlStart(uint8_t stage, uint8_t batch);  /* 0x0D: start,<stage>,<batch> */
void Serial_SendPlaceControlRetry(uint32_t command_id);           /* 0x0D: retry,<command_id> */
/* 0x0F: 鎶撳彇瀹屾垚搴忓彿 1~6 -> 涓婁綅鏈 pick_task.complete() 渚濇ゆ帹杩"鏈熸湜棰滆壊"
 * (涓嶅彂灏变竴鐩寸瓑鍚屼竴涓棰滆壊; 搴忓彿/鐘舵佸逛笉涓婁細琚蹇界暐)銆俻ayload 鍙鑳芥槸绾 ASCII 鏁板瓧銆 */
void Serial_SendPickDone(uint8_t seq);
/* 0x10: 绗涓鎵(鍓 3 娆℃姄鍙)瀹屾垚銆佸彲浠ヨ繘绗浜屾壒 -> 涓婁綅鏈哄彧鎺ュ彈 payload "1" */
void Serial_SendNextBatch(uint8_t batch);
/* 0x13: 抓取颜色 1~6 (下位机从 0x08 抄回来的) -> 视觉暂不消费, 只在日志里显示;
   0x11 现在是 AlignDone, 颜色别再走 0x11, 否则视觉会当非法 AlignDone 丢掉 */
void Serial_SendPickColor(uint8_t color);

/* ================= 区域矫正完成回传 (0x11, 对齐视觉 race_supervisor) ================= */
/* 每进站(Chassis_TrimArm)最多回一条: 底盘判到位(视觉 within 或 自判 trim_conv)那一拍发 success=1,
   视觉收到就启动下一阶段 (物料区->颜色识别, 粗加工/暂存->放置)。失败不发 (视觉收到 0 只打日志继续等)。
   Serial_AlignDoneEn: 1=开(默认) 0=关 (单独调试视觉时静音)。 */
extern uint8_t Serial_AlignDoneEn;
extern volatile uint8_t Serial_TrimFrameType;   /* 最近一帧纠偏帧类型 (0x04/0x05/0x0E), 发 AlignDone 时定 stage */
void Serial_SendAlignDone(uint8_t stage, uint8_t success);


/* PID 自动调参交互接口 */
void Serial_SendPidAck(uint8_t ok, const char *msg);
void Serial_PidTune_Tick(void);
void Serial_FeedByte(uint8_t b);

#endif
