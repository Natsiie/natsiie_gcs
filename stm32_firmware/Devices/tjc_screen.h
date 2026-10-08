#ifndef __TJC_SCREEN_H
#define __TJC_SCREEN_H

#include "stm32f4xx.h"

/**
  * @brief  TJC 串口屏 (陶晶驰 TJC4827T143, T1 系列) 驱动
  *         通信: USART6, PC6 = TX / PC7 = RX, 115200 8N1
  *         协议: 控件属性赋值字符串 + 结束符 0xFF 0xFF 0xFF
  *         页面 page0: t0/t2 = 任务码, t1 = 正确/错误次数
  *         依据: Chassis/串口屏显示开发交接说明.md
  *
  *         注意:
  *         - 屏必须 5V 独立供电 (别和舵机共用细线), 共地, TX/RX 交叉。
  *         - 只发数字与 ASCII (0-9 + / : . - G P), 不发中文。
  *         - 字体高度 >=12mm 由屏侧工程保证, 代码不处理。
  */

void TJC_Init(uint32_t baud);              /* 初始化 USART6 (115200) */
void Screen_Init(void);                    /* 上电写占位内容 (----+---- / OK:0/6 ERR:0/6) */
void Screen_ShowTaskCode(const char *code_t0, const char *code_t2);/* 显示任务码 (t0=code_t0, t2=code_t2) */
void Screen_ShowQRCode(const char *raw);   /* 解析二维码原始文本(4组三位数) -> t0=前两组 / t2=后两组 */
void Screen_ShowCount(uint8_t ok, uint8_t err, uint8_t total); /* 显示正确/错误次数 (t1) */
void Screen_Boot(void);                    /* 10ms 任务里调, 上电延时约400ms后自动 Screen_Init() 一次 */

/* ================= 测试接口 (Keil Watch 里改, 由 Screen_Test_Task 发送) ================= */
extern uint16_t screen_test_a, screen_test_b, screen_test_c, screen_test_d; /* 任务码四组 (0~999) */
extern uint8_t  screen_test_ok, screen_test_err, screen_test_total;         /* 正确/错误/总次数 */
extern uint8_t  screen_test_sel;   /* 0=任务码(t0/t2) 1=次数(t1) 2=占位(Screen_Init) */
extern uint8_t  screen_test_trig;  /* 写 1 触发发送一次, 发完自动清 0 */
void Screen_Test_Task(void);        /* 10ms 任务里调 */
void Screen_SendTestLoop(void);     /* 连续发送测试: 每100ms发 "TJC_TX_OK N" (串口助手测TX用) */

#endif /* __TJC_SCREEN_H */
