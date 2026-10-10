/***************(C) COPYRIGHT SWUST_ROBOCON *************BEGIN OF FILE***************/
/**
  ******************************************************************************
  * @file      tjc_screen.c
  * @brief     TJC 串口屏 (陶晶驰 TJC4827T143) 驱动
  *            通信: USART6 @ PC6(TX)/PC7(RX), 115200 8N1
  *            协议: 控件属性赋值 + 结束符 0xFF 0xFF 0xFF
  *            依据: Chassis/串口屏显示开发交接说明.md
  ******************************************************************************
  */
#include "tjc_screen.h"
#include "stdio.h"

/* 上电延时: 屏上电后需等 300~500ms 再发指令 (文档第6节) */
#define SCREEN_BOOT_DELAY_TICKS   40   /* 40 x 10ms = 400ms */

/* ================= 底层: 阻塞式单字节发送 ================= */
static void TJC_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
    USART_SendData(USART6, b);
}

/* ================= 初始化 USART6 ================= */
void TJC_Init(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);   /* PC6 = USART6_TX */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);   /* PC7 = USART6_RX */

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStructure);

    USART_Cmd(USART6, ENABLE);
}

/* ================= 命令发送 ================= */
static void TJC_SendStr(const char *str)
{
    while (*str)
    {
        TJC_SendByte((uint8_t)(*str++));
    }
}

static void TJC_SendEnd(void)
{
    TJC_SendByte(0xFF);
    TJC_SendByte(0xFF);
    TJC_SendByte(0xFF);
}

static void TJC_SendCmd(const char *cmd)
{
    TJC_SendStr(cmd);
    TJC_SendEnd();
}

/* ================= 对外三个接口 (所有送屏操作都走这里) ================= */

void Screen_Init(void)
{
    char buf[64];

    /* 先清空屏接收状态, 再发正式指令 (文档第6节) */
    TJC_SendByte(0x00);
    TJC_SendEnd();

    sprintf(buf, "page0.t0.txt=\"----+----\"");
    TJC_SendCmd(buf);

    sprintf(buf, "page0.t2.txt=\"----+----\"");
    TJC_SendCmd(buf);

    sprintf(buf, "page0.t1.txt=\"OK:0/6 ERR:0/6\"");
    TJC_SendCmd(buf);
}

void Screen_ShowTaskCode(const char *code_t0, const char *code_t2)
{
    char buf[64];

    sprintf(buf, "page0.t0.txt=\"%s\"", code_t0);
    TJC_SendCmd(buf);

    sprintf(buf, "page0.t2.txt=\"%s\"", code_t2);
    TJC_SendCmd(buf);
}

/* 从二维码原始文本解析 4 组三位数 (自动忽略 '+'/空格等分隔符, 也兼容无分隔的 12 位数字),
 * 拆成 t0 = 第1+2组 "aaa+bbb" / t2 = 第3+4组 "ccc+ddd" 显示。
 * 例: "156+123+516+231" 或 "156123516231" -> t0="156+123" t2="516+231" */
void Screen_ShowQRCode(const char *raw)
{
    uint16_t g[4] = {0};
    uint8_t  gi = 0, d = 0;
    char     t0[16], t2[16];

    while (*raw && gi < 4)
    {
        if (*raw >= '0' && *raw <= '9')
        {
            g[gi] = g[gi] * 10 + (uint16_t)(*raw - '0');
            if (++d == 3) { gi++; d = 0; }
        }
        raw++;
    }

    if (gi == 4)
    {
        sprintf(t0, "%03d+%03d", (int)g[0], (int)g[1]);
        sprintf(t2, "%03d+%03d", (int)g[2], (int)g[3]);
        Screen_ShowTaskCode(t0, t2);
    }
}

void Screen_ShowCount(uint8_t ok, uint8_t err, uint8_t total)
{
    char buf[64];

    sprintf(buf, "page0.t1.txt=\"OK:%u/%u ERR:%u/%u\"",
            (unsigned)ok, (unsigned)total, (unsigned)err, (unsigned)total);
    TJC_SendCmd(buf);
}

void Screen_ShowZone(uint8_t zone)
{
    char buf[32];

    sprintf(buf, "page0.t1.txt=\"ZONE:%u\"", (unsigned)(zone == 2 ? 2 : 1));
    TJC_SendCmd(buf);
}

/* ================= 上电引导 (10ms 任务里调用) ================= */

void Screen_Boot(void)
{
    static uint16_t tick = 0;
    static uint8_t  done = 0;

    if (done) return;

    if (++tick < SCREEN_BOOT_DELAY_TICKS) return;

    Screen_Init();
    done = 1;
}

/* ================= 测试接口 (Keil Watch 里改) ================= */

uint16_t screen_test_a = 156, screen_test_b = 123, screen_test_c = 516, screen_test_d = 231;
uint8_t  screen_test_ok = 5, screen_test_err = 1, screen_test_total = 6;
uint8_t  screen_test_sel  = 0;   /* 0=任务码(t0/t2) 1=次数(t1) 2=占位 */
uint8_t  screen_test_trig = 1;   /* 写 1 触发一次, 发完自动清 0 */

void Screen_Test_Task(void)
{
    if (!screen_test_trig) return;
    screen_test_trig = 0;         /* 一次触发只发一次 */

    switch (screen_test_sel)
    {
    case 0:                       /* 任务码: t0 = a+b, t2 = c+d */
        {
            char buf0[32], buf2[32];
            sprintf(buf0, "%03d+%03d", (int)screen_test_a, (int)screen_test_b);
            sprintf(buf2, "%03d+%03d", (int)screen_test_c, (int)screen_test_d);
            Screen_ShowTaskCode(buf0, buf2);
        }
        break;

    case 1:                       /* 次数 -> t1 */
        Screen_ShowCount(screen_test_ok, screen_test_err, screen_test_total);
        break;

    case 2:                       /* 占位 -> 恢复开机内容 */
    default:
        Screen_Init();
        break;
    }
}

/* ================= 连续发送测试 (串口助手测 TX 用) ================= */

void Screen_SendTestLoop(void)
{
    static uint32_t tick = 0;
    static uint32_t cnt  = 0;
    char buf[32];

    if (++tick < 10) return;      /* 每 10 拍 = 100ms 发一次 */
    tick = 0;

    sprintf(buf, "TJC_TX_OK %lu\r\n", (unsigned long)cnt++);
    TJC_SendStr(buf);
}
