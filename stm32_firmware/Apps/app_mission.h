#ifndef __APP_MISSION_H
#define __APP_MISSION_H

#include "stm32f4xx.h"

/**
  * @brief  全自动比赛流程主状态机 (总控)
  *         mission.enable=0 时完全不介入, 保留原「1功能测试代码」。
  *         跑在 DATE_Tasks (10ms), 通过 nav_cmd / kin_flag.cmd 协调底盘与塔吊。
  */

/* ================= 全自动流程打包变量 (Keil Watch 只看 mission) ================= */
typedef struct {
    uint8_t  enable;           /* 0=保留单功能测试代码; 1=全自动比赛流程 (Watch 改) */
    uint8_t  mode_load;        /* 装货(物料区) 模式: 0=视觉自动 1=手动debug 2=跳过 */
    uint8_t  mode_unload;      /* 卸货(粗加工区) 模式 */
    uint8_t  mode_reload;      /* 回装(粗加工区) 模式 */
    uint8_t  mode_stack;       /* 堆叠(暂存区) 模式 */
    uint8_t  skip_work;        /* 写1=跳过当前工作区塔吊作业直接离站, 跳完自动清0 */
    uint8_t  mat_grab;         /* 物料区手动抓取: 写1 -> 塔吊走 kin_flag.cmd=10 固定位完整装货(不走视觉), 执行时自动清0 */
    uint8_t  work_done;        /* 手动debug完成标志: MS_WAIT_MANUAL 里等 Watch 写 1, 消费后自动清 0 */
    uint8_t  state;            /* 只读: 主状态机当前状态 (值与 enum 顺序一致) */
    uint8_t  sub;              /* 只读: 当前状态内子步/第几个 */
    float    scan_goto_x;      /* 启动区1: 先去的点 X (mm); 启动区2 取 -50 */
    float    scan_goto_y;      /* 两区共用: 先去的点 Y (mm) */
    float    scan_stop_x;      /* 启动区1: 慢扫 |X| 上限 (mm); 启动区2 取 -1500 */
    uint16_t vision_timeout;   /* 工作区视觉超时(10ms/拍), 1000=10s; 超时视为当前点动作完成 */
    uint16_t trim_wait_10ms;   /* 进站等底盘视觉微调 within 的上限(10ms/拍), 500=5s; 等不到就放行 */
    uint16_t scan_pause_10ms;  /* 扫码直行到 corner x 暂停时长(10ms/拍), 200=2s */
} Mission_t;

extern Mission_t mission;

/* ================= 物料颜色对账打包变量 (Keil Watch 只看 mission_color) ================= */
/* 取色时机: 物料区每闭一次夹爪 = 拿到一件物料, app_kin 调 Mission_NoteGrabColor() 记一件。
   手动 cmd=10 / cmd=12 连抓、全自动 cmd=1 装货都记, 不依赖 mission 状态机。
   seq 按抓取顺序: seq[0]=第1件 ... seq[5]=第6件; 下标 i 对应的二维码要求是 expected[i/3][i%3]。
   颜色号 1=红 2=黄 3=蓝 4=绿 5=黑 6=浅蓝; 0=这件没记到。 */
typedef struct {
    uint8_t qr_colors_ok;      /* 只读: 1=二维码颜色组可信(12位全是1~6), 0=不参与对账 */
    uint8_t ok;                /* 只读: 实际颜色和二维码对上的件数 (收车上屏 OK) */
    uint8_t err;               /* 只读: 对不上的件数 (收车上屏 ERR) */
    uint8_t total;             /* 只读: 总件数 2轮x3件=6 (上屏分母) */
    uint8_t cnt;               /* 只读: 已记录的件数 (0..6) */
    uint8_t clr;               /* 写1=清空重开一场 (cnt/seq/ok/err 全清), 处理完自动清0 */
    uint8_t expected[2][3];    /* 只读: 二维码要求的颜色顺序 (第1组=第1轮, 第3组=第2轮) */
    uint8_t seq[6];            /* 只读: 按抓取顺序实际抓到的颜色 (0=这件还没抓到) */
} MissionColor_t;

extern MissionColor_t mission_color;

void Mission_NoteGrabColor(uint8_t color);   /* 物料区闭夹爪时调 (app_kin 里调): 记一件颜色并对账 */
void Mission_ColorTick(void);                /* 10ms 房务: 处理 Watch 写的 clr=1 */

/* ================= 抓取完成上报 (0x0F PickDone / 0x11 PickColor / 0x10 NextBatch) ================= */
/* 上位机抓完一个必须收到 0x0F 才会把"期望颜色"推进到下一个; 收不到就一直等同一个颜色、把后面的全跳过。
   0x0F payload = 序号 1~6 (轮*3+第几个+1), 这是上位机唯一认的东西, 必须和它的 Pick.sequence 对齐。
   0x11 payload = 本件颜色号 1~6: 闭爪那拍从 0x08 抄回来的, 上位机暂不解析, 只在日志里显示, 用于两边对账。
   序号报到 3 之后再发 0x10 NextBatch("1") 放行第二批。 */
extern uint8_t mission_report_pick;   /* 1=上报 (默认); 写 0 整体关闭, 联调兜底 */
extern uint8_t mission_pick_seq;      /* 已上报序号 0~6; 决定下一件报几号和何时发 0x10, Watch 可改写对齐 */
void Mission_ReportPickDone(uint8_t color);   /* 一件抓完: 0x0F(序号) + 0x11(颜色), 第 3 件再补 0x10 */

void Mission_Update(uint8_t key_pressed);   /* 10ms 主状态机, 在 DATE_Tasks 里调 */

#endif /* __APP_MISSION_H */
