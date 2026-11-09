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
    uint8_t  restart;          /* 写1=调试重启(同运行中长按一键键): 中止回 IDLE, 车要已搬回启动区; 处理完自动清0 */
    uint8_t  mat_grab;         /* 物料区手动抓取: 写1 -> 塔吊走 kin_flag.cmd=10 固定位完整装货(不走视觉), 执行时自动清0 */
    uint8_t  work_done;        /* 手动debug完成标志: MS_WAIT_MANUAL 里等 Watch 写 1, 消费后自动清 0 */
    uint8_t  state;            /* 只读: 主状态机当前状态 (值与 enum 顺序一致) */
    uint8_t  sub;              /* 只读: 当前状态内子步/第几个 */
    float    scan_goto_y;      /* 启动区扫码停点的 Y (mm), 两区共用; 停点 X 取 nav 表的 corner_x */
    uint16_t vision_timeout;   /* 工作区视觉超时(10ms/拍), 1000=10s; 超时视为当前点动作完成。
                                  !! 2026-10-31 临时关闭: app_mission.c 的看门狗只计数不放行, 这个字段目前不生效 */
    uint16_t trim_wait_10ms;   /* 进站等底盘视觉微调 within 的上限(10ms/拍), 500=5s; 等不到就放行 */
    uint16_t scan_pause_10ms;  /* 停在扫码停点的时长(10ms/拍), 100=1s; 停完直接回物料区 */

    /* ---- 诊断字段 (只读, 2026-11-02): 进 MS_POSE_DEFAULT 那拍看到的 kin_flag.done ----
       正常应恒为 0 (上一段链的 done 一定已经在 MS_LOAD/MS_UNLOAD/MS_STACK/MS_RELOAD 里被消费)。
       若某次看到 1, 说明 done 残留了 —— MS_POSE_DEFAULT 里"发 cmd=4"和"判 done"是同一拍的两个
       平级 if, 残留 1 会在同一拍直接跳 MS_TRIM -> 塔吊还没摆到探查姿态, 底盘视觉纠偏就先放开了
       (车会按收到的 corr 一点一点慢慢挪)。pose_early_cnt 是整场累计次数;
       长按/ mission.restart 重开一场时清零。 */
    uint8_t  done_at_pose;     /* 只读: 上一次进 MS_POSE_DEFAULT 时 kin_flag.done 的旧值 (0/1) */
    uint16_t pose_early_cnt;   /* 只读: done_at_pose==1 的累计次数 (=纠偏被提前放开的次数) */
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
/* 按二维码把 3 个槽位的落环填进 kin_tgt.drop_ring (cmd=15 卸货整链用; 没扫到码填 1/2/3) */
void Mission_ApplyQRDropRing(uint8_t round);
void Mission_ColorTick(void);                /* 10ms 房务: 处理 Watch 写的 clr=1 */

/* ================= 抓取完成上报 (0x0F PickDone / 0x11 PickColor / 0x10 NextBatch) ================= */
/* 上位机抓完一个必须收到 0x0F 才会把"期望颜色"推进到下一个; 收不到就一直等同一个颜色、把后面的全跳过。
   0x0F payload = 序号 1~6 (mission_pick_seq 逐件累加; 收到新任务码时清零, 和上位机 Pick.sequence 对齐),
   这是上位机唯一认的东西, 必须和它的 Pick.sequence 对齐。
   0x11 payload = 本件颜色号 1~6: 闭爪那拍从 0x08 抄回来的, 上位机暂不解析, 只在日志里显示, 用于两边对账。
   0x11 是"锦上添花": 那件 0x08 丢了 color 就是 0, 这时 0x0F 照发 (否则上位机永远等不到, 两边互锁), 只是不发 0x11。
   序号报到 3 之后再发 0x10 NextBatch("1") 放行第二批。 */
extern uint8_t mission_report_pick;   /* 1=上报 (默认); 写 0 整体关闭, 联调兜底 */
extern uint8_t mission_pick_seq;      /* 已上报序号 0~6; 决定下一件报几号和何时发 0x10, Watch 可改写对齐 */
void Mission_ReportPickDone(uint8_t color);   /* 一件抓完: 0x0F(序号, 无条件) + 0x13(颜色, 仅有效时), 第 3 件再补 0x10 */

/* ================= 放置完成回传 (0x0B) ================= */
/* 2026-11-09 起视觉改成批量模型: 识别源由 0x11 AlignDone stage 决定 (2=三圆环 3=叠料),
   0x0B 只是"本阶段整批放完"的信号 (supervisor placements_per_phase=1)。
   到站只调 NewTask 换 task_id (千万别发帧, 早发会让视觉提前切源、0x0C 的 5 帧均值被打断);
   整批卸/放完那拍调 Done (发 2 帧, 第一条计满, 第二条冗余)。 */
void Mission_PlaceFeedNewTask(void);
void Mission_PlaceFeedFirstLayer(void);   /* 旧接口, 新视觉下不要用 */
void Mission_PlaceFeedDone(void);
void Mission_PlaceFeedStacked(void);      /* 旧接口 (台架切叠料源), 不要用 */

void Mission_Update(uint8_t key_pressed);   /* 10ms 主状态机, 在 DATE_Tasks 里调 */

#endif /* __APP_MISSION_H */
