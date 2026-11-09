#include "bsp_can.h"

/**
  * @brief  ZDT X_V2 闭环步进电机 CAN 层
  *         500 kbps 扩展帧, 过滤器全收, FIFO0 + FMP0 中断
  */

__IO CAN_t can = {0};

/* EMM 定时主动上报位置快照表 (定义见 bsp_can.h) */
__IO uint32_t can_pos_raw[7]  = {0};
__IO uint8_t  can_pos_sign[7] = {0};
__IO uint16_t can_pos_cnt[7]  = {0};
__IO uint32_t can_rx_drop     = 0;
__IO uint32_t can_tx_drop     = 0;   /* 发送丢弃计数: 3 个邮箱全占满且超时放弃的帧数 */
__IO uint32_t can_rx_ovr      = 0;   /* RX FIFO0 溢出计数: 中断被压住太久导致硬件丢帧 (Watch 应恒 0) */

/* 电机母线电压回包 (0x24 单读 / 0x43 批量读系统状态第0包) 专用槽位 (定义见 bsp_can.h) */
__IO uint16_t can_vbus_mv[7] = {0};
__IO uint8_t  can_vbus_flag[7] = {0};
__IO uint32_t can_vbus_ok   = 0;

static void can_nvic_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CAN1_Init(void)
{
    GPIO_InitTypeDef      GPIO_InitStructure;
    CAN_InitTypeDef       CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    /* 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PA12 - CAN_TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA11 - CAN_RX */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    /* 500 kbps: 42M / 7 / (1 + 9 + 2) = 500k */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM      = DISABLE;
    CAN_InitStructure.CAN_ABOM      = ENABLE;   /* 自动离线管理: 总线出错进入 Bus-Off 后自动恢复 */
    CAN_InitStructure.CAN_AWUM      = DISABLE;
    CAN_InitStructure.CAN_NART      = DISABLE;
    CAN_InitStructure.CAN_RFLM      = DISABLE;
    CAN_InitStructure.CAN_TXFP      = ENABLE;   /* 发送顺序按"写入先后"(FIFO)。
                                                   原来是 DISABLE = 按报文 ID 仲裁, 广播同步触发帧
                                                   ExtId=0x0000 会插队到 0x0400(电机4) 之前,
                                                   导致 4 号轮永远晚一拍执行速度命令 (见 Chassis_SendWheels) */
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2       = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 7;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* 过滤器: 掩码 0 -> 全收 (扩展帧) */
    CAN_FilterInitStructure.CAN_FilterNumber         = 0;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    can_nvic_init();
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief   CAN 发送命令 (X_V2 协议分帧: 去掉地址字节和功能字节后的数据)
  * @param   cmd  命令字节流 (cmd[0]=地址, cmd[1]=功能码, 之后为数据)
  * @param   len  总字节数
  */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    CanTxMsg txMsg;   /* ★ 栈上局部: 不再共用全局 can.CAN_TxMsg。
                         SIGNALING(prio2, 塔吊5/6) 与 Chassis(prio4, 底盘1~4) 都走本函数,
                         高优先级任务若在拼包途中抢占, 会把全局缓冲改写成本任务的数据,
                         回去后接着写 -> 发出一帧地址与数据错配的报文。局部变量彻底消除覆盖。 */

    /* 去掉地址字节和功能字节后的数据长度 */
    j = len - 2;

    while (i < j)
    {
        k = j - i;

        txMsg.StdId   = 0x00;
        txMsg.ExtId   = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        txMsg.Data[0] = cmd[1];
        txMsg.IDE     = CAN_Id_Extended;
        txMsg.RTR     = CAN_RTR_Data;

        if (k < 8)
        {
            for (l = 0; l < k; l++, i++) { txMsg.Data[l + 1] = cmd[i + 2]; }
            txMsg.DLC = k + 1;
        }
        else
        {
            for (l = 0; l < 7; l++, i++) { txMsg.Data[l + 1] = cmd[i + 2]; }
            txMsg.DLC = 8;
        }

        /* 等一个空闲邮箱再发: CAN 只有 3 个发送邮箱, 连续发多帧时邮箱会占满。
           加超时: 若总线被某个节点堵死/离线, 邮箱一直不空, 到点就放弃本帧,
           避免整个任务卡死在这里 —— 否则排在后面的电机(如 6 号)命令永远发不出去。
           can_tx_drop 记次数: 正常应为 0, 变成非 0 说明总线/邮箱真的堵了 */
        {
            uint32_t tx_t = 0;
            while (CAN_Transmit(CAN1, &txMsg) == CAN_TxStatus_NoMailBox)
            {
                if (++tx_t > 200000) { can_tx_drop++; return; }   /* 超时: 放弃本帧及后续帧, 继续跑其他任务 */
            }
        }
        ++packNum;
    }
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg msg;                     /* 局部变量: 不再直接写共享缓冲 */
    uint8_t  addr;

    /* RX FIFO0 溢出检查: 硬件 FIFO 只有 3 帧深, 若本中断被更高优先级中断/临界区
       压住太久, 连着到的上报帧会把 FIFO 撑满, 硬件直接丢帧(丢的是后面到的那些)。
       这里清标志并计数: can_rx_ovr 恒 0, 才能排除"是 MCU 自己漏收", 把锅坐实到电机/线束侧。 */
    if (CAN1->RF0R & CAN_RF0R_FOVR0)
    {
        CAN1->RF0R |= CAN_RF0R_FOVR0;   /* 写 1 清零 */
        can_rx_ovr++;
    }

    CAN_Receive(CAN1, CAN_FIFO0, &msg);

    addr = (uint8_t)(msg.ExtId >> 8);

    /* 位置帧 0x36: 全部电机(底盘 EMM 1~4 + 塔吊 X_V2 5/6)都入按地址索引的专用槽
       can_pos_raw/can_pos_cnt, 不再走共享 rxFrameFlag —— 否则底盘 EMM 命令的 ACK
       回包会覆盖塔吊电机的 0x36 位置回包, Tower_ReadPos5/6 永久等不到就超时,
       卡死整个 SIGNALING 任务(云台跟着冻结)。解码在读取侧按电机区分:
       EMM 是 65536=360°, X_V2 是 ×0.1=度 (见 app_chassis.c / app_tower.c)。 */
    if (msg.Data[0] == 0x36u && msg.DLC == 7u)
    {
        if (addr >= 1u && addr <= MOTOR_ADDR_MAX)
        {
            can_pos_raw[addr]  = ((uint32_t)msg.Data[2] << 24) |
                                 ((uint32_t)msg.Data[3] << 16) |
                                 ((uint32_t)msg.Data[4] << 8)  |
                                  (uint32_t)msg.Data[5];
            can_pos_sign[addr] = msg.Data[1];
            can_pos_cnt[addr]++;      /* 最后写: 数据就绪标志 */
        }
        else
        {
            can_rx_drop++;            /* 位置帧地址越界 (0 或 >6) */
        }
    }
    /* 电压回包: 0x24 单读总线电压(Data[1..2]=16位mV大端) 或 0x43 批量读系统状态
       参数第0包(Data[3..4]=16位mV大端); 底盘1/塔吊5 各取自己的槽位 */
    else if (addr >= 1u && addr <= MOTOR_ADDR_MAX &&
             ((msg.Data[0] == 0x24u && msg.DLC >= 3u) ||
              (msg.Data[0] == 0x43u && (msg.ExtId & 0xFFu) == 0u && msg.DLC >= 5u)))
    {
        can_vbus_mv[addr]   = (msg.Data[0] == 0x24u)
                              ? (uint16_t)(((uint16_t)msg.Data[1] << 8) | (uint16_t)msg.Data[2])
                              : (uint16_t)(((uint16_t)msg.Data[3] << 8) | (uint16_t)msg.Data[4]);
        can_vbus_ok++;
        can_vbus_flag[addr] = 1;      /* 最后置标志 */
    }
    else
    {
        can.CAN_RxMsg   = msg;        /* 其它帧 (ACK 等): 保留旧 rxFrameFlag 路径 */
        can.rxFrameFlag = true;
    }

    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}
