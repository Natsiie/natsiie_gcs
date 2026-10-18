# AGENTS.md — STM32F407「工创小车」固件

> 给 AI 编码 agent 的工程约定。**动手改代码之前先读完这份文件。**
> 内容以当前代码为准（最后核对：2026-10-12）。参数改了这里也要同步改。

---

## 1. 一句话背景

一台 RoboMaster 风格的「闭环步进轮小车」：
**6 个 ZDT X_V2 闭环步进电机（CAN）+ 3 个舵机（TIM2 PWM）+ 维特 JY901 陀螺仪（USART2）+ ELRS/CRSF 遥控（UART3/UART5）**，跑 FreeRTOS。

任务周期：三任务全部 **10ms**（`vTaskDelayUntil`）。

---

## 2. 硬性技术事实（最容易搞错的地方）

| 项 | 值 |
|---|---|
| MCU | **STM32F407VG** @168MHz |
| 外设库 | **ST 标准外设库**（`stm32f4xx_*.h`）—— **不是 HAL，不要用 HAL 函数** |
| 编译器 | **Keil MDK / armcc（AC5）**，工程里 `<uAC6>0</uAC6>` |
| RTOS | FreeRTOS，port = **RVDS / ARM_CM4F** |
| 工程文件 | `Project/ROBOCON.uvprojx` |
| 构建产物 | `Output/Objects/ROBOCON.axf`（本身就是 ELF）、`Output/ROBOCON.hex` |

### ⚠️ AC5 的坑

- armcc **默认 C90**。C99 特性（`//` 行注释、语句中间声明变量、`for` 内声明）需要 `--c99`，**工程不一定开了**
- **不要**用 C11 语法（`_Static_assert`、匿名结构体）、**不要**用变长数组
- **不要**凭感觉写"应该能编过"的代码 —— **每次改完必须构建验证**

---

## 3. 构建（你验证改动的唯一手段）

```
Ctrl+Shift+B                      # VS Code 任务：Keil: Build
```

或命令行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\keil-build.ps1
```

底层真实命令：

```
D:\Keil5\UV4\UV4.exe -b Project\ROBOCON.uvprojx -j0 -o Project\build.log
```

**退出码**：`0`=无错无警告 · `1`=仅警告 · `2`=有错误 · `3`=致命

**注意事项**：
- 构建前确保 **Keil GUI 没有打开同一个工程**，否则 UV4 可能直接返回
- 编译错误/警告会进入 VS Code 问题面板，Error Lens 会显示在代码行尾 —— **agent 也能读到**

> **改完代码一定要跑构建。** 没有构建通过，就不要说你改好了。

---

## 4. 硬件接口映射（改之前必读）

| 资源 | 引脚 | 说明 |
|---|---|---|
| CAN1 | PA11(RX) / PA12(TX) | 500 kbps，**扩展帧 29bit**，过滤器掩码 0 全收，FIFO0 + `CAN_IT_FMP0` 中断 |
| 步进电机 1~4 | CAN 地址 **1/2/3/4** | 麦轮底盘 FL/FR/RL/RR |
| 步进电机 5 | CAN 地址 **5** (`MOTOR_HOIST`) | 塔吊升降 |
| 步进电机 6 | CAN 地址 **6** (`MOTOR_EXTEND`) | 塔吊伸出 |
| 舵机 | TIM2 — CH2=PA1 夹爪 · CH3=PA2 物料平台 · CH4=PA3 云台 | 50Hz，`CCR = angle/range*2000+500` |
| 陀螺仪 JY901 | USART2 PD5(TX)/PD6(RX) @115200 | 自带姿态融合，直接 `Imu_GetYaw()` |
| 遥控 ELRS/CRSF | UART3 / UART5 @**420000** | `ELRS_Remote_Update()` |
| VOFA 调试 | UART6 @**460800** | 串口调参 / 示波 |

---

## 5. 文件地图

| 文件 | 作用 | 改动频率 |
|---|---|---|
| `Apps/app_tower.c/.h` | 塔吊 + 舵机 + 抓取/放置流程 + 上电默认 | 🔴 **最常改** |
| `Apps/app_chassis.c/.h` | 麦轮底盘（逆运动学 + 偏航 PID） | 🟡 |
| `Apps/app_mission.c` · `app_nav.c` · `app_kin.c` | 任务 / 导航 / 运动学 | 🟡 |
| `BSP/bsp_can.c/.h` | CAN 层：`CAN1_Init` · `can_SendCmd` · `CAN1_RX0_IRQHandler` | 🟡 |
| `BSP/sys.c` | `driver_Init`：CAN/IMU/舵机/按键初始化（**调度器启动前**） | 🟡 |
| `BSP/bsp_usart.c` · `bsp_key.c` · `delay.c` | 串口 / 按键 / 延时 | 🟢 |
| `Devices/X_V2.c/.h` | 步进电机驱动（**约 2800 行**，命令全集） | ⛔ **一般不要改** |
| `Devices/imu.c` · `servo.c` · `crsf_protocol.c` · `elrs_Remote.c` · `tjc_screen.c` · `wit_c_sdk.c` | 外设驱动 | 🟢 |
| `User/car_config.h` | **集中参数**（地址 / 通道 / 灵敏度 / 角度） | 🟡 |
| `User/main.c` · `stm32f4xx_it.c` | 入口 / 中断向量 | 🟢 |
| `FreeRTOS/MyTasks.c` | 三任务循环体（**只做调度，逻辑在 App 里**） | 🟡 |
| `FreeRTOS/FreeRTOS_Init.c` | 建任务 | 🟢 |
| `Algorithms/PID.c` · `math_utils.c` | 算法 | 🟢 |
| `Library/` | ST 标准外设库 | ⛔ **不要改** |

---

## 6. 关键协议：X_V2 CAN 步进电机

- **发送**：扩展帧 `ID = (地址 << 8) | Packet`，数据 `[功能码, 数据…, 0x6B 校验]`
  - 单条命令 **> 8 字节要拆多帧**（`Packet` 从 0 递增）
- **响应**：`Data[0]` = 功能码或状态
  - `02`=成功 · `E2`=参数错 · `EE`=格式错 · `9F`=到位返回
- **读位置**：`X_V2_Read_Sys_Params(addr, S_CPOS)`（0x36）
  - 回包 `DLC=7`；`Data[1]`=符号（1=负），`Data[2..5]`=32 位**大端**位置，**×0.1 = 度**
- **位置命令**：`X_V2_Traj_Pos_Control(addr, dir, acc, dec, vel, pos, raf, snF)`（0xFD）
  - `vel ×10`、`pos ×10`，大端编码
  - `raf`：`0`=相对上次目标 · **`1`=绝对（相对零点，本项目用这个）** · `2`=相对当前位置
  - `snF`：`0`=立即执行 · `1`=缓冲
- **稳健性**：`CAN_ABOM` 已开（bus-off 自动恢复）；`can_SendCmd` 等邮箱带超时（约 20 万次循环），不会无限卡死

---

## 7. ⛔ 禁止事项（硬红线）

1. **不要修改 `Project/` 目录下任何文件** —— `.uvprojx` / `.uvoptx` / `.uvguix.*` / 启动文件 / `.sct` 分散加载
2. **不要修改 `Library/`**（ST 标准外设库源码）
3. **不要修改 `Devices/X_V2.c/.h`** —— 2800 行厂商驱动，除非我明确要求
4. **不要自动烧录、不要自动用 pyOCD/调试器写目标芯片** —— 那会刷进真实硬件。要烧录 **先问我**
5. **不要动 `Output/`、`Listings/` 及其他构建产物**
6. **不要 `git commit` / `git push`**，除非我明确要求
7. **不要为了"消除警告"而删掉你没看懂的代码** —— 先问

---

## 8. 编码约束（FreeRTOS + 中断）

- **中断服务函数里**：不许阻塞、不许 `malloc`/`free`、不许调用非 `FromISR` 版本的 FreeRTOS API
- 中断与任务共享的变量必须加 `volatile`；跨任务共享数据用队列 / 信号量，别用裸全局变量
- `CAN1_RX0_IRQHandler` 里保持短小 —— 别在里面做耗时解析
- **全局参数统一放 `User/car_config.h`**，不要散落魔法数字
- M4F 有 FPU，但**中断里用浮点要小心**（惰性保存上下文），能不用就不用
- 寄存器位定义以 **参考手册** 为准。**不确定就问我，不要猜寄存器名和位域**

---

## 9. 调试：让 AI 也能"看到"目标板

硬件调试原本只有人能看，这个工程做了脚本化，**agent 也能读到真实数据**：

工具是 **pyOCD**（装在独立 venv：`C:\Users\xszx1\.dsh\tools\pyocd`），**DAPLink(CMSIS-DAP) 和 ST-Link 都支持**。

| 目标 | 命令 |
|---|---|
| 看当前插着什么探针 | `pyocd list`（或 VS Code 任务「Debug: list connected probes」）|
| 读核心寄存器 + 外设 + HardFault 分析 | `powershell -NoProfile -ExecutionPolicy Bypass -File tools\pyocd\dump.ps1` |
| 导出符号表（查变量地址） | `D:\Keil5\ARM\ARMCC\bin\fromelf.exe --text -s -o Output\symbols.txt Output\Objects\ROBOCON.axf` |
| 读某个变量的值 | 先在 `Output\symbols.txt` 里查地址，再用 `dump_target.py` 的方式读内存 |

`tools/pyocd/dump_target.py` 会输出：

- Cortex-M4 核心寄存器（`pc` / `lr` / `sp` / `xpsr` / `r0`~`r3` / `r12`）
- **HardFault 自动分析**：解出 `CFSR` / `HFSR` 的**具体故障位**（如 `PRECISERR`、`IMPRECISERR`、`STKERR`、`DIVBYZERO`、`UNALIGNED`、`NOCP`），并直接给出 `MMFAR` / `BFAR` 故障地址
- RCC 时钟树状态
- GPIOA~E 的 `MODER` / `IDR` / `ODR`
- **TIM2** 的 `CNT` + `CCR1~4`（看舵机实际输出）
- **CAN1** 全部寄存器（看 bus-off / 错误计数 / 收件箱）

常用参数：

```
-Target stm32f407vg    # 默认就是它
-NoResume              # 读完不复位，停在 halt 状态方便连第二个脚本
-NoHalt                # 不停核（只读内存，寄存器可能不准）
```

> ⚠️ **运行前把车架起来（轮子离地）。** 调试器 halt 会让控制循环停住，电机保持最后一条命令。

> 💡 pyOCD 需要 CMSIS pack 才有 STM32F407 的目标定义，已经装好 `Keil.STM32F4xx_DFP.3.1.1`。
> 如果换了别的 MCU（比如 H7），要先 `pyocd pack install <型号>`。

### 你做不到的事（别硬撑）

- 你**看不到示波器、看不到实际机械运动**
- 涉及电机实际转不转、舵机角度对不对、CAN 总线上有没有设备 —— **必须由人在硬件上验证**
- 你的职责是：**代码正确、能编译、逻辑自洽**；然后告诉人"该怎么验证"

---

## 10. 推荐改动流程

1. 先读第 **4、5、6** 节，确认你要改的是哪一层
2. `git status` 确认工作区状态；**改之前先建分支或提交**
3. 改代码 → 跑 `Ctrl+Shift+B` 构建 → **确认 0 error**
4. 涉及新增文件时，**提醒人手动把文件加进 Keil 工程**（`.uvprojx` 由人维护，你不要碰）
5. 汇报格式：**改了什么 · 为什么 · 怎么验证的 · 还有哪些不确定性**
