# 串口协议（上位机 ROS ↔ 下位机 STM32F4）

> **这份文件是唯一事实来源。** 改协议的固定顺序：① 先改这里 → ② 再改两边代码 → ③ PR 里对方确认。
> 代码位置：下位机 `stm32_firmware/Devices/Serial.h` / `Serial.c`；上位机 `src/identify_pack/identify_pack/serial/protocol.py`
> 最后核对：2026-10-13（main `4f15747`）

## 1. 物理层与帧格式

- **USART1**：PA9 = TX，PA10 = RX，**115200 8N1**，无校验
- **帧结构**

```
+------+------+------+------------------+------+------+
| 0xFF | TYPE | LEN  |  PAYLOAD(UTF-8)  | CRC8 | 0xFE |
+------+------+------+------------------+------+------+
```

- `LEN` = payload 字节数；固件接收缓冲 `SERIAL_RX_PAYLOAD_MAX = 128`，**上位机请保证 ≤128**
- `CRC8` = `TYPE ^ LEN ^ payload` 逐字节异或
- 坏帧处理：丢弃 1 字节后重新寻找帧头，不阻塞后续合法帧（两侧实现一致）

## 2. 类型号总表

方向：**下→上** = STM32 → ROS；**上→下** = ROS → STM32

| TYPE | 名称 | 方向 | PAYLOAD 格式 | 语义 | 固件 | 视觉 |
|---|---|---|---|---|---|---|
| 0x01 | Color | 上→下 | `id:cx,cy/id:cx,cy/...` | 色块像素列表（最多 6 段） | 收 ✓ | 发 ✓ |
| 0x02 | ColorMotion | 上→下 | `state:speed` | 转盘运动状态 / 速度(px/s) | 收 ✓ | 发 ✓ |
| 0x03 | Turntable | 上→下 | `cx,cy,radius,stable` | 转盘圆心、半径、稳定标志 | 收 ✓ | 发 ✓ |
| 0x04 | ring_station | 上→下 | `err_x,err_y,ang,corr_x,corr_y,corr_yaw,within` | 三圆环工位偏差 + 底盘微调量 | 收 ✓ | 发 ✓ |
| 0x05 | stacked_ring | 上→下 | 同 0x04 | 叠放位偏差 + 底盘微调量 | 收 ✓ | 发 ✓ |
| 0x06 | QR_Decode | 上→下 | 任务码字符串 | 二维码任务码（下位机解析并上屏） | 收 ✓ | 发 ✓ |
| 0x07 | GimbalYaw | 下→上 | ASCII 角度(deg) | 云台角度反馈 | 发 ✓ | 收 ✓ |
| 0x08 | ColorTarget | 上→下 | `color_id,x_mm,y_mm,z_mm` | 颜色目标点（gimbal_base 系） | 收 ✓ | 发 ✓ |
| 0x09 | GrabCmd | 上→下 | `1` | 夹爪到位授权下降抓取 | 收 ✓ | 发 ✓ |
| 0x0A | PlaceCommand | 上→下 | `task_id,command_id,stage,batch,sequence,color,ring,layer,x,y,z` | 放置任务下发（stage: 1=普通工位 2=叠放） | 收 ✓ | 发 ✓ |
| 0x0B | PlaceResult | 下→上 | `task_id,command_id,success` | 放置结果回报 | 发 ✓ | 收 ✓ |
| 0x0C | ⚠ 见 §3.3 | 上→下 | 3 组 `x,y` | **固件叫「卸货坐标（塔吊系绝对 mm）」/ 上位机叫「RingTarget 三圆环圆心（gimbal_base mm）」——语义待统一** | 收 ✓ | 发 ✓ |
| 0x0D | PlaceControl | 下→上 | `start,stage,batch` 或 `retry,command_id` | 放置阶段控制（下位机请求上位机开始/重试） | 发 ✓ | 收 ✓ |
| 0x0E | TurntableAlign | 上→下 | `corr_x,corr_y,within(0/1)` | 转盘圆心对齐纠偏 | **未实现** ✗ | 无发送方 ✗ |
| 0x0F | PickDone | 下→上 | 序号 `1~6` | 抓取完成（上位机 `/arm/pick_done`）；序号 = `轮*3 + 本工作区第几个 + 1` | 发 ✓ | 收 ✓ |
| 0x10 | NextBatch | 下→上 | `1` | 第一批（前 3 次抓取）完成、回装就绪（上位机 `/arm/next_batch`） | 发 ✓ | 收 ✓ |
| 0x11 | AlignDone | 下→上 | `stage,success` | 底盘区域矫正完成（1=转盘 2=三圆环 3=叠料） | 发 ✓ | 收 ✓ |
| 0x12 | RegionReady | 下→上 | `stage` | 已到点且云台动作完成（2=三圆环 3=叠料）；上位机收到后才开始圆环测量并发 0x0C | 本地固件已加，需烧录 | 收 ✓ |

> 类型号来源：固件 `Devices/Serial.h` 的 `SERIAL_TYPE_*`；上位机 `serial/protocol.py` 的 `TOPIC_TYPES`。两侧必须一致。

## 3. 现存缺口（2026-10-13 核对）

1. **0x0F PickDone / 0x10 NextBatch —— 已实现（2026-10-13，固件侧）**
   固件新增 `Serial_SendPickDone()` / `Serial_SendNextBatch()`（`Devices/Serial.c`），上报点在
   `Apps/app_mission.c` 的 `MS_LOAD`：每次装货抓取完成上报序号 `轮*3 + 本工作区第几个 + 1`
   （第 1 轮 1/2/3、第 2 轮 4/5/6）；第 1 轮 3 次抓完后发 `NextBatch(1)`。
   与上位机 `pick_task.py` 的 `Pick.sequence = batch*3 + index + 1`、`next_batch(1)` 一一对应。
   **兜底开关**：`mission_report_pick`（Watch 可置 0 整体关闭上报）。
   ⚠ 尚未实机验证；当前按"Kin 动作完成"即上报，**不区分是否真的夹住物料**。

2. **0x0E TurntableAlign —— 已占号但未启用**
   上位机只在 `protocol.py` 里定义了类型号，**没有发送方**；固件 `Serial_ProcessFrame()` 也没有对应 `case`。属于"预留位"。

3. **0x0C 坐标系声明不一致 —— 需要你们拍板**
   payload 形状两侧一致（`x1,y1,x2,y2,x3,y3`，上位机发、固件收），固件把收到的值**直接当 Kin 目标**用
   （`Serial_RxData.unload_x/y` → `kin_unload_x/y` → `kin_target_x/y`）。但注释里声明的坐标系不同：
   - 上位机：`RingTarget`，三圆环圆心，**gimbal_base 毫米**
   - 固件：`SERIAL_TYPE_UNLOAD_COORD`，卸货坐标 3 组 xy，**塔吊系绝对 mm**

   同文件里 0x0A `PlaceCommand` 的注释明确写着 `gimbal_base; not the Kin ground-origin frame`，
   说明这两个系确实不是一回事。
   → **要确认 0x0C 到底属于哪个系**。如果两侧不一致，落点会带固定偏移，而现场调出来的
   `kin_unload_x_offset_mm / kin_unload_y_offset_mm` 可能正好把这个偏差掩盖了。
   定稿后把两侧注释一起改掉。

4. **文档指针失效**：`protocol.py` 顶部写"必须与 STM32F4 侧 `serial_protocol.h` 保持一致"，实际文件是 `Devices/Serial.h`。

## 4. 变更流程（两条规矩）

1. **新增/修改类型号**：先改本表 → 再改两边代码 → 开 PR（模板见 `.github/PULL_REQUEST_TEMPLATE.md`，必须勾"是否影响对方"）。
2. **不允许复用已退役的类型号**，也不要"先塞个号以后再补文档"。

## 5. 提案：0x00 HELLO 版本握手（尚未实现）

- payload：`proto_ver,fw_ver`，例如 `1,3.4.2`
- 双方启动后各发一次，并在日志里打印收到的对端版本；**版本不一致就直接报错停机**
- 目的：消灭"我以为你那边是新的"这类扯皮，联调一开场就能确认两端是不是同一份协议

## 6. 联调记录约定

**抓一段串口日志存档**（联调时后台开着，结束后日志进 `Output/serial.log`）：

```powershell
C:\Users\xszx1\.dsh\tools\pyocd\Scripts\python.exe tools\serial_monitor.py --port COM3 --log Output\serial.log
# 只看本次关心的帧:  --types 0x0F,0x10
```

VS Code 里也有现成任务：**「联调: 串口协议监视」** / **「联调: 串口监视(只看 0x0F/0x10)」**。
它会逐帧打印类型名、payload、CRC 校验结果和统计（CRC 错误、丢弃字节），比裸看 HEX 快得多。

每次联调结束，在 `DEVLOG.md` 追加一条：

```
## 联调 YYYY-MM-DD
- 固件 commit: <short sha>   上位机 commit: <short sha>
- 用到的 TYPE: 0x0A / 0x0C / 0x0F ...
- 结果: 通过 / 部分通过（列出未通过项）
- 遗留: 谁欠谁什么
```
