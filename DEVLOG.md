# natsiie_gcs 开发日志

> 最后更新：2026-10-09
> 目的：每天开工前先读一遍，快速接上上下文，别把之前调好的东西忘了。

---

## 0. 一句话现状（先看这个）

正在做**「转盘物料抓取」的视觉 + 串口链路**。ROS 侧已经基本打通：
**相机识别物块 → 投影成基座毫米坐标 → 串口发 0x08 目标 → 夹爪到位（粉色舵机外壳进 ROI 连续 5s）→ 串口发 0x09 授权下降抓取。**

**当前卡点**：下位机 STM32 的云台角度反馈（0x07）**只发一帧就停了**，导致 TF 超时。你重写的 STM32 固件（Serial.c/MyTasks.c）**还没用 Keil 编译烧录**。烧录后确认 0x07 持续上报即可继续。

---

## 1. 系统架构

两个子项目：
- `src/identify_pack/` — ROS2 视觉包（Python）
- `stm32_firmware/` — STM32F4 下位机（Keil + FreeRTOS）

**ROS 节点**（`race_start.launch.py` 一次拉起）：

| 节点 | 作用 |
|---|---|
| `camera_main` | 主相机 `/dev/camera_front`，解码二维码任务码 → `/QR_Decode` |
| `camera_secondary` | 副相机 `/dev/camera_upper`，供颜色/到位识别 |
| `color_detect_node` | 识别 1~6 号物块色块 → `/color/centers_pixel` |
| `gripper_arrival` | 识别粉色(7)在 ROI 内连续满阈值 → `/arm/arrived` |
| `tf_transform` | 云台角度 → 发布 `gimbal_base→gimbal_link` 动态 TF |
| `pixel_to_base` | 像素 → 基座毫米坐标 → `/color/targets_base` |
| `serial_bridge_node` | 统一串口桥：发 0x08/0x09，收 0x07 |

**数据流**：

```
主相机读二维码 → 任务顺序(6个pick)
副相机 → color_detect(1~6色块) → pixel_to_base(毫米) → serial_bridge → 0x08 目标
副相机 → gripper_arrival(粉色5s) → /arm/arrived → serial_bridge → 0x09 授权下降
下位机 → 0x07 云台角度 → serial_bridge → /gimbal/joint_states → tf_transform → TF
```

---

## 2. 颜色识别（七色，含粉色标记）

**颜色编号**：`1红 2黄 3蓝 4绿 5黑 6浅蓝 7粉(标记色)`

**为什么加粉色**：抓取位姿下物块本体识别不到（相机随云台转，物块出画或被挡）。改识别**夹爪上的粉色舵机外壳**，用它判断夹爪是否到位。

**关键文件** `src/identify_pack/identify_pack/scripts/color_config.py`：
- 常量 `COLOR_IDS=(1..7)`、`PICKUP_COLOR_IDS=(1..6)`、`MARKER_COLOR_IDS=(7,)`
- 粉色已标定（`color_config.json`）：`H160~172, S9~46, V205~253`

**隔离设计（重要）**：粉色是标记、不是抓取目标，全程与抓取色隔离：

| | 抓取色 1~6 | 标记色 7 粉 |
|---|---|---|
| color_detect_node | ✅ 识别输出 | ❌ 过滤掉 |
| pixel_to_base | ✅ 投影 | ❌ 跳过 |
| serial_bridge | ✅ 发目标 | ❌ 排除 |
| gripper_arrival | ❌ 不看 | ✅ 专测 |

---

## 3. 夹爪到位检测

- **纯逻辑** `scripts/arrival.py`：`GripperArrivalDetector`，粉色**连续可见**满 `min_confirm_sec` 秒判定到位；中途消失即清零重计；到位后锁存，`reset()` 复用。
- **节点** `vision_identify/gripper_arrival_node.py`：订阅副相机，在 `arrival_roi_file`（**独立于抓取 ROI**，`Arrival_ROI.txt`）内找粉色，满阈值发 `std_msgs/Bool(true)` 到 `/arm/arrived`（transient-local 锁存）。

---

## 4. 串口协议

帧格式：`FF | TYPE | LEN | PAYLOAD(UTF-8) | CRC8 | FE`，CRC = `TYPE ^ LEN ^ PAYLOAD` 逐字节异或。

类型号（两边 `protocol.py` 与 `Serial.h` 一致）：

| TYPE | 方向 | 含义 |
|---|---|---|
| 0x06 | ROS→STM | QR 任务码字符串 |
| 0x07 | STM→ROS | 云台角度（ASCII 度） |
| 0x08 | ROS→STM | 目标坐标 `color_id,x_mm,y_mm,z_mm` |
| 0x09 | ROS→STM | 夹爪到位授权，payload `"1"` |

**0x09 帧示例**：`ff 09 01 31 39 fe`（6 字节）。

**下位机侧**（你重写的 `stm32_firmware/Devices/Serial.c`）：
- RX 六态状态机 `HEAD→TYPE→LEN→PAYLOAD→CRC→TAIL`，CRC 校验。
- `Serial_ProcessFrame()` 按 type 分发：0x08 → `kin_target_x/y`（只存坐标不动作）、0x09 → `grab_cmd=1`、0x06 → 串口屏。
- `MyTasks.c` 里 10ms 循环消费 `Serial_RxFrameReady`。
- `app_kin.c` 的 `Kin_Update` 状态机：`kin_cmd=1` 触发 GotoXY → 到位点上方 → `WAIT_GRAB` 等 `grab_cmd` → 下降。

---

## 5. 像素 → 基座投影

`pixel_to_base`：像素质心 + 相机内参 + TF → 反投影到「物块顶面」平面（`object_plane_z_mm = -12.5mm`）→ `gimbal_base` 系毫米坐标。

- `x,y` 是投影算出来的；`z=-12.5` 是**固定平面高度**（标定值，不是实测）。
- **最新 TF 兜底**：图像时刻 TF 不可用（云台角度反馈慢/断）且最新 TF 不晚于图像时刻时，退回最新 TF 投影。

---

## 6. 开发历程（时间线）

1. **颜色识别迁移**：HSV 阈值从硬编码改为 `color_config.json` 配置驱动。
2. **加粉色标记色**（第 7 号）：COLOR_NAMES/PRESET_RANGES/JSON 加 pink；下游三处 `(1,2,3,4,5,6)` 硬编码改成 `PICKUP_COLOR_IDS`；标定工具支持按 `7` 选色。
3. **删除坏副本**：`vision_identify/color_calibrator.py` 是坏副本（import 路径错、无入口），已删。
4. **隔离粉色**：`color_detect_node` 生成掩膜后过滤掉粉色，只处理 1~6，避免粉色进 ROI 被主色锁定/单色块逻辑顶掉物块。
5. **夹爪到位检测**：新增 `arrival.py` + `gripper_arrival_node.py`，粉色连续 5s → `/arm/arrived`。
6. **串口授权**：`protocol.py` 加 0x09 GrabCmd；`serial_bridge` 订阅 `/arm/arrived` 发 0x09。
7. **launch 参数化**：`motion_detect`、`task_mode`、`start_gripper_arrival`、`arrived_topic`、`arrive_confirm_sec` 都提成 launch 参数。
8. **下位机 RX 重写**（你做的）：变长解析 + 0x06/0x08/0x09 分发 + grab_cmd 接通（**待 Keil 编译烧录**）。
9. **最新 TF 兜底**：解决云台角度反馈慢导致的投影超时。
10. **到位检测独立 ROI**：`gripper_arrival` 改用 `Arrival_ROI.txt`，与抓取 `ROI_Range.txt` 分开。
11. **副相机 MJPG 卡死修复**：`fourcc` 默认改回空（YUYV）。
12. **camera_node 加看门狗**：采集独立线程 + 最新帧队列 + 读帧超时自动重连。

---

## 7. 调试踩坑记录（重要）

1. **改了源码必须 `colcon build` + `source install/setup.bash`**，否则跑的是 install 里的旧版。症状：报错信息还是旧文案（如「6 种颜色」），或新参数不生效（如 `motion_detect:=false` 没反应）。
2. **`Motion=ACQUIRING` 卡住识别**：运动检测要求「先观察到转动、再观察到停稳」才 READY。转盘静止时永远不 READY，颜色识别被挡。调试设 `motion_detect:=false` 跳过。
3. **TF 超时**：`pixel_to_base` 严格按图像时刻查 TF，云台角度反馈（0x07）慢/断 → 动态 TF 过期 → 查不到 → `颜色目标过期或 TF 等待超时，跳过`。已加最新 TF 兜底，但根因是下位机 0x07 要持续发。
4. **下位机 RX 帧格式**（已修）：原来固定 8 字节，和 ROS 变长帧不匹配；重写后变长解析，注意用 Keil 烧录才生效。
5. **副相机 MJPG 卡死**（已修）：`fourcc='MJPG'` 时部分 USB 相机跑十几秒后 `read()` 阻塞卡死（日志里没有「读取帧失败」就是证据——read 是卡住、不是返回 False）。已改默认 `fourcc=''`（YUYV），并给 `camera_node` 加采集线程 + 最新帧队列 + 看门狗自动重连。定位方法：单跑 `color_detect_debug.py`（不设 MJPG）不卡，就说明是 MJPG 的锅。

---

## 8. 待办 / 下一步

- [ ] **烧录 STM32 固件**（Keil 编译 + 烧录），确认 `RX GimbalYaw` 持续刷屏（当前只发一帧就停）。
- [ ] 通信链路验证：`task_mode:=false` 连续发 0x08，看 STM32 侧 `kin_target_x/y` 被持续刷新。
- [ ] **0x08 只存坐标不触发运动**：收到 0x08 后需手动 `kin_cmd=1`（Keil Watch）或加自动触发。
- [ ] **`grab_cmd` 无超时**：`app_kin.c` 的 `WAIT_GRAB` 无限等 0x09，丢帧会卡死，建议加超时回安全位。
- [ ] 标定参数 `kin_*` 还是占位值，现场标定后填入。

---

## 9. 常用命令速查

```bash
# 编译 ROS 包
cd ~/gcs/natsiie_gcs && colcon build --packages-select identify_pack && source install/setup.bash

# 正式跑（默认任务模式）
ros2 launch identify_pack race_start.launch.py show:=true

# 调试识别（跳过运动门控）
ros2 launch identify_pack race_start.launch.py show:=true motion_detect:=false

# 调试通信（连续发坐标，验证链路）
ros2 launch identify_pack race_start.launch.py show:=true motion_detect:=false task_mode:=false

# 看话题
ros2 topic echo /arm/arrived          # 到位信号
ros2 topic echo /color/targets_base   # 基座毫米坐标
ros2 topic echo /color/centers_pixel  # 像素坐标

# 手动触发串口授权（不经过识别，测串口桥）
ros2 topic pub /arm/arrived std_msgs/msg/Bool "{data: true}" --once

# 独立调试脚本（全画面看颜色识别 + 点击采样 HSV）
cd ~/gcs/natsiie_gcs/src/identify_pack/identify_pack/scripts
python3 color_detect_debug.py --device /dev/camera_upper
python3 color_calibrator.py --device /dev/camera_upper --config color_config.json
```

---

## 10. 关键文件索引

**ROS 侧**（`src/identify_pack/`）：
- `identify_pack/scripts/color_config.py` — 颜色定义唯一源头（七色、PICKUP/MARKER 常量、HSV 工具）
- `identify_pack/scripts/color_config.json` — 已标定的七色 HSV 阈值
- `identify_pack/scripts/arrival.py` — 到位检测纯逻辑状态机
- `identify_pack/scripts/pick_task.py` — 任务码解析 + 抓取顺序状态机
- `identify_pack/vision_identify/color_detect_node.py` — 抓取识别节点（1~6）
- `identify_pack/vision_identify/gripper_arrival_node.py` — 到位检测节点（粉色 5s）
- `identify_pack/tf_transform/pixel_to_base_node.py` — 像素→毫米投影（含最新 TF 兜底）
- `identify_pack/serial/protocol.py` — 串口帧协议 + 类型号
- `identify_pack/serial/serial_bridge_node.py` — 统一串口桥
- `launch/race_start.launch.py` — 总启动入口

**下位机**（`stm32_firmware/`）：
- `Devices/Serial.c` / `Serial.h` — 串口收发（变长帧 + 按 type 分发）
- `FreeRTOS/MyTasks.c` — 任务调度（10ms 循环里消费串口、发云台角度）
- `Apps/app_kin.c` / `app_kin.h` — 塔吊正逆解 + 点到点运动状态机
- `Apps/app_tower.c` / `app_tower.h` — 塔吊一体化抓取流程（旧，现走 Kin 路径）

## 11. 放置进度（2026-10-09）

- 新增 `place_task` 独立放置状态机，复用二维码颜色与环号对应计划，按固定左/中/右 R1/R2/R3 选择目标。
- 两个圆环节点新增带时间戳的 `/ring_station/centers_pixel`、`/stacked_ring/centers_pixel`；只有新鲜、稳定、满足对位条件且有图像时刻 TF 的目标才能发出。
- 每个工位、每批的放置进度独立记录；叠料区第一批落地，第二批叠在第一批上。物料高度 60 mm，叠料地面相对 `gimbal_base` 的 Z 为 -72.5 mm。首层夹爪释放 Z 尚待测量，与地面 Z 分开配置。
- 仅匹配任务 UUID、指令编号的成功反馈推进进度；错误/超时保留目标，重复反馈不累加层数。进度原子保存，重启后不自动重发，阶段启动和重试都需要明确控制消息。
- 新增串口 `0x0A` 放置目标、`0x0B` 放置完成反馈，统一串口桥已接通。STM32 `Serial_TakePlaceCommand` / `Serial_CompletePlaceCommand` 提供目标读取、去重及完成回报接口，尚需接入实际放料与撤离动作；没有烧录或实机运动测试。
- 配置 `src/identify_pack/config/place_task.yaml` 默认禁用；入口 `ros2 launch identify_pack place_task.launch.py`。完整接口和启用要求见 `src/identify_pack/PLACEMENT.md`。
- 全套 72 个测试通过，ROS 包构建通过。安装后节点启动检查受沙箱 DDS 网络限制，提权验证未执行（自动审批服务模型配置返回 404）。

## 12. 上相机重复帧看门狗（2026-10-09）

- 原看门狗只检测无帧超时：即使上相机重复返回同一张图，话题有频率也不会重连。
- `race_start.launch.py` 对 `camera_secondary`（默认 `/dev/camera_upper`）启用 `secondary_frozen_frame_timeout_sec:=5.0`：连续 5 秒整帧像素完全相同触发重连。设为 `0` 可关闭；主相机仍只检查无帧超时。
- 使用整帧严格相等判断，单个像素变化也会清零计时。不能检测带微小变化的冻结画面，也不能判断下游显示窗口是否卡住；完全静态的合成图像可能误触发。
- 重连日志包含设备路径和话题；旧采集线程固定使用原 capture，重连后旧代帧丢弃，避免旧画面刷新看门狗。
- 修改后重新 `colcon build --packages-select identify_pack` 并 `source install/setup.bash`，停止旧启动进程后再运行。

## 13. 相机采集与重启阻塞修复（2026-10-09）

- 最近上相机日志只有打开成功，没有看门狗或重启记录；这不足以证明 USB 驱动故障。代码确认原看门狗与图像发布共用 ROS 执行线程，发布阻塞时看门狗无法执行，跨线程 `read/release` 也不能可靠恢复。
- 改为 `spawn` 独立采集进程，独占 OpenCV capture；共享内存只保存最新帧。独立监控线程检测无帧/重复帧并限时终止、重建采集进程，不在读帧线程外调用 `release()`。子进程退出时由操作系统关闭设备。
- 发布使用 BEST_EFFORT / KEEP_LAST(1)，图像订阅同步采用传感器 QoS。发布或 QR 回调超过 5 秒时退出节点，沿用 launch 的 respawn 恢复；QR 正常停止采集不会误重启。
- 每 5 秒日志打印采集 fps、发布 fps、帧龄、重复帧数；采集进程打印实际协商格式/分辨率/帧率。可用 `secondary_camera_fps`、`secondary_camera_fourcc` 调节上相机请求设置，默认保留 25/YUYV。
- 硬件设备/USB 内核日志检查受沙箱隔离，自动审批服务返回模型配置 404，实际设备检查未执行；模拟永久阻塞取帧、限时回收重启、旧帧隔离以及独立监控恢复已覆盖，仍需现场验证驱动与图像显示。
- 验证：86 个测试通过，`colcon build --packages-select identify_pack` 成功；未执行机器人启动或串口控制。

## 14. 圆环与实际位置数字独立测试（2026-10-10）

- 新增 `identify_pack/scripts/ring_station_digit_test.py`，复用 cyqm 已移植的轮廓提取和三环几何筛选，保留原始椭圆用于环内数字局部仿射矫正；新增 1/2/3 模板匹配。cyqm 原程序没有数字识别，不能把左右排序当作实际位置编号。
- 支持相机实时预览、图片离线测试、数字裁剪预览、鼠标选环采集现场模板以及原图/标注/JSON 快照。使用独立模板目录，不覆盖已有数字脚本、模板或圆环标定配置；不接入 ROS 控制流程。
- 只有三环几何通过且读到的数字为互不重复的 1/2/3 时才报告完整映射；空白、遮挡、低相似度和匹配歧义保留未知。
- 使用方法见 `src/identify_pack/identify_pack/scripts/RING_STATION_DIGIT_TEST.md`。合成图与离线入口已验证，未进行实机相机或比赛场地图像验证，现场字体建议重新采模板。
