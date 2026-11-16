# natsiie_gcs 交接 README

> 面向下一批接手的同学：先读这一份，再按需深入 `PROTOCOL.md`、`DEVLOG.md`、`PLACEMENT.md` 和 `stm32_firmware/交接文档.md`。
> 目标不是“看懂所有代码”，而是知道**系统怎么跑、参数在哪、坏了先查什么、改东西从哪里下手**。

---

## 0. 一句话说明

一台麦轮 + 塔吊 + 转盘的比赛小车：

- **STM32F4 下位机**负责底盘、塔吊、舵机、夹爪和陀螺仪；
- **树莓派 5（ROS 2 Jazzy）**负责相机、颜色/二维码/圆环视觉、坐标投影和流程状态机；
- 两者通过 **USART1 115200 的自定义变长帧协议**通信（见 `PROTOCOL.md`）；
- 比赛流程：**扫码 → 转盘对位 → 颜色识别抓取 → 粗加工区三圆环放置 → 暂存区叠料**。

主机（开发机）一般只用来改代码、跑单测、标定；**实际运行在树莓派上**。

---

## 1. 系统组成与硬件

### 1.1 硬件

| 部件 | 说明 |
|---|---|
| 下位机 | STM32F407 @168MHz，FreeRTOS，Keil MDK（armcc） |
| 底盘 | 4 个 ZDT X_V2 闭环步进（CAN 地址 1~4）+ 麦轮 |
| 塔吊 | 步进 5 = 升降，步进 6 = 伸出；3 个舵机 = 夹爪/物料平台/云台 |
| 陀螺仪 | 维特 JY901，USART2 115200，提供偏航角 |
| 遥控 | ELRS/CRSF，UART3/UART5 420000 |
| 上位机 | Raspberry Pi 5，Ubuntu 24.04，ROS 2 Jazzy |
| 主相机（扫码） | UNIQUESKY_CAR_CAMERA `1bcf:0b09` → `/dev/camera_front` |
| 上相机（视觉） | HJ USB 2.0 Camera `0c45:636b` SN0001 → `/dev/camera_upper` |
| 串口 | CH340 `1a86:7523` → `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`（同 `/dev/ttyUSB0`） |

> 两台相机都默认 **640×480 MJPG**。上相机发布 25 fps（采集端可跑满 30 fps），主相机读到二维码后停止采集。

### 1.2 固定设备名（udev）//连接相机后，要做的第一件事，要确保的是每次上位机重新上电或者相机端口重新插拔的时候，能直接映射到对应正确的绑定的相机号，避免重启后硬编码的相机号不一样导致相机打不开。

仓库里 [udev/99-camera-symlinks.rules](udev/99-camera-symlinks.rules) 按 **序列号/VID:PID** 绑定：

```bash
sudo install -m 0644 udev/99-camera-symlinks.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=video4linux
sudo udevadm trigger --action=add --subsystem-match=tty
```

不要用 `/dev/video0/2` 直接写死——USB 重新枚举后编号会变。

---

## 2. 目录结构

```
natsiie_gcs/
├── src/identify_pack/              # ROS 2 视觉包（Python）
│   ├── config/                     # 节点 YAML（部分机器专属）
│   ├── launch/                     # launch 文件（主入口 race_start.launch.py）
│   ├── identify_pack/
│   │   ├── vision_identify/        # 相机、颜色、转盘、圆环、叠料、QR、place_task
│   │   ├── tf_transform/           # TF 与像素→基座投影
│   │   ├── serial/                 # 串口协议与统一串口桥
│   │   └── scripts/                # 检测算法、标定工具、supervisor、工具脚本
│   ├── test/                       # pytest 单测 + 固件桩
│   └── setup.py                    # console_scripts / data_files
├── scripts/
│   ├── start_race.sh               # 开机自启动入口（SHOW/延时/DDS 环境）
│   ├── install_service.sh          # 生成并安装 systemd 服务
│   └── sync.sh                     # git 同步状态仪表盘
├── stm32_firmware/                 # STM32F4 固件（Keil 工程）
├── udev/                           # 相机/串口固定名规则
├── PROTOCOL.md                     # ⚠ 串口协议唯一事实来源
├── DEVLOG.md                       # 开发日志（部分内容较旧，以代码为准）
├── PLACEMENT.md                    # place_task 独立放置路径说明（当前比赛主流程未启用）
├── REMOTE_ACCESS.md                # 树莓派 SSH/RDP 调试说明（含弱密码，勿公开）
└── README.md                       # 本文件
```

---

## 3. 环境与构建

### 3.1 依赖

- ROS 2：树莓派 **Jazzy**，开发机可能是 Humble（**以目标机为准**）。
- Python：`python3-opencv`、`cv_bridge`、`pyzbar` + `libzbar0`、`pyserial`、`numpy`、`scipy`、`pytest`。
- 树莓派用户需要加入 `video`、`render` 组；串口设备用 udev 的 `MODE=0666` 免 sudo。

一键安装依赖（推荐）：

```bash
./scripts/install_deps.sh              # 安装 apt + ROS 依赖（缺失的纯 Python 包用 pip 兜底）
./scripts/install_deps.sh --check      # 只检查，不安装
./scripts/install_deps.sh --with-udev  # 顺带安装相机/串口固定名 udev 规则
```

脚本说明：

- 需要系统里**已经装好 ROS 2**（脚本不安装 ROS 本身）；
- OpenCV/numpy/scipy 优先走 apt，避免 pip 版 OpenCV 与 `cv_bridge` 冲突；
- 只有 apt 缺失的 `pyzbar`/`pyserial`/`pytest` 等纯 Python 包才用 pip 兜底（见仓库根目录 `requirements.txt`）；
- 结束时打印构建、设备检查和用户组配置的下一步命令。

### 3.2 构建

```bash
cd ~/natsiie_gcs
colcon build --symlink-install --packages-select identify_pack
source install/setup.bash
```

关键点：

- Python 包是 **egg-link**，改 `.py` 后通常立即生效；
- **YAML/JSON 是 build 时拷贝到 `install/share/` 的**，改 `src/.../config/*.yaml`、`*.json`、`*.txt` 后必须重新 `colcon build`（或手动拷到 `install/share/identify_pack`），否则节点加载的还是旧配置；
- 换机器后先确认 `source /opt/ros/<distro>/setup.bash` 的发行版与目标机一致。

### 3.3 测试

```bash
python3 -m pytest src/identify_pack/test -q
```

没有硬件也能跑大部分单元测试；`test/firmware_stubs` 是给固件相关测试用的桩。

---

## 4. 启动方式

### 4.1 手动启动（调试）

```bash
source /opt/ros/jazzy/setup.bash
source ~/natsiie_gcs/install/setup.bash

# 比赛主流程（无窗口）
ros2 launch identify_pack race_start.launch.py show:=false turntable_show:=false pixel_show:=false

# 想看调试窗口（必须有 DISPLAY）
ros2 launch identify_pack race_start.launch.py show:=true turntable_show:=true
```

> ⚠ `show` 只覆盖相机/颜色/圆环/叠料；转盘节点用的是**独立参数 `turntable_show`**，投影窗口是 `pixel_show`。无图形环境（SSH/系统服务）下任何一个为 true，节点会因 `cv2.imshow` 连不上 Qt display 直接 SIGABRT。

### 4.2 开机自启动（树莓派）

```bash
# 一次性安装/更新 systemd 服务（需要 sudo）
~/natsiie_gcs/scripts/install_service.sh

sudo systemctl enable --now race_start.service     # 开机自启 + 立即启动
sudo systemctl stop race_start.service             # 临时停止
sudo systemctl disable --now race_start.service    # 关闭自启并停止
journalctl -u race_start.service -f                # 看日志
```

服务调用 [scripts/start_race.sh](scripts/start_race.sh)，它做这几件事：

- 等串口出现（最多 30s）；
- 默认 `SHOW=false`，**三个窗口参数全部跟随 SHOW**；
- 默认 `START_DELAY_SEC=8`，串口就绪后再延时 8s，避开开机网络/udev 收尾；
- 默认 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，**DDS 只走 loopback**，不依赖 Wi-Fi 地址。

需要主机远程看话题时临时覆盖：

```bash
ROS_DISCOVERY_RANGE=SUBNET ~/natsiie_gcs/scripts/start_race.sh
```

### 4.3 数据目录

| 变量 | 默认 | 内容 |
|---|---|---|
| `NATSIIE_DATA_DIR` | `~/gcs` | `camera_extrinsics*.json`、`ground_homography.json` 等标定文件 |
| `NATSIIE_STATE_DIR` | `~/.local/state/natsiie_gcs` | place_task 等运行状态/检查点 |

系统服务里这两个变量已按当前用户家目录设置。**标定文件属于机器专属数据，换机器/换相机安装必须重新标定。**

---

## 5. 节点与话题

`race_start.launch.py` 默认拉起：

| 节点 | 作用 |
|---|---|
| `camera_main` | 主相机扫码 → `/QR_Decode`；读到码后停采集 |
| `camera_secondary` | 上相机 → `/camera/secondary/image_raw` |
| `color_detect_node` | 颜色块检测 + 停稳判定 → `/color/centers_pixel`、`/arm/arrived` |
| `turntable_detect_node` | 转盘外圆/圆心对齐 → `/Turntable`、`/TurntableAlign`（0x03 / 0x0E） |
| `ring_station_detect_node` | 三圆环工位 → `/ring_station`（0x04）、`/ring_station/centers_pixel` |
| `stacked_ring_detect_node` | 叠料区（中环 + 上下边线）→ `/stacked_ring`、`/stacked_ring/centers_pixel` |
| `tf_transform` | 云台角度 + 相机外参 → 动态 TF |
| `pixel_to_base` | 像素 → `gimbal_base` 毫米；颜色目标与 0x0C 三环圆心 |
| `serial_bridge_node` | 统一串口桥：协议帧收发、任务码、反馈话题 |
| `race_supervisor_node` | 比赛流程状态机 + 检测节点使能 |

### 关键话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/QR_Decode` | String | 任务码；**已改 TRANSIENT_LOCAL**，相机首发后 0.6s 内会重复发布 |
| `/camera/main/image_raw` | Image | 主相机原图 |
| `/camera/secondary/image_raw` | Image | 上相机原图（BEST_EFFORT） |
| `/color/centers_pixel` | PointCloud | 色块像素中心 + `motion_ready` |
| `/color/targets_base` | PointCloud | 颜色目标基座坐标（米） |
| `/ring_station/centers_pixel` | PointCloud | 三环像素中心 |
| `/ring_station/targets_base` | PointCloud | 三环基座坐标（0x0C） |
| `/stacked_ring/centers_pixel` | PointCloud | 叠料区像素中心 |
| `/Turntable` / `/TurntableAlign` | String | 转盘外圆 / 对齐纠偏 |
| `/ring_station` / `/stacked_ring` | String | 工位偏差 `err_x,err_y,ang,corr_*,within` |
| `/arm/arrived` | Int32 | 色块停稳，携带颜色号 |
| `/arm/align_done` | String | 下位机 0x11 矫正完成 JSON |
| `/arm/region_ready` | String | 下位机 0x12 到点+云台完成 |
| `/arm/pick_done` | UInt32 | 0x0F 抓取完成序号 |
| `/arm/next_batch` | UInt32 | 0x10 第一批完成 |
| `/arm/place_result` | String | 0x0B 放置结果 |
| `/arm/place_control` | String | 0x0D 放置阶段控制 |
| `/arm/task_status` | String | 串口桥内部任务状态（latched） |
| `/gimbal/joint_states` | JointState | 0x07 云台角度（弧度） |
| `/tf`、`/tf_static` | TF | 动态/静态变换 |
| `/color/enable` 等 | Bool | supervisor 使能各检测节点（一次性） |

> `/color/enable`、`/ring_station/enable`、`/turntable/enable`、`/stacked_ring/enable` 目前仍是**普通一次性 Bool**；如果发现节点没被使能，先查这几条是否丢失。

---

## 6. 比赛流程 / 状态机

`race_supervisor_node` 的步骤：

```
WAIT_QR → TURNTABLE → COLOR → ALIGN → MEASURE → PLACE → ... → DONE
```

| 步骤 | 等待什么 | 来源 |
|---|---|---|
| `WAIT_QR` | `/QR_Decode` | 主相机 |
| `TURNTABLE` | `/arm/align_done` stage=1 | 下位机 0x11 |
| `COLOR` | 3 次 `/arm/pick_done` | 下位机 0x0F |
| `ALIGN` | `/arm/align_done` stage=2/3 | 下位机 0x11 |
| `MEASURE/PLACE` | `/arm/region_ready` + AlignDone；`pixel_to_base` 发一次 0x0C；`/arm/place_result` | 下位机 0x12/0x0B |

阶段顺序（`PHASES`）：

```python
('ring_station', 1),
('ring_station', 1),
('ring_station', 2),
('stacked_ring', 2),
```

- 前三次是粗加工区平地放置，最后一次是暂存区叠料；
- 第二次 `TURNTABLE` 发生在 `phase_index` 到 2 时；
- **重要**：supervisor 收到 `AlignDone` 就切下一步，并不校验“这次是否真的重新检测过”。如果转盘节点刚使能、下位机因底盘仍在容差内立刻回一条 stage=1，看起来就像“直接跳到了颜色识别”。排查时看 `检测节点: turntable` 与 `转盘矫正完成` 两行的时间差。

`pixel_to_base` 在收到 `RegionReady` 后观察 `ring_measure_window_sec`（默认 1.0s），剔除离群点后**只发一次 0x0C**；固件侧也是“只等这一帧”，丢帧本站就没有落点。

---

## 7. 串口协议要点

**`PROTOCOL.md` 是唯一事实来源。** 帧格式：

```
0xFF | TYPE | LEN | PAYLOAD(UTF-8) | CRC8 | 0xFE
CRC8 = TYPE ^ LEN ^ payload（逐字节异或）
```

常用类型（完整表见 `PROTOCOL.md`）：

| TYPE | 名称 | 方向 | 要点 |
|---|---|---|---|
| 0x03 | Turntable | 上→下 | 转盘圆心/半径/稳定 |
| 0x04 | ring_station | 上→下 | 三环偏差 + 底盘纠偏 |
| 0x05 | stacked_ring | 上→下 | 叠料位偏差 |
| 0x06 | QR_Decode | 上→下 | 任务码，串口桥收到后每秒重发、持续 30s |
| 0x07 | GimbalYaw | 下→上 | ASCII 角度，必须持续上报 |
| 0x08 | ColorTarget | 上→下 | `color_id,x_mm,y_mm,z_mm`（gimbal_base） |
| 0x09 | GrabCmd | 上→下 | `1` = 授权下降抓取 |
| 0x0B | PlaceResult | 下→上 | `task_id,command_id,success` |
| 0x0C | RingTarget | 上→下 | 三组 `x,y`，三环圆心（坐标系声明待统一，见 §12） |
| 0x0D | PlaceControl | 下→上 | `start,stage,batch` 或 `retry,command_id` |
| 0x0E | TurntableAlign | 上→下 | `corr_x,corr_y,within` |
| 0x0F | PickDone | 下→上 | 序号 1~6 |
| 0x10 | NextBatch | 下→上 | `1` |
| 0x11 | AlignDone | 下→上 | `stage,success` |
| 0x12 | RegionReady | 下→上 | `stage` |
| 0x13 | PickColor | 下→上 | 本件颜色（视觉日志用） |

改协议的顺序：**先改 `PROTOCOL.md` → 再改 `serial/protocol.py` 和固件 `Devices/Serial.h/.c` → 开 PR 双方确认**。

---

## 8. 标定与配置

### 8.1 哪些是机器专属（不要跨机覆盖）

| 文件 | 内容 |
|---|---|
| `src/identify_pack/config/pixel_to_base.yaml` | 圆环测量窗口、yaw 偏置、径向/单环补偿 |
| `src/identify_pack/identify_pack/scripts/color_config.json` | 七色 HSV 阈值（跟光照强相关） |
| `.../scripts/ring_station_detector.json` | 参考圆心、仿射矩阵、`chassis_image_to_motion`、检测阈值 |
| `.../scripts/stacked_ring_detector.json` / `turntable_*.json` | 搜索范围/参考值 |
| `.../scripts/ROI_Range.txt` / `Arrival_ROI.txt` | 相机安装位置相关 ROI |
| `.../serial/serial_config.json` | 串口设备/波特率 |
| `~/gcs/camera_extrinsics*.json`、`ground_homography.json` | 相机外参/地面单应 |
| `/etc/systemd/system/race_start.service` | 机器路径/用户 |
| `/etc/udev/rules.d/99-camera-symlinks.rules` | 相机/串口绑定 |

主机和树莓派这两套配置**本来就不一样**；同步代码时不要用主机配置覆盖 Pi 的标定值。

### 8.2 标定工具（按依赖顺序）

```bash
# 1) 相机内参（棋盘格）
ros2 run identify_pack camera_diagnostics          # 先确认相机/曝光/画面
python3 -m identify_pack.scripts.calibrate_camera

# 2) 相机外参 / 地面单应（未注册成 ros2 可执行，用 python3 -m）
python3 -m identify_pack.scripts.calibrate_camera_extrinsics_auto   # 3x3 圆点板
python3 -m identify_pack.scripts.calibrate_ground_homography        # 地面标记 + gimbal_base 坐标

# 3) 转盘
ros2 run identify_pack turntable_calibrator        # 外圆参数
ros2 run identify_pack turntable_align_calibrator  # 对齐参考圆心

# 4) 三圆环工位
ros2 run identify_pack ring_station_calibrator     # 参考圆心 + px/mm
python3 -m identify_pack.scripts.chassis_calibrator # 底盘纠偏 3x3 矩阵

# 5) 叠料区
python3 -m identify_pack.scripts.stacked_ring_calibrator

# 6) 颜色（编号：1红 2黄 3蓝 4绿 5黑 6浅蓝 7粉标记）
ros2 run identify_pack color_calibrator            # 数字键选色，拖框采样，S 保存
```

标定工具大多直接打开相机，**先停掉 race_start 服务**，避免设备被占用。

---

## 9. 调试方法

```bash
# 话题
ros2 topic list
ros2 topic hz /camera/secondary/image_raw
ros2 topic echo /arm/task_status --qos-durability transient_local
ros2 topic echo /rosout

# 节点图
ros2 node list
ros2 node info /race_supervisor_node
ros2 topic info /QR_Decode -v

# 服务日志（推荐）
journalctl -u race_start.service -f

# 无 ROS 的相机/检测诊断
python3 -m identify_pack.scripts.camera_diagnostics --device /dev/camera_upper
```

本仓库的调试经验：

- **先看 `journalctl` 里的 `[supervisor] step=...`**：卡住时新加的 30s 超时告警会打出来；
- 相机状态行会打印 `capture/publish` fps，低于预期先查 CPU 和 USB；
- `pixel_to_base` 的 `Waiting for image-time TF` 警告通常是 0x07 云台角度断流或时间戳超前；
- 主相机扫码时，画面里二维码至少要占 150~200px；太小/过曝会解不出来；
- 需要无显示环境调试时，别用 `show:=true`。

---

## 10. 已知坑与经验（重点，都是真金白银踩出来的）

1. **改了配置没重编**：YAML/JSON 是拷贝到 `install/share` 的，节点加载的是那份。症状是参数不生效。改完 `colcon build --packages-select identify_pack`。
2. **V4L2 单缓冲把帧率砍半**：`CAP_PROP_BUFFERSIZE=1` 时相机要等应用读完才能填下一帧，640×480 MJPG 实测只有 ~16 fps；改成 **2** 后稳定 30 fps（`camera_capture.py`）。
3. **二维码是一次性 VOLATILE 消息**：相机发完立刻停采集，DDS 发现稍抖就永久丢失。现已改为 **TRANSIENT_LOCAL + 首发后 0.6s 重复发布窗口**；但 `/arm/*` 的很多状态消息仍是一次性的。
4. **DDS 发现会被 Wi-Fi 地址变化搞坏**：开机时 Wi-Fi/IPv6 还在变，participant 绑到过渡期地址，之后新节点发现不了老节点。系统服务现默认 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` + 启动延时 8s。单机跑比赛建议保持 LOCALHOST；不要依赖现场网络。
5. **无图形环境开调试窗口会崩**：`turntable_show` 是独立参数，默认 true；systemd 下会 `qt.qpa.xcb: could not connect to display` → SIGABRT。`start_race.sh` 已把 `show/turntable_show/pixel_show` 全部跟随 `SHOW`。
6. **二维码全角加号**：任务码里用了全角 `＋`（U+FF0B）会被 `parse_task` 拒绝（只认半角 `+`）。生成二维码时用半角；后续可考虑 NFKC 归一化。
7. **三圆环置信度上限**：`minimum_confidence` 公式上限约 0.8，别调到 0.9 以上，否则永远进不了 stable；当前 Pi 用 0.5。`stable_frame_count` 也不能太小，否则运动拖影会被当成稳定。
8. **一次性状态消息**：0x0B/0x0F/0x11/0x12/0x10/0x0D 都是“发一次”，丢了流程就静默卡住。supervisor 已加 30s 超时告警；排查时优先看上下位机两侧日志的时间对齐。
9. **USB 供电**：总线供电的扩展坞带 500mA 相机会枚举失败（`Cannot enable`），相机请直插 Pi 或使用带外接电源的坞。
10. **主/副相机不要写死 `/dev/videoN`**：用 udev 的 `/dev/camera_front`、`/dev/camera_upper`。
11. **`install/share` 与 `src` 可能漂移**：接手后先确认两者配置一致；换机器时把标定文件一起带走。
12. **Pi 的 RTC/NTP 起步会跳时间**：早期日志时间戳看起来错位，属正常现象，不影响功能。

---

## 11. 二次开发指南

### 11.1 新增一个检测节点

1. 在 `identify_pack/vision_identify/` 或 `scripts/` 写节点，沿用现有模式：
   - `enabled` + `enable_topic` 常驻休眠；
   - 图像订阅用 `qos_profile_sensor_data`；
   - 关键状态消息优先考虑 `TRANSIENT_LOCAL` 或重复发布；
   - `show` 参数默认 False，且用 try/except 包住 `cv2.imshow`。
2. 在 `setup.py` 的 `console_scripts` 注册；
3. 在 `race_start.launch.py` 加 Node，并决定由谁使能；
4. 给纯逻辑部分写 `test/` 单测（无 ROS 也能跑）。

### 11.2 新增/修改串口类型

按 `PROTOCOL.md` 的流程：先文档，再 `serial/protocol.py` + 固件 `Devices/Serial.h/.c`，最后 PR。**不要复用已退役的类型号。**

### 11.3 改比赛阶段

`race_supervisor_node.py` 的 `PHASES`、`STAGE_NAME` 与 `serial/protocol.py` 的 stage 映射（1/2/3）必须一起改；`pixel_to_base` 的 `ring_centers_topic_after`、`serial_bridge` 的 stage 判断也要对齐。

### 11.4 改配置

优先改 `src`，然后 `colcon build`。机器专属配置（见 §8.1）只改本机，不要提交/覆盖对方的标定值。

---

## 12. 当前未完成 / 风险点

- **0x0C 坐标系语义未统一**：上位机注释是 `gimbal_base` 毫米，固件注释是塔吊系绝对 mm（见 `PROTOCOL.md` §3.3）。现场偏移补偿可能掩盖了这个差异，接手后务必拍板。
- **0x0E 文档状态**：`PROTOCOL.md` 写“固件未实现”，但固件 `app_chassis.c` 已有 trim/AlignDone 逻辑，需以实际烧录版本为准核对。
- **第二次 TURNTABLE 可能“假完成”**：supervisor 只认 `AlignDone`，不校验是否发生了新的转盘检测。建议加一条 guard：进入 TURNTABLE 后记录时间，`AlignDone` 到达时打印 elapsed；若过短或期间没有新的 `/TurntableAlign`，继续等待。
- **一次性 `/arm/*` 消息**：目前只有 `/QR_Decode` 做了 latched；其余关键反馈仍可能丢失。可考虑统一 TRANSIENT_LOCAL 或加确认/重发。
- **二维码全角字符**：`parse_task` 尚未做 NFKC 归一化。
- **满栈帧率**：全栈运行时上相机约 10~15 fps（CPU/带宽限制），停稳判定按秒计算不受影响，但首帧延迟和漏检概率会变差。
- **place_task / PLACEMENT.md 路径**：当前比赛主流程用 0x0C 批量模型，`place_task` 节点默认未启动；它是另一条独立放置路径，不要和主流程混用。

---

## 13. 交接检查清单

- [ ] 树莓派能 SSH；`udev` 规则已安装，`/dev/camera_front`、`/dev/camera_upper`、串口 by-id 都存在。
- [ ] `colcon build --symlink-install` 通过，`source install/setup.bash` 后 `ros2 pkg list | grep identify_pack` 正常。
- [ ] `python3 -m pytest src/identify_pack/test -q` 通过（无硬件的用例）。
- [ ] `race_start.launch.py` 手动能起，10 个节点无 `process has died`。
- [ ] 主相机能读码、supervisor 能收到 `二维码任务码`；串口桥能 `TX QR_Decode`。
- [ ] 0x07 云台角度持续刷新，`/tf` 正常，`pixel_to_base` 不再频繁报 TF 超时。
- [ ] 标定文件（`~/gcs/*.json`、各 `*.yaml/*.json`）已备份，并记录当前相机/云台安装状态。
- [ ] `scripts/install_service.sh` 生成的服务单元路径/用户正确；`systemctl enable/disable` 行为符合预期。
- [ ] 与下位机确认烧录的固件版本与 `PROTOCOL.md` 一致（尤其 0x0C/0x0E/0x11/0x12）。

---

## 14. 相关文档

| 文档 | 内容 |
|---|---|
| `PROTOCOL.md` | 串口协议唯一事实来源 |
| `DEVLOG.md` | 开发日志（注意末尾时间较旧，以代码为准） |
| `PLACEMENT.md` | place_task 独立放置路径 |
| `stm32_firmware/交接文档.md` | 固件结构、硬件映射、电机/舵机踩坑 |
| `REMOTE_ACCESS.md` | 树莓派 SSH/RDP 调试（含弱密码，勿公开/勿提交） |
| `src/identify_pack/identify_pack/scripts/CAMERA_TO_GIMBAL.md` | 相机→云台坐标链说明 |
| `src/identify_pack/identify_pack/scripts/RING_STATION_DIGIT_TEST.md` | 圆环数字识别测试 |

---

> 最后更新：2026-11-16
> 维护建议：每次大改后同步更新本文件的“已知坑”“风险点”和“交接检查清单”；协议改动必须先改 `PROTOCOL.md`。
