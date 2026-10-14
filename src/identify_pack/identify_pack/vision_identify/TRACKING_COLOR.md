# 全图多物块颜色跟踪

新增 `tracking_color_node.py` 与原来的 `color_detect_node.py` 可同时运行。原 ROI 文件、颜色识别节点和 `race_start.launch.py` 不由此功能修改。新节点订阅同一相机图像，不自行打开设备，独立发布跟踪话题。它持续检测转动中的物块，不等待停稳，也不发送串口或机械臂指令。

## 启动

```bash
cd ~/gcs/natsiie_gcs
colcon build --packages-select identify_pack
source install/setup.bash

# 已有相机节点发布 /camera/secondary/image_raw 时
ros2 launch identify_pack tracking_color.launch.py show:=true
```

如运行 `race_start.launch.py`，在另一个终端启动上面跟踪 launch 即可；跟踪节点与 ROI 节点会同时收到相机图像。`race_start` 自身会运行原有任务和串口链路，如果只验证跟踪，使用下面的独立相机方式：

```bash
# 相机没有被其他程序占用时：只启动一台相机和跟踪节点
ros2 launch identify_pack tracking_color.launch.py start_camera:=true show:=true

# 或已有其他图像话题
ros2 launch identify_pack tracking_color.launch.py image_topic:=/camera/image_raw show:=true
```

默认 `start_camera:=false`；不要同时让两个相机节点打开 `/dev/camera_upper`。独立相机默认 640×480、25 fps、YUYV、`flip_code:=0`，可用 `device:=/dev/video0` 或 `flip_code:=-1` 覆盖。

窗口显示各物块的 ID、颜色、状态、像素速度和历史轨迹。默认 `show:=false`，适合通过 SSH 或服务运行；无图形桌面时可以查看 `/tracking_color/debug_image`。

```bash
ros2 topic echo /tracking_color/tracks
ros2 topic echo /tracking_color/centers_pixel
ros2 topic hz /tracking_color/debug_image
```

也可直接运行节点并传参数：

```bash
ros2 run identify_pack tracking_color_node --ros-args \
  -p show:=true -p min_blob_area:=500.0 -p max_match_distance_px:=60.0
```

## 检测与跟踪方式

1. 复用 `color_config.json` 的六种物料 HSV 阈值，在全画面提取连通域，并按面积、边长、长宽比、实心度和填充率筛选。触及图像边界的目标不作为完整检测。粉色标记像素不参与物料检测。
2. 蓝色与浅蓝色的掩膜先合并寻找物块，再在每个候选框内统计两者支持量。跨颜色高度重叠的候选去重，不对整幅画面做主色锁定，也不强制只保留三个目标。
3. 用二维位置与速度的卡尔曼滤波预测位置，用距离、检测框重叠、面积变化和颜色差异构建匈牙利匹配。关联后积累最近几帧的颜色支持量，减少短暂颜色跳变。
4. 初始轨迹连续出现 3 帧后确认；短暂丢失最多保留 0.5 秒的预测，再删除。删除、图像尺寸变化、长时间断帧或相机超时后，新目标获得新的 ID。分辨率、相机坐标系和时间基线变化不会沿用旧坐标。

固定速度模型用于短时间平滑与关联，不是转盘匀速圆周预测，也没有补偿相机/云台运动。`track_id` 是程序内的临时身份，不是任务环号或颜色编号；重新启动程序会重新编号。

## 发布数据

`/tracking_color/tracks` 是每张有效图像对应的 JSON：

| 字段 | 含义 |
|---|---|
| `stamp`、`frame_id`、`image_size` | 原图时间戳、相机坐标系及宽高 |
| `status` | `tracking` 或 `image_timeout` |
| `observed_confirmed_count` | 当前实际观测到并已确认的目标数 |
| `tracks[].track_id` | 跟踪 ID |
| `color_id` | 1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝 |
| `state` | `tentative` 待确认、`confirmed` 已确认且本帧观测到、`lost` 本帧丢失且仅预测 |
| `confirmed`、`observed` | 是否已确认轨迹、是否有本帧检测支持 |
| `center_px`、`bbox_px` | 滤波/预测中心 `[u,v]` 与检测框 `[x,y,w,h]`，单位像素 |
| `measurement_center_px` | 本帧原始色块质心；丢失时为 `null` |
| `velocity_px_s`、`position_std_px` | 图像速度 `[du/dt,dv/dt]`、滤波位置标准差 |
| `color_consistency` | 最近颜色加权投票的一致程度，不是识别正确概率 |
| `detection_score` | 最近检测的框内 HSV 像素支持比例，不是模型置信度 |
| `last_seen_sec`、`missed_sec` | 最近实际观测的图像时间、丢失时长 |
| `stationary` | 连续实际观测中滤波速度低于门槛；不是转盘停稳或允许抓取信号 |
| `history_px` | 历史滤波中心轨迹，只在有实际观测时追加 |

`/tracking_color/centers_pixel` 是带原图时间戳的像素 PointCloud，只包含 **本帧观测到且已经确认** 的目标。附带 `track_id`、`color_id`、面积、图像尺寸、速度、颜色一致度和 `stationary` 等逐点通道。丢失预测、待确认目标不进入这个话题；没有有效目标时发布空点云。

`/tracking_color/debug_image` 是标注图像。无新鲜图像超过 0.75 秒时，清空所有目标并发布一次 `image_timeout` 和空点云，标注图保留上次原图时间戳，不能当作新图像。所有消费者仍需检查时间戳；进程退出后不会持续发送清空消息。

## 调参和边界

默认参数在 `config/tracking_color.yaml`，可以复制后通过 `params_file:=/path/to/your.yaml` 指定。参数启动时读取，修改后重启跟踪节点。

| 参数 | 默认值 | 调节作用 |
|---|---|---|
| `min_blob_area` | 300.0 | 增大可排除小干扰，过大漏检远处物块 |
| `max_blob_area_fraction` | 0.15 | 允许的最大色块面积占全图比例，排除大背景 |
| `min_blob_dimension` | 12 | 候选框最小边长 |
| `max_aspect_ratio` | 3.0 | 越小越严格排除细长区域 |
| `min_solidity` / `min_extent` | 0.65 / 0.3 | 实心度/填充率门槛，遮挡或不规则物块可能被过滤 |
| `min_confirmed_hits` | 3 | 连续多少次观测后确认轨迹 |
| `max_match_distance_px` | 80.0 | 本帧检测距预测中心的最大关联距离 |
| `max_missed_sec` | 0.5 | 短时遮挡预测保留时间 |
| `color_window` | 7 | 颜色投票窗口，增大更稳但颜色变化响应更慢 |
| `stationary_speed_px_s` / `stationary_frames` | 8.0 / 5 | 连续低像素速度的判定门槛 |
| `publish_debug_image` | true | 关闭可减少调试图发布与绘制开销 |

本版本定位的是颜色轮廓质心，未检测物块顶面抓取点，也未转换成机械臂毫米坐标。全图中与物块颜色、形状相近的背景仍可能被当作目标；相邻同色物块接触时可能合并，遮挡和强反光也可能造成丢失或换 ID。粉色与真实红色阈值重叠时，标记剔除可能损失一部分红色像素，需要现场标定。

先对比窗口与实物，验证转动、停稳、短时遮挡和抓走一个目标后其余 ID 是否连续。之后再决定是否加入转盘工作空间过滤或目标检测模型，以及如何选择抓取点。当前节点独立测试，没有接入旧 `/color/centers_pixel`、坐标投影、任务选择或串口控制。
