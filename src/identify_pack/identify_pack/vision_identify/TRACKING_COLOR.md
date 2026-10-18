# 全图多物块颜色跟踪

`tracking_color_node.py` 默认先在全图检测物块的圆/椭圆顶面，再在顶面内部识别颜色并跟踪。依据比赛规则第 5 页图 5，初赛物料为上直径 30 mm、下直径 50 mm、总高 60 mm 的带上下圆柱边缘的圆台。原 ROI 文件和颜色识别节点保留；原全图 HSV 色块检测可切换作对照。当前 `race_start.launch.py` 临时只启用上相机与全图颜色跟踪，其余功能节点已注释保留。节点持续检测转动中的物块，不等待停稳，也不发送串口或机械臂指令。

## 启动

```bash
cd ~/gcs/natsiie_gcs
colcon build --packages-select identify_pack
source install/setup.bash

# 当前比赛入口已切为跟踪调试，只启动上相机和跟踪窗口
ros2 launch identify_pack race_start.launch.py show:=true

# 对照原全图 HSV 色块模式
ros2 launch identify_pack race_start.launch.py tracking_detector_mode:=hsv_blob show:=true

# 已有相机节点发布 /camera/secondary/image_raw 时
ros2 launch identify_pack tracking_color.launch.py show:=true
```

两个入口任选一个，不要同时启动两套跟踪节点。当前 `race_start` 不启动旧任务和串口链路，可用 `tracking_color_params_file:=/path/to/your.yaml` 覆盖跟踪参数；`show:=false` 可关闭窗口。`race_start` 用 `tracking_detector_mode` 选择检测模式，独立 launch 用 `detector_mode`，这两个启动参数均会覆盖 YAML 中的 `detector_mode`。若使用独立 launch 并需要它启动相机：

```bash
# 相机没有被其他程序占用时：只启动一台相机和跟踪节点
ros2 launch identify_pack tracking_color.launch.py start_camera:=true show:=true

# 或已有其他图像话题
ros2 launch identify_pack tracking_color.launch.py image_topic:=/camera/image_raw show:=true

# 独立入口切换为原色块检测
ros2 launch identify_pack tracking_color.launch.py detector_mode:=hsv_blob show:=true
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
ros2 topic hz /tracking_color/debug_image
```

也可直接运行节点并传参数：

```bash
ros2 run identify_pack tracking_color_node --ros-args \
3. 用二维位置与速度的卡尔曼滤波预测位置，用距离、检测框重叠、面积变化和颜色差异构建匈牙利匹配。关联后积累最近几帧的颜色支持量，减少短暂颜色跳变。
4. 初始轨迹连续出现 3 帧后确认；短暂丢失最多保留 0.5 秒的预测，再删除。删除、图像尺寸变化、长时间断帧或相机超时后，新目标获得新的 ID。分辨率、相机坐标系和时间基线变化不会沿用旧坐标。

固定速度模型用于短时间平滑与关联，不是转盘匀速圆周预测，也没有补偿相机/云台运动。`track_id` 是程序内的临时身份，不是任务环号或颜色编号；重新启动程序会重新编号。

`hsv_blob` 模式保留原全图六色掩膜、连通域与几何筛选；蓝/浅蓝先合并分割再逐候选判色，定位使用色块质心。两种模式共用同一套 HSV 配置和跟踪逻辑，顶面模式拒绝的目标不会自动回退到色块模式。

## 发布数据

`/tracking_color/tracks` 是每张有效图像对应的 JSON：

| 字段 | 含义 |
|---|---|
| `stamp`、`frame_id`、`image_size` | 原图时间戳、相机坐标系及宽高 |
| `status` | `tracking` 或 `image_timeout` |
| `detector_mode` | `material_top` 或 `hsv_blob` |
| `top_candidates` | 顶面候选及通过/拒绝原因、相关指标；通过表示检测筛选通过，重叠去重后数量仍可能减少 |
| `observed_confirmed_count` | 当前实际观测到并已确认的目标数 |
| `tracks[].track_id` | 跟踪 ID |
| `color_id` | 1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝 |
| `state` | `tentative` 待确认、`confirmed` 已确认且本帧观测到、`lost` 本帧丢失且仅预测 |
| `confirmed`、`observed` | 是否已确认轨迹、是否有本帧检测支持 |
| `center_px`、`bbox_px` | 滤波/预测中心 `[u,v]` 与检测框 `[x,y,w,h]`，单位像素 |
| `measurement_center_px` | 本帧原始顶面椭圆中心（色块模式为质心）；丢失时为 `null` |
| `top_ellipse_px` | 本帧顶面 `[[中心u,v],[两轴直径],角度]`，采用 OpenCV 定义；色块模式或丢失时为 `null` |
| `shape_score`、`detector` | 最近椭圆边缘覆盖与拟合误差的组合评分、检测来源；评分不是物块正确概率 |
| `velocity_px_s`、`position_std_px` | 图像速度 `[du/dt,dv/dt]`、滤波位置标准差 |
| `color_consistency` | 最近颜色加权投票的一致程度，不是识别正确概率 |
| `detection_score` | 最近检测的顶面内部 HSV 支持比例（色块模式为框内比例），不是模型置信度 |
| `last_seen_sec`、`missed_sec` | 最近实际观测的图像时间、丢失时长 |
| `stationary` | 连续实际观测中滤波速度低于门槛；不是转盘停稳或允许抓取信号 |
| `history_px` | 历史滤波中心轨迹，只在有实际观测时追加 |

`/tracking_color/centers_pixel` 是带原图时间戳的像素 PointCloud，只包含 **本帧观测到且已经确认** 的目标。附带 `track_id`、`color_id`、面积、图像尺寸、速度、颜色一致度和 `stationary` 等逐点通道。丢失预测、待确认目标不进入这个话题；没有有效目标时发布空点云。

`/tracking_color/debug_image` 是标注图像。无新鲜图像超过 0.75 秒时，清空所有目标并发布一次 `image_timeout` 和空点云，标注图保留上次原图时间戳，不能当作新图像。所有消费者仍需检查时间戳；进程退出后不会持续发送清空消息。

## 调参和边界

默认参数在 `config/tracking_color.yaml`，可以复制后通过 `params_file:=/path/to/your.yaml` 指定。参数启动时读取，修改后重启跟踪节点。

建议先启动顶面模式观察，再按以下顺序调参：

1. **确定顶面的像素尺寸**：30 mm 顶面在画面中的直径取决于相机高度和姿态。`top_min_minor_axis_px` 是最小短轴直径，`top_max_major_axis_px` 是最大长轴直径，都不是半径，也不是毫米；缩紧尺寸范围能排除许多场地圆环和大阴影。
2. **查看拒绝原因**：将 YAML 中 `show_rejected_tops` 改为 `true`，窗口会用灰色标出未通过的椭圆和原因。JSON 的 `top_candidates` 始终提供诊断。完全没有候选时先检查尺寸、扁率和原图边缘；这些初筛不画灰色椭圆。
3. **校准颜色**：仍用 `color_calibrator.py` 修改原 `color_config.json`，在真实顶面上采样。出现 `unknown_color` 时先检查 HSV 阈值和反光，再调支持量，不要直接扩大黑色 V 上限来接收阴影。修改颜色配置后需重新构建并重启，或传入源文件绝对路径。
4. **固定斜拍视角后再启用侧面检查**：侧面在顶面下方用 `top_body_direction: 1`，在上方用 `-1`；完全俯视或侧面朝左/右保持 `0`。侧面采样按 60/30 高径比与 50/30 上下径比估计，不是三维姿态解算；颜色侧面太暗、不在 HSV 阈值内或被遮挡时会漏检。

| 顶面参数 | 默认值 | 调节作用 |
|---|---|---|
| `top_min_minor_axis_px` / `top_max_major_axis_px` | 12.0 / 120.0 | 顶面短轴最小、长轴最大直径，优先按实物画面调节 |
| `top_min_aspect` | 0.35 | 短轴/长轴下限，过高会漏掉斜拍顶面 |
| `top_canny_low` / `top_canny_high` | 35 / 100 | 边缘低/高门槛；降低可找弱顶面边缘，也会增加噪声 |
| `top_edge_close_kernel_px` | 5 | 连通边缘断点的奇数核大小；过大可能连接相邻目标 |
| `top_max_fit_error` / `top_min_contour_fill` | 0.07 / 0.75 | 归一化径向拟合误差上限、轮廓与拟合椭圆面积比例下限 |
| `top_min_edge_coverage` / `top_edge_tolerance_px` | 0.70 / 2.0 | 真实边缘覆盖下限及距拟合边界的容差 |
| `top_min_boundary_contrast` | 10.0 | 顶面边缘内外灰度差的中位数下限 |
| `top_inner_ratio` | 0.70 | 判色椭圆轴长/顶面轴长，缩小可减少边缘与侧面污染 |
| `top_min_color_support` / `top_min_color_margin` | 0.45 / 0.08 | 最佳颜色像素比例及相对第二名的差值下限 |
| `top_black_min_edge_coverage` | 0.82 | 黑色的更严格边缘覆盖率 |
| `top_black_min_dark_boundary` | 0.45 | 外围比内部亮至少上述灰度差的边界比例下限；斜拍时暗侧面可能占近一半外围 |
| `top_black_max_outer_support` | 0.55 | 外围采样点允许的黑色比例上限 |
| `top_duplicate_overlap` | 0.60 | 候选框交集/较小框面积达到门槛时去重 |
| `top_body_direction` / `top_min_body_support` | 0 / 0.35 | 可选侧面方向及同色支持比例 |

| 灰色拒绝提示 | 含义 |
|---|---|
| `ellipse_fit` | 轮廓不像完整圆/椭圆，可能有拖尾、侧面混入或断边 |
| `clipped` | 顶面及外围检查区域靠近图像边界 |
| `weak_boundary` | 真实边缘覆盖不足或内外对比太低 |
| `small_interior` | 顶面判色区域太小 |
| `unknown_color` | HSV 支持不足或多色阈值歧义 |
| `black_shadow_check` | 黑色候选边缘或外围明暗关系不符合要求 |
| `body_structure` | 已启用侧面检查，但指定方向的同色支持不足 |

以下色块参数仅对 `hsv_blob` 生效；跟踪和发布参数对两种模式均生效：

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

顶面模式给出的是图像中的椭圆中心，不是标定后的物理顶面中心，透视下两者可能有偏差，尚未转换成机械臂毫米坐标。形状筛选不能证明候选一定是物块：**边缘清晰的圆形阴影、尺寸相近的场地圆环或背景圆斑仍可能通过**。顶面与侧面明暗相同、黑色物块边缘看不清、遮挡或强反光时可能漏检；相邻目标接触可能合并或换 ID。此模式针对规则中的初赛圆台，不能直接覆盖决赛的可变形状。粉色与红色阈值重叠时，标记剔除可能损失红色像素，需要现场标定。

先对比窗口与实物，验证转动、停稳、短时遮挡和抓走一个目标后其余 ID 是否连续。之后再决定是否加入转盘工作空间过滤或目标检测模型，以及如何选择抓取点。当前节点独立测试，没有接入旧 `/color/centers_pixel`、坐标投影、任务选择或串口控制。
