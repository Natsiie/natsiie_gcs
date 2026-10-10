# 圆环与环内数字独立测试

`ring_digit_detect.py` 和 `ring_station_digit_test.py` 使用同一套融合识别核心，不需要 ROS、串口或模型权重，依赖 Python 3、OpenCV、NumPy。

## 方法与来源

- 圆环：融合 `gcs-gold-medal/cv2_python/zongchengxu20250724chusai.py` 的灰度、高斯滤波、霍夫圆定位与现有 Canny、椭圆拟合、同心轮廓聚类、三环共线筛选。近正视时霍夫圆作局部圆心复核，斜拍时保留椭圆轴长做数字裁剪矫正；圆心用浮点数和连续帧中值输出，不沿用旧程序的颜色环号推断。
- 数字：cyqm 原程序没有数字识别，按 `ring1_side` 指定环号。本脚本新增中心裁剪、Otsu 二值化、背景极性判断、连通域筛选和 1/2/3 模板匹配。
- 已确认两个区域的相机画面始终左到右为 1、2、3。`slot` 是画面候选序号，`digit` 是模板真正读出的数字（遮挡为 null），`ring_id` 是用于放置的固定位置编号。可见数字不符合固定顺序时，`numbering_conflict` 为 true，停止输出放置目标。
- 规则中的 1~6 环是评分圈层，位置数字是 1/2/3，两个编号含义不同。580×150 mm 区域、图示 150 mm 圆心间距可用于几何参考；现场尺寸和字体需要实测。

## 直接看相机

停止占用该相机的 ROS 节点，再运行：

```bash
cd ~/gcs/natsiie_gcs/src/identify_pack/identify_pack/scripts
python3 ring_digit_detect.py --device /dev/camera_upper --place-order 231
```

默认请求 640×480、25 fps，相机格式沿用驱动默认值。主窗口显示圆心、椭圆、实际数字和相似度；`Digit crops` 显示矫正后的环和提取数字。无数字或匹配不确定时显示 `?`。

默认 `--flip-code 0` 表示不翻转，与当前 launch 一致。相机画面倒置可加 `--rotate 180`；90/270 表示顺时针旋转。镜像画面应先通过 `--flip-code 1` 恢复，数字方向需要和采集模板一致。

固定编号模式要求最终处理画面仍然左到右为 1、2、3；旋转或镜像改变这个顺序时应恢复画面方向，或用 `--numbering templates` 单独验证数字识别。

按 `S` 保存处理后的原图、标注图、数字预览及 JSON 到 `ring_station_digit_snapshots/`，按 `Q` 或 Esc 退出。像素坐标属于旋转/翻转后的处理图像，不能直接用于原图坐标或机械臂控制。

## 采集现场字体

默认已提供从三张实拍快照提取且人工核对标签的 9 张字体模板，1/2/3 各 3 张。原 `digit_templates/` 存在文件名与数字图形不符的样本，因此保留原文件，融合入口使用 `ring_station_digit_templates/`。更换现场字体后应重新采集：

```bash
python3 ring_station_digit_test.py --device /dev/camera_upper --capture
```

鼠标点击主窗口中的某个环，选中圆心显示紫色标记。确认 `Digit crops` 中提取的形状确实是数字，再按它的真实数字键 `1`、`2` 或 `3`。只保存选中的一个数字，按多次可采不同距离和光照下的样本。相机移动导致选中环超出跟踪范围时，重新点击选择。

模板保存在独立目录 `ring_station_digit_templates/`，不使用或覆盖现有 `digit_templates/`。每个数字有现场模板后，该数字改用现场模板；按键保存后立即重新加载。错误模板可从目录删除，重新运行脚本。

## 图片离线测试

```bash
python3 ring_station_digit_test.py --image /path/to/photo.png --no-show --output-dir /tmp/ring_test
```

只处理一张图，输出 JSON 并保存结果。去掉 `--no-show` 可交互查看，也可与 `--capture` 一起用于采模板。图片模式按键后会重新处理同一张图。

## 调参

圆环筛选默认读取脚本旁的 `ring_station_detector.json`；也可用 `--config` 指定另一份。只使用检测参数，不要求对位零点或底盘纠偏标定通过。命令行调参不写回配置：

```bash
python3 ring_station_digit_test.py --device /dev/camera_upper \
  --max-major-axis 300 --min-minor-axis 10 --canny-low 40 --canny-high 120 \
  --inner-ratio 0.55 --min-confidence 0.55 --min-margin 0.08
```

- 没有圆环：先检查边缘清晰度、圆环像素大小是否在轴长筛选范围内；当前配置最大长轴为 160 px，近距离时可能需要增大。
- 数字裁剪带环线：减小 `--inner-ratio`；数字被裁断则适当增大。该比例相对最外椭圆半径，默认 0.55。
- 数字预览正确但显示 `?`：优先采集真实字体，再检查最高相似度和与次高分的差值。相似度不是概率；降低门槛会增加误识别。
- 单环/双环可显示候选，但 `three_ring_geometry` 为 false。`complete_digit_mapping` 仅表示三个真实数字都读到且为 1/2/3 的排列；`placement_mapping_valid` 表示根据当前编号方式可确定放置编号，两者含义不同。

## 已接入正式放置链路

`ring_station_detect_node` 现在同时使用三圆环椭圆检测、局部霍夫圆复核和现场数字模板。默认模板目录是安装后的 `ring_station_digit_templates/`；正式节点要求每个数字至少有一张真实模板，缺失时启动报错。每帧发布状态，配置中的稳定窗口（当前 7 帧）完成后才标记可放置：

- `/ring_station/digits`：JSON，包含每个圆心、实际数字、相似度、稳定帧数和 `centers_by_digit_px`；
- `/ring_station/centers_pixel`：点按打印数字 1、2、3 排序，通道 `ring_id` 与点索引一致；
- `digit_mapping_valid`、`digit_mapping_stable`：本项目已知相机视角固定为左到右 1、2、3，因此数字被物块遮住时仍可按几何位置编号；圆环缺失、顺序冲突、跳变或圆心抖动时为 0。

放置协调器对 `ring_station` 阶段要求两个数字通道都为 1，任务码中的环号才会取对应数字的圆心。环号绑定固定为画面左、中、右分别是 1、2、3；模板数字只作现场核验，显示数字与固定顺序冲突会拒绝本帧。`stacked_ring` 仍使用其自身的几何检测结果。

单帧/离线测试默认也使用固定顺序；需要专门验证模板编号时加 `--numbering templates`。`--task-code 156+123+516+231 --batch 2` 会输出第二批的放置环顺序 `2,3,1`。

`placement_targets_px` 是当前帧按任务码排列的像素预览；单张图片不能证明连续帧稳定，真实动作还要求 `digit_mapping_stable`、几何稳定、对位容差、正确标定及新鲜图像时间戳。整环被遮住导致三环几何丢失时不使用历史圆心继续放置。

构建和查看正式输出：

```bash
cd ~/gcs/natsiie_gcs
source /opt/ros/humble/setup.bash
colcon build --packages-select identify_pack --symlink-install
source install/setup.bash
ros2 topic echo /ring_station/digits
```

`race_start.launch.py` 的圆环节点已使用融合逻辑。更换模板目录可通过 `ring_digit_templates:=/absolute/path` 覆盖。仅观察、不发送动作时，先独立运行上述相机测试脚本。

斜拍矫正是椭圆局部仿射近似，强透视、数字旋转、反光和物料遮挡仍可能失败；比赛前应使用现场相机高度、光照和字体重新采集模板，并用 `/ring_station/digits` 的 JSON 检查三环映射稳定后再启用自动放置。
