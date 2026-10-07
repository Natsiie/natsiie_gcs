# 云台和相机 TF 骨架

```text
gimbal_base -- 实际 yaw，动态 --> gimbal_link -- 外参，静态 --> camera_optical_frame
```

仅实现绕竖直 Z 轴旋转的云台。坐标、TF 平移统一用米，角度用弧度。
光学系 X 向图像右、Y 向下、Z 向前；外参必须描述光学系在云台系中的姿态。
此节点不解析串口、不执行动作、不把像素直接转换成毫米。

## 填写参数

编辑 `src/identify_pack/config/tf_transform.yaml`：

- `base_to_gimbal_translation_m`：底座原点到旋转中心的位置；原点重合时为零。
- `camera_extrinsics_file`：填写外参标定 JSON 的绝对路径。
  文件中的 parent/child 必须与 YAML 的 gimbal/camera frame 一致。
- 或直接填写 `camera_translation_m`、`camera_quaternion_xyzw`，
  并将 `camera_transform_configured` 改为 true；四元数顺序为 x,y,z,w。
- `joint_states_topic`、`yaw_joint_name`：下位机角度接收节点的输出接口。
- `angle_sign`、`angle_offset_rad`：角度方向与零位补偿。

参数启动时读取，修改后重新启动。没有角度反馈时不发布假定的零位动态 TF；
没有实测外参时不发布相机静态 TF。每条 TF 关系只能由一个发布者维护。

## 角度输入接口

`serial_bridge_node` 接收 `TYPE=0x07` 的反馈帧，载荷为 ASCII 角度（单位度），
转换为弧度后发布下面的消息。不要用目标指令角度代替反馈。

反馈帧格式：`FF 07 LEN PAYLOAD XOR FE`，例如 `90.0` 的 payload 是 ASCII 字节。
校验是 `TYPE + LEN + PAYLOAD` 的逐字节异或，与 `protocol.py` 一致。
例如实际角度 90.0 度的完整反馈是 `FF 07 04 39 30 2E 30 14 FE`。
串口桥每 10 ms 检查输入缓冲区，支持分片/连续帧和校验失败后的重同步。
`GimbalYaw` 不会作为发送话题订阅。

在已加载 ROS 与工作区环境的终端运行：

```bash
ros2 run identify_pack serial_bridge_node --serial /dev/ttyUSB0 --baud 115200
ros2 topic echo /gimbal/joint_states
```

不要同时运行占用同一设备的串口测试脚本或旧串口桥。
串口桥的 `joint_states_topic` 和 `yaw_joint_name` 参数必须与 TF 节点一致；
正负方向及零位补偿由 TF 节点的 `angle_sign` 和 `angle_offset_rad` 处理。
串口桥不发布没有接收到的默认角度，非法/非有限角度不会更新话题。

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.header.stamp = node.get_clock().now().to_msg()
msg.name = ['gimbal_yaw_joint']
msg.position = [actual_yaw_rad]
publisher.publish(msg)  # topic: /gimbal/joint_states
```

上面的时间是串口接收时间。初期只在云台停稳后定位；运动定位需要同步下位机
采样时刻到 ROS 时钟，并使用真实采样时间。节点拒绝零时间戳、过期/未来反馈、
重复/倒序时间戳以及非法角度，不使用定时器将旧角度伪装为新反馈。

## 启动

在 `natsiie_gcs` 下，进入已 source ROS Humble 的终端：

```bash
colcon build --packages-select identify_pack --symlink-install
source install/setup.bash
ros2 launch identify_pack tf_transform.launch.py
```

启动时可指定另一份参数文件：

```bash
ros2 launch identify_pack tf_transform.launch.py params_file:=/absolute/path/tf_transform.yaml
```

修改源码配置后可直接指定源码 YAML，避免安装目录仍使用旧配置。

```bash
ros2 run tf2_ros tf2_echo gimbal_base camera_optical_frame
```

需要已配置相机外参且收到角度反馈，才能查询完整链路。

## 留给识别节点的接口

`TFTransformNode.point_to_base(PointStamped)` 提供按采集时间转换三维点的接口，
查询不到对应时间的 TF 时抛出 TF 异常，由调用方跳过目标。
这是 Python 方法，尚未连接颜色/圆环 ROS 话题。

像素投影阶段可通过 TF Buffer 查询
`lookup_transform('gimbal_base', 'camera_optical_frame', image_time)`，
将结果转成 4×4 矩阵，交给已有 `camera_geometry.pixel_to_plane`。
物块和圆环分别提供自己的底座平面 Z（m），共用这条 TF 链。
相机图像也需要填写 `header.frame_id=camera_optical_frame`。
平面高度、像素投影以及串口接收留在相应模块实现，不是 TF 安装参数。

## 像素点击到点验证

`pixel_to_base` 订阅已有相机的原始图像，不再打开摄像头设备；串口动作由串口桥发送。
内参、畸变和分辨率取自 `camera_extrinsics.json`；安装位姿实际取自图像时刻的 TF。
当前参数读取后必须重启节点才能生效。

1. 先运行相机节点、`serial_bridge_node` 和 `tf_transform`，保持实际角度反馈更新。
2. 编辑 `config/pixel_to_base.yaml`，设置实际图像话题，例如
   `/camera/image_raw` 或 `/camera/secondary/image_raw`。原始分辨率为 640×480，
   不要使用已经去畸变、裁剪、翻转或缩放的图像。
3. `plane_mode: calibration` 使用 JSON 的标定板高度（当前 -72.5 mm）。
   此模式要求底座与云台原点 Z 重合且旋转轴竖直（当前 TF 配置符合）。
   若底座原点高度不同，应配置 object/ring 平面为底座中的实际高度。
4. 物块顶面高度填写到 `object_plane_z_mm`，并将
   `object_plane_configured` 改为 true；圆环同理设置 `ring_plane_z_mm` 和
   `ring_plane_configured`。高度都是底座 Z（mm），不是镜头到物面的距离。

构建后，在已加载 ROS/工作区环境的终端运行：

```bash
ros2 launch identify_pack pixel_to_base.launch.py \
  params_file:=/home/lulu/gcs/natsiie_gcs/src/identify_pack/config/pixel_to_base.yaml
```

点击验证时设置 `show: true`、`auto_project: false`；当前默认开启自动颜色投影且不显示点击窗口。
左键点击显示底座 XYZ（mm），并发布 `/pixel_to_base/point`
（`geometry_msgs/PointStamped`，frame 为 `gimbal_base`，单位 m）。
Space 冻结/恢复，1 选标定板，2 选物块，3 选圆环，Q/Esc 退出。
未配置的平面不能选择；无该帧时间的 TF 时拒绝点击，不会使用最新角度替代。
图像冻结时仍使用冻结帧的原始时间与缓存 TF，不代表物料此刻的位置。

```bash
ros2 topic echo /pixel_to_base/point
```

选实际高度对应的平面，用未参与标定的固定点在零度和 30 度等角度停稳后点击。
比较底座 XYZ 是否与实测一致、是否在多个角度下保持一致。打印的 Z 是输入的
平面高度，不是独立测得的深度。相机话题当前未提供 frame_id 时，节点警告并使用
`camera_frame` 参数；非空但不匹配的 frame_id 会被拒绝。

图像时间戳与角度反馈时间戳尚有硬件采样和传输延迟，初期只在云台和目标停稳时验证。
颜色检测由颜色节点执行，通过下面的自动接口接入；圆环自动接口尚未接入。

## 自动颜色坐标发送

现在可用 `color_to_arm.launch.py` 一次启动副相机、颜色检测、TF、像素投影和串口桥。
无需鼠标点击。先关闭已经占用相机、串口或重复发布同一 TF 的节点。

```bash
cd /home/lulu/gcs/natsiie_gcs
source /opt/ros/humble/setup.bash
colcon build --packages-select identify_pack
source install/setup.bash
ros2 launch identify_pack color_to_arm.launch.py
```

已有相机发布图像时，可使用 `start_camera:=false`；查看颜色检测窗口用 `show:=true`。
设备与波特率可覆盖：`camera_device:=/dev/camera_upper serial_port:=/dev/ttyUSB0 baud:=115200`。
`pixel_params_file`、`tf_params_file` 可指定源码 YAML。配置修改后重新构建或指定源码参数文件。

自动链路：

```text
Image -> /color/centers_pixel -> /color/targets_base -> TYPE=0x08 -> STM32
          像素 u,v              基座 XYZ (m)         XYZ (mm)
```

`/color/centers_pixel` 使用 `sensor_msgs/PointCloud` 承载稀疏像素中心：
points.x=u、points.y=v、points.z=0；它不是米制点云，不能直接送入三维点云工具。
header 是原图的时间戳和 optical frame；channels 与 points 一一对应，包含
`color_id`、`area_px2`、`image_width`、`image_height`。
投影节点校验原图分辨率，按原图时间查询 TF；短暂缺少 TF 时最多等待 `tf_wait_sec`，
不使用最新 TF 替代。目标时间超过 `target_max_age_sec`（1 秒）则拒绝。

物块顶面已按实测填写 `object_plane_z_mm: -12.5`、`object_plane_configured: true`。
自动颜色投影始终用 object 平面，GUI 的其他平面选择仅用于点击检查。
同色多个色块取面积最大的一个；识别到不同颜色时分别发送。空检测不发送旧目标。
`/color/targets_base` 同样使用 PointCloud，points 为 `gimbal_base` XYZ，单位 m，
channels 中的 `color_id` 保留每个目标颜色。串口桥将 XYZ 乘 1000，仅在此处换成 mm。

```bash
ros2 topic echo /color/centers_pixel
ros2 topic echo /color/targets_base
```

目标帧：`FF 08 LEN PAYLOAD XOR FE`，PAYLOAD 是 ASCII 文本
`color_id,x_mm,y_mm,z_mm`，例如 `1,100.0,0.0,-12.5`。
LEN 是文本字节长度；XOR 对 TYPE、LEN、PAYLOAD 逐字节异或，名称沿用 CRC8，实际并非多项式 CRC。
上述例子完整帧：`FF 08 11 31 2C 31 30 30 2E 30 2C 30 2E 30 2C 2D 31 32 2E 35 30 FE`。
颜色编号：1红、2黄、3蓝、4绿、5黑、6浅蓝。
XYZ 是固定底座原点下的绝对坐标，不是像素、增量或电机脉冲。
投影间隔默认 0.2 秒；串口桥 `target_max_hz` 默认每种颜色最多 5 Hz。
连续帧是目标坐标更新，不应让下位机每帧重新排队执行一遍完整抓取。

统一启动关闭旧 `/Color`、`/ColorMotion` 串口转发（`send_legacy_color: false`），
保留 ROS 发布；其他旧检测话题的串口转发仍按原配置。
云台角度反馈仍用 `0x07`。单独运行串口桥默认保留旧转发，可用
`--ros-args -p send_legacy_color:=false -p target_max_hz:=5.0` 改为新目标链路。

本次仅修改上位机。仓库中的下位机接收仍是固定 8 字节，无法解析上述变长坐标帧；
下位机需要由其维护者支持 TYPE/LEN/XOR 和 0x08，才能接入运动函数。
上位机发出的 Z 是视觉目标表面高度；下位机须按自身夹爪和升降原点设置抓取高度。
当前没有到位/忙碌应答、抓取次序或任务锁定，多个颜色的选择和执行由下位机处理。
尚未实测自动定位时，应先验证接收数值和单个目标的到点行为，再进行抓取。
