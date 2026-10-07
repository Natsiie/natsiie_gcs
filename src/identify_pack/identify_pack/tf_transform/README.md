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
