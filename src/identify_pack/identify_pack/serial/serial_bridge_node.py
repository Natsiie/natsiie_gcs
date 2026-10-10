#!/usr/bin/env python3
"""统一串口桥：发送检测结果，接收 STM32F4 实际云台角度并发布 JointState。

取代原 qr_serial_bridge（两者不能同时运行，串口设备只能被一个进程打开）。
串口参数优先级：命令行 > serial_config.json > 默认值。
"""
import os
import sys
import json
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt32
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud

from identify_pack.serial.protocol import (
    GIMBAL_YAW_TYPE, GRAB_CMD_TYPE, OUTGOING_TOPIC_TYPES, extract_frames,
    make_frame, COLOR_TARGET_TYPE, RING_TARGET_TYPE,
    PLACE_COMMAND_TYPE, PLACE_RESULT_TYPE, PLACE_CONTROL_TYPE,
    parse_place_result, parse_place_control, place_command_payload,
)
from identify_pack.scripts.pick_task import PickTask
from identify_pack.scripts.color_config import PICKUP_COLOR_IDS

try:
    import serial
except Exception:
    serial = None


class SerialBridgeNode(Node):
    def __init__(self, serial_port: str, baud: int, timeout: float = 0.1):
        super().__init__('serial_bridge_node')
        self.declare_parameter('joint_states_topic', '/gimbal/joint_states')
        self.declare_parameter('yaw_joint_name', 'gimbal_yaw_joint')
        self.declare_parameter('yaw_feedback_type', GIMBAL_YAW_TYPE)
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.yaw_feedback_type = int(self.get_parameter('yaw_feedback_type').value)
        self.declare_parameter('target_topic', '/color/targets_base')
        self.declare_parameter('target_type', COLOR_TARGET_TYPE)
        self.declare_parameter('target_max_hz', 5.0)
        self.declare_parameter('target_max_age_sec', 1.0)
        self.declare_parameter('target_frame', 'gimbal_base')
        self.declare_parameter('send_legacy_color', True)
        self.declare_parameter('task_mode', False)
        self.declare_parameter('task_code_resend_interval_sec', 1.0)
        self.declare_parameter('task_code_send_duration_sec', 30.0)
        self.declare_parameter('wait_between_batches', True)
        self.declare_parameter('centers_topic', '/color/centers_pixel')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('arrived_topic', '/arm/arrived')
        self.declare_parameter('ring_target_topic', '/ring_station/targets_base')
        self.declare_parameter('ring_target_type', RING_TARGET_TYPE)
        self.target_topic = self.get_parameter('target_topic').value
        self.target_type = int(self.get_parameter('target_type').value)
        self.target_max_hz = float(self.get_parameter('target_max_hz').value)
        self.target_max_age = float(self.get_parameter('target_max_age_sec').value)
        self.target_frame = self.get_parameter('target_frame').value
        if not self.joint_states_topic or not self.yaw_joint_name:
            raise ValueError('joint_states_topic and yaw_joint_name cannot be empty')
        if self.yaw_feedback_type != GIMBAL_YAW_TYPE:
            raise ValueError(f'yaw_feedback_type must be 0x{GIMBAL_YAW_TYPE:02X}')
        if self.target_type != COLOR_TARGET_TYPE:
            raise ValueError(f'target_type must be 0x{COLOR_TARGET_TYPE:02X}')
        if not math.isfinite(self.target_max_hz) or self.target_max_hz <= 0:
            raise ValueError('target_max_hz must be finite and positive')
        if not math.isfinite(self.target_max_age) or self.target_max_age <= 0:
            raise ValueError('target_max_age_sec must be finite and positive')
        if not self.target_topic or not self.target_frame:
            raise ValueError('Target topic and frame cannot be empty')
        self._last_target_send = {}
        self._last_target_stamp = {}
        self.task_mode = self.get_parameter('task_mode').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.task = PickTask(self.get_parameter('wait_between_batches').value)
        self.motion_stamp_ns = 0
        self.ready_since_ns = 0
        self.motion_ready = False
        self.motion_receive_time = -math.inf
        self.task_code_sent = False
        self.task_code = None
        self.task_code_send_deadline = None
        self.pub_task_status = self.create_publisher(
            String, '/arm/task_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.task_mode:
            self.create_subscription(String, 'QR_Decode', self._on_task, 10)
            self.create_subscription(UInt32, '/arm/pick_done', self._on_pick_done, 10)
            self.create_subscription(UInt32, '/arm/next_batch', self._on_next_batch, 10)
            self.create_subscription(PointCloud, self.get_parameter('centers_topic').value,
                                     self._on_motion, 1)
            self._publish_task_status()
            resend_interval = float(self.get_parameter('task_code_resend_interval_sec').value)
            if not math.isfinite(resend_interval) or resend_interval <= 0:
                raise ValueError('task_code_resend_interval_sec must be finite and positive')
            self.task_code_send_duration = float(
                self.get_parameter('task_code_send_duration_sec').value)
            if not math.isfinite(self.task_code_send_duration) or self.task_code_send_duration <= 0:
                raise ValueError('task_code_send_duration_sec must be finite and positive')
            self.create_timer(resend_interval, self._resend_task_code)
        self.ser = None
        self.rx_buffer = bytearray()
        self.pub_joint_state = self.create_publisher(
            JointState, self.joint_states_topic, 10)
        self.pub_place_result = self.create_publisher(String, '/arm/place_result', 10)
        self.create_subscription(String, '/arm/place_command', self._on_place_command, 10)
        self.declare_parameter('place_control_topic', '/arm/place_control')
        self.pub_place_control = self.create_publisher(
            String, self.get_parameter('place_control_topic').value, 10)
        if serial is None:
            self.get_logger().error('pyserial 未安装，无法发送串口')
        else:
            try:
                self.ser = serial.Serial(port=serial_port, baudrate=int(baud),
                                         timeout=float(timeout), write_timeout=0.1)
                self.get_logger().info(f'已打开串口 {serial_port} @ {baud}')
                self.rx_timer = self.create_timer(0.01, self._poll_feedback)
            except Exception as e:
                self.get_logger().error(f'打开串口失败: {e}')

        for topic, type_byte in OUTGOING_TOPIC_TYPES.items():
            if topic == 'QR_Decode' and self.task_mode:
                continue  # 任务模式中只转发首次通过校验的任务码。
            if (topic in ('Color', 'ColorMotion')
                    and not self.get_parameter('send_legacy_color').value):
                continue
            # 用默认参数闭包捕获当前 topic/type，避免 lambda 晚绑定
            self.create_subscription(
                String, topic,
                lambda msg, t=type_byte, name=topic: self._send(t, name, msg.data),
                10)
        self.create_subscription(PointCloud, self.target_topic,
                                 self._send_color_targets, 1)
        self.ring_target_topic = self.get_parameter('ring_target_topic').value
        self.ring_target_type = int(self.get_parameter('ring_target_type').value)
        if self.ring_target_type != RING_TARGET_TYPE:
            raise ValueError(f'ring_target_type must be 0x{RING_TARGET_TYPE:02X}')
        self.create_subscription(PointCloud, self.ring_target_topic,
                                 self._send_ring_targets, 1)
        arrived_topic = self.get_parameter('arrived_topic').value
        if arrived_topic:
            self.create_subscription(Bool, arrived_topic, self._on_arrived, 10)

    def _publish_task_status(self):
        pick = self.task.expected
        status = {'task_code': self.task.code, 'state': self.task.state,
                  'sequence': pick.sequence if pick else 0,
                  'batch': pick.batch if pick else 0,
                  'color': pick.color if pick else 0, 'ring': pick.ring if pick else 0,
                  'plan': [{'sequence': p.sequence, 'batch': p.batch,
                            'color': p.color, 'ring': p.ring} for p in self.task.picks]}
        self.pub_task_status.publish(String(data=json.dumps(status)))
        self.get_logger().info(f'抓取任务: {status}')

    def _on_task(self, msg):
        try:
            code = msg.data.strip()
            changed = self.task.load(code, self.get_clock().now().nanoseconds)
            self.task_code = code
            duration = getattr(self, 'task_code_send_duration', 30.0)
            self.task_code_send_deadline = time.monotonic() + duration
            if not self.task_code_sent:
                self.task_code_sent = self._send(
                    OUTGOING_TOPIC_TYPES['QR_Decode'], 'QR_Decode', code)
            if changed:
                self._publish_task_status()
        except ValueError as error:
            self.get_logger().warning(f'任务码拒绝: {error}', throttle_duration_sec=2.0)

    def _resend_task_code(self):
        """周期重发任务码，持续 task_code_send_duration_sec 后停止。"""
        if self.task_code is None:
            return
        deadline = getattr(self, 'task_code_send_deadline', None)
        if deadline is not None and time.monotonic() >= deadline:
            self.get_logger().info('任务码发送时长已到，停止重发')
            self.task_code = None
            self.task_code_send_deadline = None
            return
        if self._send(OUTGOING_TOPIC_TYPES['QR_Decode'], 'QR_Decode', self.task_code):
            self.task_code_sent = True

    def _on_pick_done(self, msg):
        if self.task.complete(msg.data, self.get_clock().now().nanoseconds):
            self._publish_task_status()
        else:
            self.get_logger().warning(f'忽略抓取完成序号 {msg.data}：当前状态或序号不符')

    def _on_next_batch(self, msg):
        if self.task.next_batch(msg.data, self.get_clock().now().nanoseconds):
            self._publish_task_status()
        else:
            self.get_logger().warning('忽略批次通知：需要第一批三个抓取确认后发送 data=1')

    def _on_motion(self, msg):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp_ns <= self.motion_stamp_ns:
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        values = [channel.values for channel in msg.channels if channel.name == 'motion_ready']
        valid = (msg.header.frame_id == self.camera_frame and stamp_ns > 0
                 and -0.05 <= age <= self.target_max_age
                 and len(values) == 1 and len(values[0]) == 1 and values[0][0] == 1.0)
        if valid and not self.motion_ready:
            self.ready_since_ns = stamp_ns
        self.motion_ready = valid
        self.motion_stamp_ns = stamp_ns
        self.motion_receive_time = time.monotonic()

    def _poll_feedback(self):
        """读取并解析下位机反馈；角度载荷为 ASCII 度数，例如 ``90.0``。"""
        if not self.ser or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                chunk = self.ser.read(waiting)
                self.get_logger().debug(
                    f'RX raw ({len(chunk)} B): hex={chunk.hex(" ")} '
                    f'text={chunk.decode("utf-8", errors="replace")!r}')
                self.rx_buffer.extend(chunk)
            frames, remainder = extract_frames(self.rx_buffer)
            valid_bytes = sum(5 + len(payload.encode('utf-8')) for _, payload in frames)
            discarded = len(self.rx_buffer) - len(remainder) - valid_bytes
            if discarded > 0:
                self.get_logger().warning(
                    f'收到串口字节，但有 {discarded} 字节未通过帧解析；'
                    '需要 FF TYPE LEN ASCII载荷 XOR FE，云台角度 TYPE=07。'
                    '使用 --ros-args --log-level serial_bridge_node:=debug 查看原始 HEX。',
                    throttle_duration_sec=2.0)
            self.rx_buffer = bytearray(remainder)
        except Exception as error:
            self.get_logger().error(f'串口读取失败: {error}', throttle_duration_sec=2.0)
            return
        for type_byte, payload in frames:
            if type_byte == PLACE_RESULT_TYPE:
                try:
                    result = parse_place_result(payload)
                    self.pub_place_result.publish(String(data=json.dumps(result)))
                except ValueError as error:
                    self.get_logger().warning(f'Invalid placement feedback: {error}')
                continue
            if type_byte == PLACE_CONTROL_TYPE:
                try:
                    control = parse_place_control(payload)
                    self.pub_place_control.publish(String(data=json.dumps(control)))
                except ValueError as error:
                    self.get_logger().warning(f'Invalid placement control: {error}')
                continue
            if type_byte != self.yaw_feedback_type:
                self.get_logger().warning(
                    f'忽略反馈帧 type=0x{type_byte:02X} payload={payload!r}；'
                    f'云台角度需要 type=0x{self.yaw_feedback_type:02X}',
                    throttle_duration_sec=2.0)
                continue
            try:
                yaw_deg = float(payload.strip())
                if not math.isfinite(yaw_deg):
                    raise ValueError('角度不是有限数')
                yaw_rad = math.radians(yaw_deg)
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = [self.yaw_joint_name]
                message.position = [yaw_rad]
                self.pub_joint_state.publish(message)
                self.get_logger().info(
                    f'RX GimbalYaw: {yaw_deg:.3f} deg ({yaw_rad:.6f} rad) '
                    f'-> {self.joint_states_topic}', throttle_duration_sec=1.0)
            except (TypeError, ValueError, OverflowError) as error:
                self.get_logger().warning(
                    f'忽略非法云台角度 payload={payload!r}: {error}',
                    throttle_duration_sec=2.0)

    def _send(self, type_byte: int, topic: str, text: str):
        try:
            frame = make_frame(type_byte, text)
        except Exception as e:
            self.get_logger().error(f'打包失败 topic={topic}: {e}')
            return False
        if self.ser and self.ser.is_open:
            try:
                written = self.ser.write(frame)
                if written != len(frame):
                    raise IOError(f'串口只写入 {written}/{len(frame)} 字节')
                self.get_logger().info(
                    f'TX {topic}(0x{type_byte:02X}) len={len(frame)}: {text!r}')
                return True
            except Exception as e:
                self.get_logger().error(f'串口写入失败: {e}')
        else:
            self.get_logger().info(f'(无串口) {topic}(0x{type_byte:02X}): {text!r}')
        return False

    def _send_ring_targets(self, message: PointCloud):
        """发送三圆环圆心（gimbal_base 毫米）给下位机，payload 为 x1,y1,x2,y2,x3,y3（不含 Z）。"""
        if message.header.frame_id != self.target_frame:
            self.get_logger().warning(
                f'忽略圆环目标：frame_id={message.header.frame_id!r}，需要 {self.target_frame}',
                throttle_duration_sec=2.0)
            return
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age < -0.05 or age > self.target_max_age:
            self.get_logger().warning('忽略圆环目标：图像时间戳缺失、过期或超前',
                                      throttle_duration_sec=2.0)
            return
        if len(message.points) != 3:
            self.get_logger().warning('忽略圆环目标：圆心数不是 3 个')
            return
        values = []
        for point in message.points:
            xy = (point.x * 1000.0, point.y * 1000.0)
            if not all(math.isfinite(v) for v in xy):
                self.get_logger().warning('忽略圆环目标：坐标非有限')
                return
            values.extend(xy)
        payload = ','.join(f'{v:.1f}' for v in values)
        self._send(self.ring_target_type, 'RingTarget', payload)

    def _on_place_command(self, msg):
        try:
            command = json.loads(msg.data)
            payload = place_command_payload(command, self.target_frame)
            if not self._send(PLACE_COMMAND_TYPE, 'PlaceCommand', payload):
                self.pub_place_result.publish(String(data=json.dumps({
                    'task_id': command['task_id'], 'command_id': command['command_id'],
                    'success': False})))
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f'Invalid placement command: {error}')

    def _on_arrived(self, msg):
        """收到夹爪到位信号 -> 下发下降抓取授权帧（GrabCmd，payload "1"）。"""
        if not msg.data:
            return
        self.get_logger().info('收到夹爪到位信号，下发下降抓取授权')
        self._send(GRAB_CMD_TYPE, 'GrabCmd', '1')

    def _send_color_targets(self, message: PointCloud):
        """Send base-frame target coordinates in millimetres.

        PointCloud points are metres in ``gimbal_base``; color_id is carried in
        the channel with the same name. One frame is sent per detected color.
        """
        if message.header.frame_id != self.target_frame:
            self.get_logger().warning(
                f'忽略目标：frame_id={message.header.frame_id!r}，需要 {self.target_frame}',
                throttle_duration_sec=2.0)
            return
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age < -0.05 or age > self.target_max_age:
            self.get_logger().warning('忽略目标：图像时间戳缺失、过期或超前',
                                      throttle_duration_sec=2.0)
            return
        color_ids = None
        for channel in message.channels:
            if channel.name == 'color_id':
                color_ids = channel.values
                break
        if color_ids is None or len(color_ids) != len(message.points):
            self.get_logger().warning('忽略目标：缺少与 points 一一对应的 color_id')
            return
        now = time.monotonic()
        interval = 1.0 / self.target_max_hz
        if self.task_mode:
            if not self.task_code_sent:
                return
            motion_age = (self.get_clock().now().nanoseconds - self.motion_stamp_ns) / 1e9
            ready = [c.values for c in message.channels if c.name == 'motion_ready']
            if (not self.motion_ready or motion_age > self.target_max_age
                    or now - self.motion_receive_time > self.target_max_age
                    or stamp_ns < self.ready_since_ns
                    or len(ready) != 1 or list(ready[0]) != [1.0]):
                return
        for index, point in enumerate(message.points):
            values = (float(color_ids[index]), point.x * 1000.0,
                      point.y * 1000.0, point.z * 1000.0)
            if not all(math.isfinite(value) for value in values):
                self.get_logger().warning('忽略非有限目标坐标')
                continue
            if values[0] not in PICKUP_COLOR_IDS:
                self.get_logger().warning(f'忽略非法颜色编号 {values[0]}')
                continue
            color_id = int(values[0])
            if self.task_mode and not self.task.accepts(color_id, stamp_ns):
                continue
            if stamp_ns <= self._last_target_stamp.get(color_id, 0):
                continue
            if now - self._last_target_send.get(color_id, -math.inf) < interval:
                continue
            payload = f'{color_id},{values[1]:.1f},{values[2]:.1f},{values[3]:.1f}'
            if self._send(self.target_type, self.target_topic, payload):
                self._last_target_send[color_id] = now
                self._last_target_stamp[color_id] = stamp_ns
                if self.task_mode:
                    self.task.sent(stamp_ns)
                    self._publish_task_status()
                    break


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--serial', dest='serial_port', default=None)
    parser.add_argument('--baud', dest='baud', type=int, default=None)
    known, remaining = parser.parse_known_args()

    # 优先读安装目录（share/identify_pack），失败回退源码同目录
    try:
        from ament_index_python.packages import get_package_share_directory
        config_dir = get_package_share_directory('identify_pack')
    except Exception:
        config_dir = os.path.dirname(__file__)
    config_path = os.path.join(config_dir, 'serial_config.json')
    file_cfg = {}
    if os.path.exists(config_path):
        try:
            with open(config_path) as f:
                file_cfg = json.load(f)
        except Exception as e:
            print(f'读取配置失败 {config_path}: {e}', file=sys.stderr)

    serial_port = known.serial_port or file_cfg.get('serial_port', '/dev/ttyUSB0')
    baud = known.baud or file_cfg.get('baud', 115200)
    timeout = file_cfg.get('timeout', 0.1)

    rclpy.init(args=remaining)
    node = SerialBridgeNode(serial_port, baud, timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
