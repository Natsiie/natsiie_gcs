#!/usr/bin/env python3
"""ROS2 节点：cv2.VideoCapture 打开摄像头并发布图像。

- 采集放独立进程，独立监控线程回收阻塞的采集进程；ROS 只发布最新帧；
- 看门狗：连续 ``reconnect_timeout_sec`` 秒没收到新帧，重启采集进程；
- 可选 ``frozen_frame_timeout_sec``：像素完全相同持续超时也重连，0 表示关闭；
- 可选 QR 解码（主相机）：连续读到同一内容后发布，``stop_after_qr`` 时停摄像头。
"""
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from identify_pack.scripts.camera_watchdog import FrameWatchdog
from identify_pack.scripts.camera_capture import CaptureProcess


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 25)
        self.declare_parameter('fourcc', 'YUYV')  # 实际格式/帧率由驱动协商，采集进程会打印结果。
        self.declare_parameter('flip_code', 0)  # 0=不翻转, 1=左右镜像, -1=上下+左右(180°)
        self.declare_parameter('reconnect_timeout_sec', 3.0)  # 连续多少秒无新帧触发重连
        self.declare_parameter('frozen_frame_timeout_sec', 0.0)
        self.declare_parameter('publish_timeout_sec', 5.0)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        # QR 解码（主相机）
        self.declare_parameter('qr_decode', False)
        self.declare_parameter('qr_topic', 'QR_Decode')
        self.declare_parameter('qr_min_consistent', 3)
        self.declare_parameter('stop_after_qr', True)
        self.declare_parameter('show', False)  # cv2 调试窗口（需图形环境，无 DISPLAY 时自动关闭）

        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.fourcc = self.get_parameter('fourcc').value
        self.flip_code = int(self.get_parameter('flip_code').value)
        self.reconnect_timeout = float(self.get_parameter('reconnect_timeout_sec').value)
        self.frozen_frame_timeout = float(self.get_parameter('frozen_frame_timeout_sec').value)
        self.publish_timeout = float(self.get_parameter('publish_timeout_sec').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        if (self.width <= 0 or self.height <= 0 or self.fps <= 0
                or self.reconnect_timeout <= 0 or self.frozen_frame_timeout < 0
                or self.publish_timeout <= 0 or (self.fourcc and len(self.fourcc) != 4)
                or self.flip_code not in (-1, 0, 1)):
            raise ValueError('Invalid camera dimensions, fps, timeout or fourcc')
        self._frame_watchdog = FrameWatchdog(self.reconnect_timeout, self.frozen_frame_timeout)
        self.qr_topic = self.get_parameter('qr_topic').value
        self.qr_min_consistent = int(self.get_parameter('qr_min_consistent').value)
        self.stop_after_qr = bool(self.get_parameter('stop_after_qr').value)
        self.show = bool(self.get_parameter('show').value)
        self._window_name = f'{self.get_name()} [{self.device}]'

        # QR 解码（主相机用，副相机不依赖 pyzbar）
        self.decode_qr = None
        self.qr_pub = None
        if bool(self.get_parameter('qr_decode').value):
            try:
                from pyzbar.pyzbar import decode
            except Exception as e:
                self.get_logger().error(f'qr_decode 已开启但无法导入 pyzbar：{e}')
            else:
                self.decode_qr = decode
                self.qr_pub = self.create_publisher(String, self.qr_topic, 10)
                self.get_logger().info(
                    f'QR 解码已开启 -> {self.qr_topic}；连续 {self.qr_min_consistent} 帧后'
                    f'{"关闭摄像头" if self.stop_after_qr else "持续解码"}')
        self.qr_last = None
        self.qr_count = 0
        self.qr_stopped = False

        self.br = CvBridge()
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(Image, self.image_topic, image_qos)

        # Only the monitor thread owns the capture process and watchdog state.
        self._frame_queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._generation = 0
        self._capture = CaptureProcess(self.device, self.width, self.height, self.fps, self.fourcc)
        self._publishing_since = None
        self._published_frames = 0
        self._monitor = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor.start()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_cb)
        self.get_logger().info(
            f'Publishing {self.device} -> {self.image_topic} @ {self.fps}fps '
            f'fourcc={self.fourcc or "default"} '
            f'frozen_frame_timeout={self.frozen_frame_timeout:g}s')

    def _restart_capture(self):
        self._generation += 1
        try:
            self._frame_queue.get_nowait()
        except queue.Empty:
            pass
        if not self._capture.stop():
            self._fatal_exit('采集进程无法被终止，请检查 USB 驱动/设备状态')
            return
        self._capture.start()
        self._frame_watchdog.reset(time.monotonic())
        self.get_logger().info(f'采集进程已启动: {self.device} pid={self._capture.process.pid}')

    def _fatal_exit(self, detail):
        self.get_logger().error(f'{detail}；退出节点，由 launch respawn 重启 {self.device}')
        self._capture.stop()
        os._exit(1)

    def _monitor_loop(self):
        try:
            self._restart_capture()
            last_status = time.monotonic()
            captured = 0
            published = self._published_frames
            while not self._stop.wait(0.02):
                if self.qr_stopped:
                    break
                result = self._capture.latest()
                if result is not None:
                    frame, captured_at = result
                    self._frame_watchdog.observe(frame, captured_at)
                    captured += 1
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._frame_queue.put_nowait((self._generation, frame))
                    except queue.Full:
                        pass
                self.watchdog_cb()
                now = time.monotonic()
                if now - last_status >= 5.0:
                    self.get_logger().info(
                        f'相机状态 {self.device}: capture={captured / (now - last_status):.1f}fps '
                        f'publish={(self._published_frames - published) / (now - last_status):.1f}fps '
                        f'frame_age={now - self._frame_watchdog.last_frame_time:.2f}s '
                        f'repeats={self._frame_watchdog.repeated_frames}')
                    captured = 0
                    published = self._published_frames
                    last_status = now
        except Exception as error:
            self._fatal_exit(f'采集监控异常: {error}')
        finally:
            self._capture.stop()

    def timer_cb(self):
        try:
            generation, frame = self._frame_queue.get_nowait()
        except queue.Empty:
            return
        if self.qr_stopped or generation != self._generation:
            return
        self._publishing_since = time.monotonic()
        try:
            self._publish_frame(frame)
        finally:
            self._publishing_since = None

    def _publish_frame(self, frame):
        if self.flip_code:
            frame = cv2.flip(frame, self.flip_code)
        try:
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        self.pub.publish(msg)
        self._published_frames += 1

        if self.decode_qr is not None:
            qr_objs = self.maybe_read_qr(frame)
        else:
            qr_objs = ()
        if self.show:
            self._show_frame(frame, qr_objs)

    def watchdog_cb(self):
        """Run outside the ROS executor so publish/QR stalls cannot disable recovery."""
        if self.qr_stopped:
            return
        now = time.monotonic()
        publishing_since = self._publishing_since
        if publishing_since is not None and now - publishing_since > self.publish_timeout:
            self._fatal_exit(f'图像发布/QR 回调阻塞超过 {self.publish_timeout:g}s')
            return
        reason = self._frame_watchdog.reason(now)
        if reason:
            detail = (f'读帧超时 {self.reconnect_timeout:g}s' if reason == 'missing' else
                      f'连续 {self.frozen_frame_timeout:g}s 像素完全相同 '
                      f'({self._frame_watchdog.repeated_frames} 帧重复)')
            self.get_logger().warning(
                f'{detail}，重启采集进程 {self.device} -> {self.image_topic}')
            self._restart_capture()

    def maybe_read_qr(self, frame):
        """连续多帧读到同一 QR 内容则发布；stop_after_qr 时随即停摄像头。
        返回本帧解码到的 QR 对象列表，供调试窗口画框。"""
        objs = list(self.decode_qr(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))
        current = None
        for obj in objs:
            try:
                text = obj.data.decode('utf-8').strip()
            except Exception:
                text = None
            if text:
                current = text
                break

        if not current:
            self.qr_last = None
            self.qr_count = 0
            return objs

        if current == self.qr_last:
            self.qr_count += 1
        else:
            self.qr_last = current
            self.qr_count = 1
        self.get_logger().info(f'Detected QR: {current} (count={self.qr_count})',
                               throttle_duration_sec=1.0)

        if self.qr_count < self.qr_min_consistent:
            return objs

        out = String()
        out.data = current
        self.qr_pub.publish(out)
        self.get_logger().info(f'Published {self.qr_topic}: {current}')
        self.qr_last = None
        self.qr_count = 0
        if self.stop_after_qr:
            self.qr_stopped = True
            self.get_logger().info('QR 已读取，停止采集进程')
        return objs

    def _show_frame(self, frame, qr_objs):
        """cv2 调试窗口：叠加 QR 框和内容；无显示环境时自动关闭。"""
        for obj in qr_objs:
            try:
                text = obj.data.decode('utf-8').strip()
            except Exception:
                text = None
            if not text:
                continue
            x, y, w, h = obj.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, text, (x, max(y - 6, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        try:
            cv2.imshow(self._window_name, frame)
            cv2.waitKey(1)
        except cv2.error:
            self.show = False
            self.get_logger().warning(
                '无法打开调试窗口（无图形环境/DISPLAY），已自动关闭 show')

    def destroy_node(self):
        self._stop.set()
        self._monitor.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
