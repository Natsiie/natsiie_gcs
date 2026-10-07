#!/usr/bin/env python3
"""
ROS2 节点：用 cv2.VideoCapture 打开主机摄像头，并发布到 image_topic（默认 /camera/image_raw）。

可选 QR 解码（qr_decode=True，主相机用）：连续多帧读到同一内容后发布到 qr_topic，
随后释放摄像头（stop_after_qr）。副相机保持 qr_decode=False，行为不变。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('fourcc', 'MJPG')  # 像素格式：MJPG 压缩，避免 YUYV 高分辨率卡顿
        # QR 解码（主相机）：读到内容后释放摄像头
        self.declare_parameter('qr_decode', False)
        self.declare_parameter('qr_topic', 'QR_Decode')
        self.declare_parameter('qr_min_consistent', 3)
        self.declare_parameter('stop_after_qr', True)

        device = self.get_parameter('device').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        fourcc = self.get_parameter('fourcc').get_parameter_value().string_value
        self.qr_decode = self.get_parameter('qr_decode').get_parameter_value().bool_value
        self.qr_topic = self.get_parameter('qr_topic').get_parameter_value().string_value
        self.qr_min_consistent = self.get_parameter('qr_min_consistent').get_parameter_value().integer_value
        self.stop_after_qr = self.get_parameter('stop_after_qr').get_parameter_value().bool_value

        # 只在需要时才导入 pyzbar，副相机不依赖它
        self.decode_qr = None
        self.qr_pub = None
        if self.qr_decode:
            try:
                from pyzbar.pyzbar import decode
            except Exception as e:
                self.get_logger().error(f'qr_decode 已开启但无法导入 pyzbar：{e}')
                self.qr_decode = False
            else:
                self.decode_qr = decode
                self.qr_pub = self.create_publisher(String, self.qr_topic, 10)
                self.get_logger().info(
                    f'QR 解码已开启 -> {self.qr_topic}；'
                    f'连续 {self.qr_min_consistent} 帧后'
                    f'{"关闭摄像头" if self.stop_after_qr else "持续解码"}')
        self.qr_last = None
        self.qr_count = 0

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # 尝试把 device 当作整数索引再开一次
            try:
                self.cap = cv2.VideoCapture(int(device))
            except Exception:
                pass
        if not self.cap.isOpened():
            self.cap = None
            self.get_logger().error(f'无法打开摄像头: {device}，请检查 camera_device 参数')
        else:
            # 先设像素格式（MJPG 压缩），否则部分相机高分辨率下 YUYV 会读不到帧/卡第一帧
            if fourcc and len(fourcc) == 4:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.get_logger().info(f'Publishing {device} -> {image_topic} @ {fps}fps fourcc={fourcc}')

        self.br = CvBridge()
        self.pub = self.create_publisher(Image, image_topic, 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_cb)

    def timer_cb(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('读取帧失败', throttle_duration_sec=5.0)
            return
        try:
            msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

        if self.decode_qr is not None:
            self.maybe_read_qr(frame)

    def maybe_read_qr(self, frame):
        """连续多帧读到同一 QR 内容则发布；stop_after_qr 时随即释放摄像头。"""
        current = None
        for obj in self.decode_qr(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):
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
            return

        if current == self.qr_last:
            self.qr_count += 1
        else:
            self.qr_last = current
            self.qr_count = 1
        self.get_logger().info(f'Detected QR: {current} (count={self.qr_count})',
                               throttle_duration_sec=1.0)

        if self.qr_count < self.qr_min_consistent:
            return

        out = String()
        out.data = current
        self.qr_pub.publish(out)
        self.get_logger().info(f'Published {self.qr_topic}: {current}')
        self.qr_last = None
        self.qr_count = 0
        if self.stop_after_qr and self.cap is not None:
            self.cap.release()
            self.cap = None
            self.get_logger().info('QR 已读取，摄像头已关闭')

    def destroy_node(self):
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
