#!/usr/bin/env python3
"""Full-frame multi-color tracking on independent ROS topics; no pickup control."""

import copy
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import ChannelFloat32, Image, PointCloud
from std_msgs.msg import String

from identify_pack.scripts.color_config import COLOR_DISPLAY, COLOR_NAMES, load_color_config
from identify_pack.scripts.color_tracking import ColorMultiTracker, detect_color_objects


def make_centers_message(header, tracks, image_size):
    """Only confirmed, currently observed tracks; never emit a lost prediction."""
    message = PointCloud()
    message.header = copy.deepcopy(header)
    channels = {name: [] for name in (
        'track_id', 'color_id', 'area_px2', 'image_width', 'image_height',
        'velocity_x_px_s', 'velocity_y_px_s', 'color_consistency',
        'detection_score', 'observed', 'stationary')}
    for track in tracks:
        if not track['confirmed'] or not track['observed']:
            continue
        x, y = track['center_px']
        if not (0 <= x < image_size[0] and 0 <= y < image_size[1]):
            continue
        message.points.append(Point32(x=float(x), y=float(y), z=0.))
        values = (track['track_id'], track['color_id'], track['area_px2'], *image_size,
                  *track['velocity_px_s'], track['color_consistency'],
                  track['detection_score'], 1., float(track['stationary']))
        for name, value in zip(channels, values):
            channels[name].append(float(value))
    message.channels = [ChannelFloat32(name=name, values=values)
                        for name, values in channels.items()]
    return message


def draw_tracks(frame, tracks, status):
    image = frame.copy()
    for track in tracks:
        color = COLOR_DISPLAY[COLOR_NAMES[track['color_id']]]
        if not track['observed']:
            color = (160, 160, 160)
        x, y, width, height = (int(round(value)) for value in track['bbox_px'])
        cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
        point = tuple(int(round(value)) for value in track['center_px'])
        cv2.drawMarker(image, point, color, cv2.MARKER_CROSS, 12, 1)
        history = np.asarray(track['history_px'], dtype=np.int32)
        if len(history) >= 2:
            cv2.polylines(image, [history], False, color, 1)
        speed = np.linalg.norm(track['velocity_px_s'])
        text = f"ID{track['track_id']} {COLOR_NAMES[track['color_id']]} {track['state']} {speed:.0f}px/s"
        cv2.putText(image, text, (max(0, x), max(18, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    cv2.putText(image, status, (8, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1)
    return image


class TrackingColorNode(Node):
    WINDOW = 'tracking_color'

    def __init__(self):
        super().__init__('tracking_color_node')
        defaults = {
            'image_topic': '/camera/secondary/image_raw', 'color_config': '',
            'tracks_topic': '/tracking_color/tracks',
            'centers_topic': '/tracking_color/centers_pixel',
            'debug_image_topic': '/tracking_color/debug_image',
            'show': False, 'publish_debug_image': True,
            'min_blob_area': 300., 'max_blob_area_fraction': 0.15,
            'min_blob_dimension': 12, 'max_aspect_ratio': 3.,
            'min_solidity': 0.65, 'min_extent': 0.3,
            'open_kernel_px': 3, 'close_kernel_px': 5, 'duplicate_overlap': 0.7,
            'min_confirmed_hits': 3, 'max_missed_sec': 0.5,
            'max_frame_gap_sec': 1., 'max_match_distance_px': 80., 'max_area_ratio': 3.,
            'measurement_std_px': 3., 'acceleration_std_px_s2': 80.,
            'color_window': 7, 'history_length': 30,
            'stationary_speed_px_s': 8., 'stationary_frames': 5,
            'image_timeout_sec': 0.75, 'max_image_age_sec': 1.,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.options = {name: self.get_parameter(name).value for name in defaults}
        tracker_names = (
            'min_confirmed_hits', 'max_missed_sec', 'max_frame_gap_sec',
            'max_match_distance_px', 'max_area_ratio', 'measurement_std_px',
            'acceleration_std_px_s2', 'color_window', 'history_length',
            'stationary_speed_px_s', 'stationary_frames')
        self.tracker = ColorMultiTracker(**{name: self.options[name] for name in tracker_names})
        self.detection_options = {
            'min_area': self.options['min_blob_area'],
            'max_area_fraction': self.options['max_blob_area_fraction'],
            'min_dimension': self.options['min_blob_dimension'],
            **{name: self.options[name] for name in (
                'max_aspect_ratio', 'min_solidity', 'min_extent', 'open_kernel_px',
                'close_kernel_px', 'duplicate_overlap')},
        }
        for name in ('image_timeout_sec', 'max_image_age_sec'):
            if not np.isfinite(self.options[name]) or self.options[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        path = self.options['color_config']
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = str(Path(get_package_share_directory('identify_pack')) / 'color_config.json')
        self.color_config = load_color_config(path)
        # Validate detector options before subscribing or opening the GUI.
        detect_color_objects(np.full((32, 32, 3), 255, np.uint8), self.color_config,
                             **self.detection_options)
        self.show = self.options['show']
        if self.show:
            from identify_pack.scripts.camera_diagnostics import require_display
            require_display()
            cv2.namedWindow(self.WINDOW, cv2.WINDOW_AUTOSIZE)
        self.bridge = CvBridge()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.track_pub = self.create_publisher(String, self.options['tracks_topic'], 1)
        self.center_pub = self.create_publisher(PointCloud, self.options['centers_topic'], qos)
        self.debug_pub = self.create_publisher(Image, self.options['debug_image_topic'], qos)
        self.subscription = self.create_subscription(Image, self.options['image_topic'], self.image_cb, qos)
        self.last_received = None
        self.last_header = None
        self.last_stamp_ns = None
        self.last_size = None
        self.last_frame = None
        self.timeout_reported = False
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.get_logger().info(f'全图颜色跟踪订阅 {self.options["image_topic"]}；HSV 配置 {path}')

    def publish_results(self, header, tracks, image_size, status, frame=None):
        report = {
            'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
            'frame_id': header.frame_id, 'image_size': list(image_size),
            'status': status, 'tracks': tracks,
            'observed_confirmed_count': sum(track['confirmed'] and track['observed'] for track in tracks),
        }
        self.track_pub.publish(String(data=json.dumps(report, ensure_ascii=False, allow_nan=False)))
        self.center_pub.publish(make_centers_message(header, tracks, image_size))
        if frame is not None and (self.show or self.options['publish_debug_image']):
            preview = draw_tracks(frame, tracks, status)
            if self.options['publish_debug_image']:
                debug = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
                debug.header = copy.deepcopy(header)
                self.debug_pub.publish(debug)
            if self.show:
                cv2.imshow(self.WINDOW, preview)
                cv2.waitKey(1)

    def image_cb(self, message):
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or not -0.05 <= age <= self.options['max_image_age_sec']:
            self.get_logger().warning('忽略缺少时间戳或过期的图像', throttle_duration_sec=2.)
            return
        # Dropped/out-of-order packets cannot count as fresh observations or hits.
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            detections = detect_color_objects(frame, self.color_config, **self.detection_options)
            if self.last_header is not None and message.header.frame_id != self.last_header.frame_id:
                self.tracker.reset()
            tracks = self.tracker.update(detections, stamp_ns / 1e9, frame.shape[1::-1])
        except (ValueError, cv2.error, CvBridgeError) as error:
            self.get_logger().error(f'颜色跟踪帧处理失败: {error}', throttle_duration_sec=2.)
            return
        self.last_received = time.monotonic()
        self.last_stamp_ns = stamp_ns
        self.last_header = copy.deepcopy(message.header)
        self.last_size = frame.shape[1::-1]
        self.last_frame = frame
        self.timeout_reported = False
        self.publish_results(message.header, tracks, self.last_size, 'tracking', frame)
        self.get_logger().info(
            f'候选={len(detections)} 跟踪={len(tracks)} '
            f'当前确认={sum(track["confirmed"] and track["observed"] for track in tracks)}',
            throttle_duration_sec=1.)

    def check_timeout(self):
        if (self.last_received is None or self.timeout_reported
                or time.monotonic() - self.last_received <= self.options['image_timeout_sec']):
            return
        self.tracker.reset()
        self.last_stamp_ns = None
        self.timeout_reported = True
        # Keep the last source timestamp: never make old pixels look like new data.
        self.publish_results(self.last_header, [], self.last_size, 'image_timeout', self.last_frame)
        self.get_logger().warning('相机图像超时，清空目标并重建跟踪')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingColorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.show:
                cv2.destroyWindow(node.WINDOW)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
