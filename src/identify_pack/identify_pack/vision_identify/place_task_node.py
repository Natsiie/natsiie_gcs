"""Placement coordinator: numbered ring observations to confirmed commands."""

import copy
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from identify_pack.config_paths import data_path, state_path
from identify_pack.scripts.camera_geometry import pixel_to_plane
from identify_pack.scripts.ground_homography import apply_homography, load as load_ground_homography
from identify_pack.scripts.place_task import DEFAULT_PHASES, PlaceTask, STAGES
from identify_pack.tf_transform.pixel_to_base_node import transform_matrix


def _select_ring_point(message, ring, image_size, camera_frame, release_z_mm,
                       require_aligned=True, require_printed_ids=False):
    channels = {channel.name: list(channel.values) for channel in message.channels}
    if len(channels) != len(message.channels):
        raise ValueError('Duplicate ring metadata')
    if (message.header.frame_id != camera_frame or len(message.points) != 3
            or channels.get('ring_id') != [1.0, 2.0, 3.0]
            or channels.get('stable') != [1.0]
            or (require_aligned and channels.get('within_tolerance') != [1.0])):
        raise ValueError('Three stable, numbered rings in the camera frame are required')
    if require_printed_ids and (channels.get('digit_mapping_valid') != [1.0]
                                or channels.get('digit_mapping_stable') != [1.0]):
        raise ValueError('A stable, verified mapping of ring IDs is required')
    if (channels.get('image_width') != [float(image_size[0])] * 3
            or channels.get('image_height') != [float(image_size[1])] * 3):
        raise ValueError('Ring resolution differs from camera calibration')
    if type(ring) is not int or ring not in (1, 2, 3):
        raise ValueError('Invalid ring ID')
    for point in message.points:
        if (not all(math.isfinite(v) for v in (point.x, point.y, point.z))
                or not 0 <= point.x < image_size[0] or not 0 <= point.y < image_size[1]):
            raise ValueError('Ring center outside image bounds')
    if not math.isfinite(release_z_mm):
        raise ValueError('Invalid release height')
    return message.points[ring - 1]


def placement_target(message, ring, image_size, camera_frame, matrix, distortion,
                     base_from_camera, plane_z_mm, release_z_mm, require_aligned=True,
                     require_printed_ids=False):
    point = _select_ring_point(message, ring, image_size, camera_frame, release_z_mm,
                               require_aligned, require_printed_ids)
    # Project the visible ring plane, then set the independently calibrated gripper Z.
    xyz = pixel_to_plane((point.x, point.y), matrix, distortion,
                         base_from_camera, plane_z_mm / 1000.0) * 1000.0
    xyz[2] = release_z_mm
    return tuple(float(v) for v in xyz)


def placement_target_homography(message, ring, image_size, camera_frame, homography,
                                base_from_link, release_z_mm, require_aligned=True,
                                require_printed_ids=False):
    point = _select_ring_point(message, ring, image_size, camera_frame, release_z_mm,
                               require_aligned, require_printed_ids)
    link = apply_homography((point.x, point.y), homography)
    base = np.asarray(base_from_link, dtype=float) @ np.asarray(link, dtype=float)
    return (float(base[0]), float(base[1]), float(release_z_mm))


class PlaceTaskNode(Node):
    def __init__(self):
        super().__init__('place_task')
        defaults = {
            'enabled': False,
            'calibration_file': 'camera_extrinsics_v2.json',
            'checkpoint_file': '',
            'base_frame': 'gimbal_base', 'camera_frame': 'camera_optical_frame',
            'gimbal_frame': 'gimbal_link',
            'mapping_mode': 'pinhole', 'homography_file': '',
            'confirmation_timeout_sec': 30.0, 'target_max_age_sec': 0.5,
            'require_aligned': True, 'item_height_configured': True,
            'item_height_mm': 60.0,
            'phase_order': list(DEFAULT_PHASES),
        }
        for stage in STAGES:
            defaults.update({f'{stage}.height_configured': False,
                             f'{stage}.plane_z_mm': 0.0,
                             f'{stage}.first_release_z_mm': 0.0,
                             f'{stage}.centers_topic': f'/{stage}/centers_pixel'})
        defaults['stacked_ring.plane_z_mm'] = -116.0
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.enabled = self.param('enabled')
        self.max_age = self.param('target_max_age_sec')
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('target_max_age_sec must be finite and positive')
        self.path = state_path(self.param('checkpoint_file'), 'place_progress.json')
        timeout = self.param('confirmation_timeout_sec')
        phases = self.param('phase_order')
        self.task = (PlaceTask.restore(self.path, timeout, phases)
                     if self.path.exists() else PlaceTask(timeout, phases))
        self.mapping_mode = self.param('mapping_mode')
        if self.mapping_mode not in ('pinhole', 'homography'):
            raise ValueError("mapping_mode must be 'pinhole' or 'homography'")
        self.matrix = self.distortion = self.image_size = None
        self.homography = None
        if self.enabled:
            if self.mapping_mode == 'homography':
                self.load_homography()
            else:
                self.load_calibration()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observations = {}
        self.frozen_observation = None
        self.phase_started_ns = self.get_clock().now().nanoseconds
        self.command_pub = self.create_publisher(String, '/arm/place_command', 10)
        self.status_pub = self.create_publisher(
            String, '/arm/place_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, 'QR_Decode', self.on_task, 10)
        self.create_subscription(
            String, '/arm/task_status', self.on_pick_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/arm/place_control', self.on_control, 10)
        self.create_subscription(String, '/arm/place_result', self.on_result, 10)
        for stage in STAGES:
            self.create_subscription(
                PointCloud, self.param(f'{stage}.centers_topic'),
                lambda message, name=stage: self.on_centers(name, message), 10)
        self.create_timer(0.1, self.tick)
        self.publish_status()

    def param(self, name):
        return self.get_parameter(name).value

    def load_calibration(self):
        calibration = json.loads(
            data_path(self.param('calibration_file')).read_text(encoding='utf-8'))
        if calibration['child_frame'] != self.param('camera_frame'):
            raise ValueError('Calibration camera frame does not match')
        if (not self.param('base_frame') or not self.param('camera_frame')
                or self.param('base_frame') == self.param('camera_frame')):
            raise ValueError('Distinct base and camera frames are required')
        self.matrix = np.asarray(calibration['camera_matrix'], dtype=float)
        self.distortion = np.asarray(calibration['dist_coeffs'], dtype=float).reshape(-1)
        self.image_size = tuple(calibration['image_size'])
        if (self.matrix.shape != (3, 3) or not np.isfinite(self.matrix).all()
                or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0
                or not np.allclose(self.matrix[2], [0, 0, 1])
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.distortion).all()
                or len(self.image_size) != 2 or min(self.image_size) <= 0):
            raise ValueError('Invalid camera calibration')

    def load_homography(self):
        mapping = load_ground_homography(data_path(self.param('homography_file')))
        if mapping['base_frame'] != self.param('base_frame'):
            raise ValueError('Homography base frame does not match')
        self.homography = mapping['homography']
        self.image_size = mapping['image_size']

    def commit(self, operation):
        previous = copy.deepcopy(self.task)
        try:
            result = operation()
            if self.task.code:
                self.task.save(self.path)
        except Exception:
            self.task = previous
            raise
        self.publish_status()
        return result

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            **self.task.status(), 'enabled': self.enabled}, allow_nan=False)))

    def reject(self, error):
        self.get_logger().warning(f'Placement rejected: {error}', throttle_duration_sec=2.0)

    def on_task(self, message):
        try:
            self.commit(lambda: self.task.load(message.data.strip()))
        except (ValueError, OSError) as error:
            self.reject(error)

    def on_pick_status(self, message):
        try:
            code = json.loads(message.data).get('task_code')
            if code:
                self.on_task(String(data=code))
        except (ValueError, TypeError, AttributeError) as error:
            self.reject(error)

    def validate_heights(self, stage):
        if stage not in STAGES or not self.param(f'{stage}.height_configured'):
            raise ValueError('Stage plane and release heights are not configured')
        if not all(math.isfinite(self.param(f'{stage}.{name}')) for name in
                   ('plane_z_mm', 'first_release_z_mm')):
            raise ValueError('Stage heights must be finite')
        if stage == 'stacked_ring' and (
                not self.param('item_height_configured')
                or not math.isfinite(self.param('item_height_mm'))
                or self.param('item_height_mm') <= 0):
            raise ValueError('Stacking item height is not configured')

    def on_control(self, message):
        try:
            data = json.loads(message.data)
            if not self.enabled:
                raise ValueError('Placement is disabled; configure heights and enable it at startup')
            # 下位机经串口下发的控制帧不带 task_id，用当前已加载任务的 ID 兜底；
            # 手动 pub 时仍可显式携带 task_id 精确指定。
            task_id = data.get('task_id') or self.task.task_id
            if data['action'] == 'start':
                self.validate_heights(data['stage'])
                self.commit(lambda: self.task.start(task_id, data['stage'], data['batch']))
                self.phase_started_ns = self.get_clock().now().nanoseconds
                self.frozen_observation = self._freeze_observation(data['stage'])
            elif data['action'] == 'retry':
                command = self.commit(lambda: self.task.retry(
                    task_id, data['command_id'], time.monotonic()))
                self.command_pub.publish(String(data=json.dumps(command, allow_nan=False)))
            else:
                raise ValueError('Unknown placement control action')
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.reject(error)

    def _freeze_observation(self, stage):
        """冻结下位机停到位（收到 start）瞬间的圆环圆心观测。

        车停稳后圆心稳定且对齐；冻结后整个阶段都用这份观测计算放置坐标，
        后续车/云台运动导致的圆心漂移不再被采用。仅当 start 到达时已有足够
        新、稳定且对齐的观测才冻结，否则返回 None，由 tick 退回实时观测兜底。
        """
        observation = self.observations.get(stage)
        if observation is None:
            self.get_logger().warning(
                f'{stage} 收到 start 时暂无圆心观测，退回实时观测兜底')
            return None
        message, received = observation
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (self.phase_started_ns - stamp_ns) / 1e9
        if age < 0 or age > self.max_age or time.monotonic() - received > self.max_age:
            self.get_logger().warning(
                f'{stage} 停稳圆心观测已过期（{age:.2f}s），拒绝冻结，退回实时观测')
            return None
        channels = {channel.name: list(channel.values) for channel in message.channels}
        if (channels.get('stable') != [1.0] or channels.get('within_tolerance') != [1.0]):
            self.get_logger().warning(
                f'{stage} 停稳圆心观测不稳定或未对齐，拒绝冻结，退回实时观测')
            return None
        self.get_logger().info(
            f'{stage} 冻结停稳圆心观测（start 前 {age:.3f}s），后续运动不再影响放置坐标')
        return observation

    def on_result(self, message):
        try:
            data = json.loads(message.data)
            changed = self.commit(lambda: self.task.finish(
                data['task_id'], data['command_id'], data['success']))
            if changed:
                self.phase_started_ns = self.get_clock().now().nanoseconds
                self.observations.clear()
            else:
                raise ValueError('Result does not match the pending placement command')
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.reject(error)

    def on_centers(self, stage, message):
        self.observations[stage] = (message, time.monotonic())

    def tick(self):
        try:
            if self.task.check_timeout(time.monotonic()):
                self.task.save(self.path)
                self.publish_status()
            if not self.enabled or self.task.state != 'WAIT_TARGET':
                return
            stage = self.task.stage
            if self.frozen_observation is not None:
                message, _ = self.frozen_observation
            else:
                if stage not in self.observations:
                    return
                message, received = self.observations[stage]
                now_ns = self.get_clock().now().nanoseconds
                stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                age = (now_ns - stamp_ns) / 1e9
                if (stamp_ns <= self.phase_started_ns or age < 0 or age > self.max_age
                        or time.monotonic() - received > self.max_age):
                    return
            self.validate_heights(stage)
            stamp = Time.from_msg(message.header.stamp, clock_type=self.get_clock().clock_type)
            ring = self.task.expected.ring
            count = self.task.stack_count[ring - 1] if stage == 'stacked_ring' else 0
            release_z = self.param(f'{stage}.first_release_z_mm') + count * self.param('item_height_mm')
            if self.mapping_mode == 'homography':
                transform = self.tf_buffer.lookup_transform(
                    self.param('base_frame'), self.param('gimbal_frame'), stamp)
                base_from_link = transform_matrix(transform)[:2, :2]
                xyz = placement_target_homography(
                    message, ring, self.image_size, self.param('camera_frame'),
                    self.homography, base_from_link, release_z,
                    self.param('require_aligned'),
                    require_printed_ids=stage == 'ring_station')
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.param('base_frame'), self.param('camera_frame'), stamp)
                xyz = placement_target(
                    message, ring, self.image_size, self.param('camera_frame'), self.matrix,
                    self.distortion, transform_matrix(transform),
                    self.param(f'{stage}.plane_z_mm'), release_z, self.param('require_aligned'),
                    require_printed_ids=stage == 'ring_station')
            command = self.commit(lambda: self.task.prepare(
                xyz, count + 1, self.param('base_frame'), time.monotonic()))
            self.command_pub.publish(String(data=json.dumps(command, allow_nan=False)))
        except (ValueError, OSError, TransformException, cv2.error) as error:
            self.reject(error)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlaceTaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
