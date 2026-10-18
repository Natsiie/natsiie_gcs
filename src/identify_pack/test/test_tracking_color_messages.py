"""Check ROS output without opening DDS, cameras or a robot control connection."""

import json
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from std_msgs.msg import Header
from identify_pack.scripts.color_tracking import ColorMultiTracker
from identify_pack.scripts.color_config import build_default_color_config
from identify_pack.scripts.material_top_detector import MaterialTopConfig
from identify_pack.vision_identify.tracking_color_node import (
    TrackingColorNode, draw_tracks, make_centers_message,
)
from test_color_tracking import detection


class TrackingMessageTests(unittest.TestCase):
    def test_only_current_confirmed_measurements_are_published(self):
        header = Header(frame_id='camera_optical_frame')
        header.stamp.sec, header.stamp.nanosec = 10, 100000000
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        tracker.update([detection(100), detection(300, color=2)], 10., (640, 480))
        tracks = tracker.update([detection(110), detection(310, color=2)], 10.1, (640, 480))
        message = make_centers_message(header, tracks, (640, 480))
        self.assertEqual(message.header, header)
        self.assertEqual(len(message.points), 2)
        self.assertTrue(all(len(channel.values) == 2 for channel in message.channels))
        channels = {channel.name: list(channel.values) for channel in message.channels}
        self.assertEqual(channels['color_id'], [1., 2.])
        self.assertEqual(channels['observed'], [1., 1.])
        self.assertNotIn('motion_ready', channels)
        # One lost track and one tentative detection must not appear in the cloud.
        tracks = tracker.update([detection(320, color=2), detection(500, color=3)],
                                10.2, (640, 480))
        message = make_centers_message(header, tracks, (640, 480))
        self.assertEqual(len(message.points), 1)
        channels = {channel.name: list(channel.values) for channel in message.channels}
        self.assertEqual(channels['color_id'], [2.])

    def test_no_detections_clears_points_and_per_point_metadata(self):
        message = make_centers_message(Header(), [], (640, 480))
        self.assertEqual(message.points, [])
        self.assertTrue(all(list(channel.values) == [] for channel in message.channels))

    def fake_node(self):
        image = np.full((480, 640, 3), 245, np.uint8)
        for x, color in zip((100, 300, 500), ((0, 0, 220), (0, 220, 220), (120, 0, 0))):
            cv2.circle(image, (x, 200), 30, color, -1)
        logger = Mock()
        return SimpleNamespace(
            options={'max_image_age_sec': 1., 'image_timeout_sec': 0.75, 'detector_mode': 'material_top'},
            top_config=MaterialTopConfig(), shape_diagnostics=[],
            tracker=ColorMultiTracker(), color_config=build_default_color_config(),
            detection_options={}, bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a, **kw: image),
            get_logger=lambda: logger,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_200_000_000)),
            last_received=None, last_header=None, last_stamp_ns=None, last_size=None,
            last_frame=None, timeout_reported=False, publish_results=Mock(),
        )

    def test_image_callback_tracks_without_a_stop_gate_and_rejects_old_packets(self):
        node = self.fake_node()
        message = SimpleNamespace(header=Header(frame_id='camera_optical_frame'))
        for stamp in (10_000_000_000, 10_100_000_000, 10_200_000_000):
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            TrackingColorNode.image_cb(node, message)
        tracks = node.publish_results.call_args[0][1]
        self.assertEqual(len(tracks), 3)
        self.assertTrue(all(track['confirmed'] and track['observed'] for track in tracks))
        json.dumps(tracks, allow_nan=False)
        image = draw_tracks(node.last_frame, tracks, 'tracking')
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertEqual(node.last_frame[0, 0].tolist(), [245, 245, 245])
        before = node.tracker.snapshot()
        previous_receive = node.last_received
        for stamp in (10_200_000_000, 10_100_000_000, 9_000_000_000):
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            TrackingColorNode.image_cb(node, message)
        self.assertEqual(node.publish_results.call_count, 3)
        self.assertEqual(node.tracker.snapshot(), before)
        self.assertEqual(node.last_received, previous_receive)

    def test_timeout_clears_and_publishes_empty_tracks_once_with_source_stamp(self):
        node = self.fake_node()
        message = SimpleNamespace(header=Header(frame_id='camera_optical_frame'))
        message.header.stamp.sec = 10
        TrackingColorNode.image_cb(node, message)
        node.last_received = time.monotonic() - 2
        TrackingColorNode.check_timeout(node)
        self.assertEqual(node.tracker.snapshot(), [])
        header, tracks, size, status, _ = node.publish_results.call_args[0]
        self.assertEqual(header.stamp.sec, 10)
        self.assertEqual(tracks, [])
        self.assertEqual(status, 'image_timeout')
        self.assertEqual(size, (640, 480))
        calls = node.publish_results.call_count
        TrackingColorNode.check_timeout(node)
        self.assertEqual(node.publish_results.call_count, calls)
        message.header.stamp.nanosec = 100000000
        TrackingColorNode.image_cb(node, message)
        self.assertFalse(node.timeout_reported)
        self.assertGreater(node.tracker.snapshot()[0]['track_id'], 3)

    def test_hsv_mode_keeps_blob_centers_and_has_no_top_metadata(self):
        node = self.fake_node()
        node.options['detector_mode'] = 'hsv_blob'
        message = SimpleNamespace(header=Header(frame_id='camera_optical_frame'))
        message.header.stamp.sec = 10
        TrackingColorNode.image_cb(node, message)
        tracks = node.publish_results.call_args[0][1]
        self.assertEqual(len(tracks), 3)
        self.assertTrue(all(track['detector'] == 'hsv_blob' for track in tracks))
        self.assertTrue(all(track['top_ellipse_px'] is None for track in tracks))
        self.assertEqual(node.shape_diagnostics, [])

    def test_shape_diagnostics_and_ellipses_serialize_in_ros_report(self):
        node = self.fake_node()
        message = SimpleNamespace(header=Header(frame_id='camera_optical_frame'))
        message.header.stamp.sec = 10
        TrackingColorNode.image_cb(node, message)
        header, tracks, size, status, frame = node.publish_results.call_args[0]
        node.options.update(publish_debug_image=False, show_rejected_tops=True)
        node.show = False
        node.track_pub, node.center_pub = Mock(), Mock()
        TrackingColorNode.publish_results(node, header, tracks, size, status, frame)
        report = json.loads(node.track_pub.publish.call_args[0][0].data)
        self.assertEqual(report['detector_mode'], 'material_top')
        self.assertTrue(report['top_candidates'])
        self.assertTrue(all(track['top_ellipse_px'] for track in report['tracks']))
        self.assertEqual(report['observed_confirmed_count'], 0)
        preview = draw_tracks(frame, tracks, status, node.shape_diagnostics)
        self.assertEqual(preview.shape, frame.shape)


if __name__ == '__main__':
    unittest.main()
