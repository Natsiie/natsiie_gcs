from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from identify_pack.scripts.camera_watchdog import FrameWatchdog


def test_repeated_frames_trigger_even_when_frames_keep_arriving():
    watchdog = FrameWatchdog(3.0, 5.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    for now in range(7):
        watchdog.observe(frame.copy(), float(now))
    assert watchdog.reason(6.0) == 'frozen'
    assert watchdog.repeated_frames == 6


def test_small_pixel_changes_keep_static_scene_alive():
    watchdog = FrameWatchdog(3.0, 5.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    for now in range(12):
        frame[0, 0, 0] = now
        watchdog.observe(frame, float(now))
        assert watchdog.reason(float(now)) is None


def test_changed_frame_restarts_frozen_timeout():
    watchdog = FrameWatchdog(3.0, 5.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    watchdog.observe(frame, 0.0)
    watchdog.observe(frame, 4.0)
    frame[0, 0, 0] = 1
    watchdog.observe(frame, 4.5)
    watchdog.observe(frame, 8.0)
    assert watchdog.reason(8.0) is None
    watchdog.observe(frame, 10.0)
    assert watchdog.reason(10.0) == 'frozen'


def test_missing_frames_trigger_with_freeze_detection_disabled():
    watchdog = FrameWatchdog(3.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    watchdog.observe(frame, 10.0)
    assert watchdog.reason(13.1) == 'missing'
    watchdog.observe(frame, 14.0)
    assert watchdog.reason(14.0) is None


def test_reset_discards_old_frozen_state():
    watchdog = FrameWatchdog(3.0, 5.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    watchdog.observe(frame, 0.0)
    watchdog.observe(frame, 6.0)
    assert watchdog.reason(6.0) == 'frozen'
    watchdog.reset(6.0)
    watchdog.observe(frame, 6.1)
    assert watchdog.reason(6.1) is None
    assert watchdog.repeated_frames == 0
    assert watchdog.reason(9.2) == 'missing'


def test_watchdog_reopens_own_camera_and_logs_device(monkeypatch):
    from identify_pack.vision_identify import camera_node

    monkeypatch.setattr(camera_node.time, 'monotonic', lambda: 6.0)
    tracker = FrameWatchdog(3.0, 5.0)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    tracker.observe(frame, 0.0)
    tracker.observe(frame, 6.0)
    logger = Mock()
    node = SimpleNamespace(qr_stopped=False, _frame_watchdog=tracker,
                           reconnect_timeout=3.0, frozen_frame_timeout=5.0,
                           _publishing_since=None, publish_timeout=5.0,
                           device='/dev/camera_upper', image_topic='/camera/secondary/image_raw',
                           get_logger=lambda: logger, _restart_capture=Mock())
    camera_node.CameraNode.watchdog_cb(node)
    node._restart_capture.assert_called_once_with()
    assert '/dev/camera_upper' in logger.warning.call_args.args[0]


def test_qr_stopped_skips_reconnect():
    from identify_pack.vision_identify.camera_node import CameraNode

    node = SimpleNamespace(qr_stopped=True, _restart_capture=Mock())
    CameraNode.watchdog_cb(node)
    node._restart_capture.assert_not_called()


def test_old_generation_frame_is_not_published():
    import queue
    from identify_pack.vision_identify.camera_node import CameraNode

    frames = queue.Queue()
    frames.put((1, np.zeros((4, 4, 3), dtype=np.uint8)))
    node = SimpleNamespace(_frame_queue=frames, _generation=2, qr_stopped=False,
                           qr_stop_deadline=None, _frame_watchdog=Mock(), pub=Mock())
    CameraNode.timer_cb(node)
    node._frame_watchdog.observe.assert_not_called()
    node.pub.publish.assert_not_called()


def test_blocked_publisher_is_detected_outside_executor(monkeypatch):
    from identify_pack.vision_identify import camera_node

    monkeypatch.setattr(camera_node.time, 'monotonic', lambda: 10.0)
    node = SimpleNamespace(qr_stopped=False, _publishing_since=2.0,
                           publish_timeout=5.0, _fatal_exit=Mock())
    camera_node.CameraNode.watchdog_cb(node)
    node._fatal_exit.assert_called_once()
    assert 'QR' in node._fatal_exit.call_args.args[0]
