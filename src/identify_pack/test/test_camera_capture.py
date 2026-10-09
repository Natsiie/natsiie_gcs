import time

import numpy as np

from identify_pack.scripts.camera_capture import CaptureProcess


def hanging_capture(device, width, height, fps, fourcc, pixels, sequence, shape, stamp):
    stamp.value = time.monotonic()
    while True:
        time.sleep(10)


def frame_then_hang(device, width, height, fps, fourcc, pixels, sequence, shape, stamp):
    sequence.value = 1
    np.frombuffer(pixels, dtype=np.uint8)[:] = 7
    shape[0], shape[1] = height, width
    stamp.value = time.monotonic()
    sequence.value = 2
    hanging_capture(device, width, height, fps, fourcc, pixels, sequence, shape, stamp)


def test_blocked_capture_can_be_killed_and_restarted():
    capture = CaptureProcess('fake', 4, 4, 25, 'YUYV', worker=hanging_capture)
    try:
        capture.start()
        old_pid = capture.process.pid
        deadline = time.monotonic() + 5.0
        while capture.stamp.value == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert capture.stamp.value > 0
        assert capture.latest() is None
        started = time.monotonic()
        assert capture.stop()
        assert time.monotonic() - started < 2.0
        capture.start()
        assert capture.process.pid != old_pid
        assert capture.latest() is None
    finally:
        capture.stop()


def test_latest_frame_is_copied_once_and_restart_discards_old_frame():
    capture = CaptureProcess('fake', 4, 4, 25, 'YUYV', worker=frame_then_hang)
    try:
        capture.start()
        deadline = time.monotonic() + 5.0
        result = None
        while result is None and time.monotonic() < deadline:
            result = capture.latest()
            time.sleep(0.01)
        assert result is not None
        frame, stamp = result
        assert frame.shape == (4, 4, 3)
        assert np.all(frame == 7)
        assert stamp > 0
        assert capture.latest() is None
        capture.pixels[0] = 9
        assert frame[0, 0, 0] == 7
        assert capture.stop()
        capture.worker = hanging_capture
        capture.start()
        assert capture.latest() is None
    finally:
        capture.stop()


def test_partial_shared_frame_is_never_read():
    capture = CaptureProcess('fake', 4, 4, 25, 'YUYV', worker=hanging_capture)
    try:
        capture.start()
        capture.sequence.value = 1
        assert capture.latest() is None
    finally:
        capture.stop()


def test_monitor_recovers_missing_frames_without_ros_executor():
    import queue
    import threading
    from types import MethodType, SimpleNamespace
    from unittest.mock import Mock
    from identify_pack.scripts.camera_watchdog import FrameWatchdog
    from identify_pack.vision_identify.camera_node import CameraNode

    capture = CaptureProcess('fake', 4, 4, 25, 'YUYV', worker=hanging_capture)
    node = SimpleNamespace(_capture=capture, _generation=0, _frame_queue=queue.Queue(1),
                           _frame_watchdog=FrameWatchdog(0.1), _stop=threading.Event(),
                           qr_stopped=False, _published_frames=0, _publishing_since=None,
                           publish_timeout=5.0, reconnect_timeout=0.1, frozen_frame_timeout=0,
                           device='fake', image_topic='/fake', get_logger=lambda: Mock(),
                           _fatal_exit=Mock())
    node._restart_capture = MethodType(CameraNode._restart_capture, node)
    node.watchdog_cb = MethodType(CameraNode.watchdog_cb, node)
    monitor = threading.Thread(target=CameraNode._monitor_loop, args=(node,))
    monitor.start()
    try:
        deadline = time.monotonic() + 5.0
        while node._generation < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert node._generation >= 2
        node._fatal_exit.assert_not_called()
    finally:
        node._stop.set()
        monitor.join(timeout=3.0)
        assert not monitor.is_alive()
        assert capture.process is None
