import json
from pathlib import Path
import time

import cv2
import numpy as np

from identify_pack.scripts import camera_capture
from identify_pack.scripts.tools import camera_diagnostics


def hanging_worker(*args):
    time.sleep(30)


def frame_worker(device, width, height, fps, fourcc, pixels, sequence, shape, stamp):
    sequence.value = 1
    np.frombuffer(pixels, dtype=np.uint8)[:] = 9
    shape[0], shape[1] = height, width
    stamp.value = time.monotonic()
    sequence.value = 2
    time.sleep(30)


def test_camera_read_deadline_and_release(monkeypatch):
    factory = camera_capture.CaptureProcess
    monkeypatch.setattr(camera_capture, 'CaptureProcess',
                        lambda *args: factory(*args, worker=hanging_worker))
    camera = camera_capture.TimedCamera('0', 4, 4, 25, timeout=0.1)
    try:
        started = time.monotonic()
        assert camera.read() == (False, None)
        assert time.monotonic() - started < 1
        assert camera.capture.process.is_alive()
        assert camera.capture.config[0] == 0
    finally:
        camera.release()
    assert camera.capture.process is None


def test_camera_only_returns_fresh_frames(monkeypatch):
    factory = camera_capture.CaptureProcess
    monkeypatch.setattr(camera_capture, 'CaptureProcess',
                        lambda *args: factory(*args, worker=frame_worker))
    camera = camera_capture.TimedCamera('fake', 4, 4, 25, timeout=3)
    try:
        ok, frame = camera.read()
        assert ok and np.all(frame == 9)
        camera.timeout = 0.1
        assert camera.read() == (False, None)
    finally:
        camera.release()


def test_offline_ring_report_and_lost_target(tmp_path):
    image = np.full((480, 640, 3), 240, np.uint8)
    for x in (199, 328, 458):
        for radius in (18, 28, 40):
            cv2.circle(image, (x, 227), radius, (20, 20, 20), 2, cv2.LINE_AA)
    source = tmp_path / 'rings.png'
    report = tmp_path / 'report.json'
    snapshot = tmp_path / 'snapshot.png'
    assert cv2.imwrite(str(source), image)
    args = ['--image', str(source), '--flip-code', '0', '--no-show', '--frames', '10',
            '--report', str(report), '--snapshot', str(snapshot)]
    assert camera_diagnostics.main(args) == 0
    result = json.loads(report.read_text())
    assert result['detected_frames'] == 10
    assert result['stable_frames'] == 4
    assert np.allclose(result['last_result']['centers_px'],
                       [(199, 227), (328, 227), (458, 227)], atol=1)
    assert cv2.imread(str(snapshot)).shape == image.shape
    assert cv2.imwrite(str(source), np.full_like(image, 240))
    assert camera_diagnostics.main(args) == 2
    result = json.loads(report.read_text())
    assert result['detected_frames'] == 0
    assert result['last_result']['centers_px'] == []
    assert result['last_result']['affine_residual'] is None
