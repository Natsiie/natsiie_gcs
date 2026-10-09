"""Isolate blocking V4L2 calls in a disposable process; share only the latest frame."""
import multiprocessing as mp
import time

import numpy as np


def capture_worker(device, width, height, fps, fourcc, pixels, sequence, shape, stamp):
    import cv2

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera {device}')
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        format_name = ''.join(chr((actual_fourcc >> (8 * i)) & 255) for i in range(4))
        print(f'[capture] {device}: negotiated '
              f'{cap.get(cv2.CAP_PROP_FRAME_WIDTH):g}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):g} '
              f'{cap.get(cv2.CAP_PROP_FPS):g}fps {format_name}', flush=True)
        destination = np.frombuffer(pixels, dtype=np.uint8)
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
                raise RuntimeError(f'Unsupported camera frame: {frame.shape} {frame.dtype}')
            if frame.size > destination.size:
                raise RuntimeError(f'Camera returned {frame.shape}; requested {width}x{height}')
            # An odd sequence marks a write in progress. No lock can be left held on kill.
            sequence.value += 1
            destination[:frame.size] = frame.reshape(-1)
            shape[0], shape[1] = frame.shape[:2]
            stamp.value = time.monotonic()
            sequence.value += 1
    finally:
        cap.release()


class CaptureProcess:
    def __init__(self, device, width, height, fps, fourcc, worker=capture_worker):
        self.context = mp.get_context('spawn')
        self.config = (device, width, height, fps, fourcc)
        self.worker = worker
        self.process = None
        self.last_sequence = 0

    def start(self):
        if self.process is not None:
            raise RuntimeError('Capture process already exists')
        _, width, height, _, _ = self.config
        self.pixels = self.context.RawArray('B', width * height * 3)
        self.sequence = self.context.RawValue('Q', 0)
        self.shape = self.context.RawArray('i', 2)
        self.stamp = self.context.RawValue('d', 0.0)
        self.last_sequence = 0
        self.process = self.context.Process(
            target=self.worker, args=(*self.config, self.pixels, self.sequence,
                                      self.shape, self.stamp), daemon=True)
        self.process.start()

    def latest(self):
        sequence = self.sequence.value
        if sequence == 0 or sequence % 2 or sequence == self.last_sequence:
            return None
        height, width = self.shape
        size = height * width * 3
        if height <= 0 or width <= 0 or size > len(self.pixels):
            return None
        frame = np.frombuffer(self.pixels, dtype=np.uint8, count=size).copy()
        stamp = self.stamp.value
        if sequence != self.sequence.value:
            return None
        self.last_sequence = sequence
        return frame.reshape(height, width, 3), stamp

    def stop(self):
        if self.process is None:
            return True
        # Do not call release() concurrently with read(); the OS closes the worker's FD.
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=0.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=0.5)
        if self.process.is_alive():
            return False
        self.process.close()
        self.process = None
        return True


class TimedCamera:
    """VideoCapture-style reads with a deadline, including driver open/read hangs."""

    def __init__(self, device, width, height, fps, fourcc='MJPG', timeout=5.0):
        if (width <= 0 or height <= 0 or fps <= 0 or timeout <= 0
                or (fourcc and len(fourcc) != 4)):
            raise ValueError('Invalid camera dimensions, fps, format or timeout')
        if isinstance(device, str) and device.isdecimal():
            device = int(device)
        self.capture = CaptureProcess(device, width, height, fps, fourcc)
        self.timeout = timeout
        self.capture.start()

    def read(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self.capture.process is None or not self.capture.process.is_alive():
                return False, None
            result = self.capture.latest()
            if result is not None:
                return True, result[0]
            time.sleep(0.005)
        return False, None

    def release(self):
        if not self.capture.stop():
            raise RuntimeError('Camera worker could not be stopped')
