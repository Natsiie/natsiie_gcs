"""Image motion from OpenCV optical flow, independent of pickup color recognition."""

import cv2
import numpy as np

from identify_pack.scripts.motion import TurntableMotionTracker


class OpticalFlowMotion:
    def __init__(self, rois=None, **motion_params):
        self.rois = rois
        self.tracker = TurntableMotionTracker(**motion_params)
        self.gray = None
        self.features = None
        self.timestamp = None
        self.distance = 0.0

    def update(self, image, timestamp):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        speed = None
        interval = 0 if self.timestamp is None else timestamp - self.timestamp
        if (self.gray is not None and self.gray.shape == gray.shape
                and self.features is not None and 0.02 <= interval <= 0.25):
            points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.gray, gray, self.features, None,
                winSize=(21, 21), maxLevel=3)
            if points is not None and status is not None:
                valid = status.reshape(-1) == 1
                displacement = np.linalg.norm(
                    points.reshape(-1, 2)[valid] - self.features.reshape(-1, 2)[valid], axis=1)
                displacement = displacement[np.isfinite(displacement)]
                if len(displacement) >= 6:
                    speed = float(np.percentile(displacement, 90)) / interval
        mask = None
        if self.rois:
            mask = np.zeros(gray.shape, dtype=np.uint8)
            for x, y, w, h in self.rois:
                left, top = max(0, x), max(0, y)
                right, bottom = min(gray.shape[1], x + w), min(gray.shape[0], y + h)
                if right > left and bottom > top:
                    mask[top:bottom, left:right] = 255
        self.features = cv2.goodFeaturesToTrack(
            gray, maxCorners=150, qualityLevel=0.01, minDistance=7, mask=mask)
        self.gray, self.timestamp = gray, timestamp
        if speed is None:
            # Missing texture or a frame gap cannot prove the turntable has stopped.
            self.tracker.reset()
            self.distance = 0.0
            result = self.tracker.update([(0, self.distance, 0.)], timestamp)
            result['state'] = 'ACQUIRING'
            return result
        self.distance += speed * interval
        return self.tracker.update([(0, self.distance, 0.)], timestamp)
