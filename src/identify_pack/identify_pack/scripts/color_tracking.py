"""Full-frame HSV detections and multiple constant-velocity tracks, without ROS."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

from .color_config import PICKUP_COLOR_IDS, find_color_blobs, make_all_color_masks


@dataclass(frozen=True)
class ColorDetection:
    color_id: int
    center: tuple
    bbox: tuple
    area: float
    score: float


def box_overlap(first, second):
    """Return IoU and intersection / smaller box area."""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    intersection = max(0., min(ax + aw, bx + bw) - max(ax, bx)) * max(
        0., min(ay + ah, by + bh) - max(ay, by))
    small = min(aw * ah, bw * bh)
    union = aw * ah + bw * bh - intersection
    return intersection / max(union, 1.), intersection / max(small, 1.)


def detect_color_objects(frame, config, min_area=300., max_area_fraction=0.15,
                         min_dimension=12, max_aspect_ratio=3., min_solidity=0.65,
                         min_extent=0.3, open_kernel_px=3, close_kernel_px=5,
                         duplicate_overlap=0.7):
    """Detect all six pickup colors; no ROI, whole-frame dominant color or top-3 cap.

    Shape and border filtering reject obvious background. Overlapping candidates
    from competing HSV masks are suppressed, but adjacent separate colors stay
    separate. These are color blobs, not a semantic object detector.
    """
    if (not math.isfinite(min_area) or min_area <= 0
            or not 0 < max_area_fraction <= 1 or min_dimension < 1
            or not math.isfinite(max_aspect_ratio) or max_aspect_ratio < 1
            or not 0 <= min_solidity <= 1 or not 0 <= min_extent <= 1
            or not 0 < duplicate_overlap <= 1
            or open_kernel_px < 1 or close_kernel_px < 1):
        raise ValueError('Invalid color detection thresholds')
    all_masks = make_all_color_masks(frame, config)
    # Pink marker pixels can also fall inside a broad red HSV range. Excluding
    # label 7 alone would still create a red robot-body target.
    marker = all_masks[7] > 0
    masks = {color: np.where(marker, 0, mask).astype(np.uint8)
             for color, mask in all_masks.items() if color in PICKUP_COLOR_IDS}
    # The blue/light-blue threshold often divides one shaded object. Segment their
    # union once, then classify that individual candidate using both raw masks.
    segmentation = dict(masks)
    segmentation[3] = np.bitwise_or(masks[3], masks[6])
    segmentation.pop(6)
    height, width = frame.shape[:2]
    blobs = find_color_blobs(
        segmentation, min_area=min_area, min_dimension=min_dimension,
        open_kernel_px=open_kernel_px, close_kernel_px=close_kernel_px,
        max_aspect_ratio=max_aspect_ratio, min_solidity=min_solidity,
        min_extent=min_extent, roi=None)
    candidates = []
    for color, cx, cy, x, y, w, h, area in blobs:
        if (area > width * height * max_area_fraction
                or x <= 0 or y <= 0 or x + w >= width or y + h >= height):
            continue
        if color == 3:
            color = max((3, 6), key=lambda label:
                        np.count_nonzero(masks[label][y:y + h, x:x + w]))
        # Fraction of this candidate box supported by its raw HSV mask.
        score = float(np.count_nonzero(masks[color][y:y + h, x:x + w])) / (w * h)
        candidates.append(ColorDetection(color, (cx, cy), (x, y, w, h), area, score))
    retained = []
    for candidate in sorted(candidates, key=lambda item: item.area * item.score, reverse=True):
        if any(box_overlap(candidate.bbox, old.bbox)[1] >= duplicate_overlap
               for old in retained):
            continue
        retained.append(candidate)
    return sorted(retained, key=lambda item: (item.center[0], item.center[1], item.color_id))


class _Track:
    def __init__(self, track_id, detection, timestamp, history_length, color_window):
        self.track_id = track_id
        self.state = np.array([*detection.center, 0., 0.], dtype=float)
        self.covariance = np.diag([9., 9., 10000., 10000.])
        self.created = self.last_seen = timestamp
        self.hits = self.hit_streak = 1
        self.confirmed = False
        self.observed = True
        self.missed_frames = self.low_speed_frames = 0
        self.bbox = tuple(float(value) for value in detection.bbox)
        self.area = detection.area
        self.score = detection.score
        self.measurement = detection.center
        self.colors = deque(maxlen=color_window)
        self.colors.append((detection.color_id, max(0.05, detection.score)))
        self.history = deque(maxlen=history_length)
        self.history.append(list(detection.center))

    def color(self):
        votes = {}
        for color, weight in self.colors:
            votes[color] = votes.get(color, 0.) + weight
        # Most recent observation breaks equal accumulated scores.
        label = max(votes, key=lambda color: (votes[color], color == self.colors[-1][0]))
        return label, votes[label] / sum(votes.values())

    def predict(self, interval, acceleration_std):
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = interval
        noise_map = np.array([[interval**2 / 2, 0.], [0., interval**2 / 2],
                              [interval, 0.], [0., interval]])
        self.state = transition @ self.state
        self.covariance = (transition @ self.covariance @ transition.T
                           + acceleration_std**2 * noise_map @ noise_map.T)
        # Recenter the association box on the filtered prediction instead of
        # carrying the previous raw measurement's position offset forward.
        _, _, w, h = self.bbox
        self.bbox = (self.state[0] - w / 2, self.state[1] - h / 2, w, h)
        self.observed = False

    def correct(self, detection, timestamp, measurement_std):
        observation = np.eye(2, 4)
        noise = np.eye(2) * measurement_std**2
        innovation = np.asarray(detection.center) - observation @ self.state
        covariance = observation @ self.covariance @ observation.T + noise
        gain = np.linalg.solve(covariance, observation @ self.covariance).T
        self.state += gain @ innovation
        residual = np.eye(4) - gain @ observation
        self.covariance = residual @ self.covariance @ residual.T + gain @ noise @ gain.T
        self.bbox = tuple(float(value) for value in detection.bbox)
        self.area, self.score = detection.area, detection.score
        self.measurement = detection.center
        self.last_seen = timestamp
        self.hits += 1
        self.hit_streak += 1
        self.missed_frames = 0
        self.observed = True
        self.colors.append((detection.color_id, max(0.05, detection.score)))
        self.history.append(self.state[:2].tolist())


class ColorMultiTracker:
    """Kalman prediction + gated Hungarian assignment + weighted color history.

    IDs stay unique during the process lifetime, including after reset. Predictions
    may be displayed briefly, but cannot be mistaken for current measurements.
    """

    def __init__(self, min_confirmed_hits=3, max_missed_sec=0.5, max_frame_gap_sec=1.,
                 max_match_distance_px=80., max_area_ratio=3.,
                 measurement_std_px=3., acceleration_std_px_s2=80.,
                 color_window=7, history_length=30, stationary_speed_px_s=8.,
                 stationary_frames=5):
        numbers = (max_missed_sec, max_frame_gap_sec, max_match_distance_px,
                   max_area_ratio, measurement_std_px, acceleration_std_px_s2,
                   stationary_speed_px_s)
        if (not all(math.isfinite(value) and value > 0 for value in numbers)
                or max_area_ratio <= 1 or max_missed_sec > max_frame_gap_sec
                or any(not isinstance(value, int) or value < 1 for value in
                       (min_confirmed_hits, color_window, history_length, stationary_frames))):
            raise ValueError('Invalid color tracking parameters')
        self.min_hits = min_confirmed_hits
        self.max_missed = max_missed_sec
        self.max_gap = max_frame_gap_sec
        self.max_distance = max_match_distance_px
        self.max_area_ratio = max_area_ratio
        self.measurement_std = measurement_std_px
        self.acceleration_std = acceleration_std_px_s2
        self.color_window = color_window
        self.history_length = history_length
        self.stationary_speed = stationary_speed_px_s
        self.stationary_frames = stationary_frames
        self._next_id = 1
        self.reset()

    def reset(self):
        self.tracks = []
        self.timestamp = None
        self.image_size = None

    def update(self, detections, timestamp, image_size):
        if not math.isfinite(timestamp) or timestamp <= 0:
            raise ValueError('A positive finite image timestamp is required')
        if len(image_size) != 2 or any(value < 1 for value in image_size):
            raise ValueError('Invalid image size')
        for detection in detections:
            values = (*detection.center, *detection.bbox, detection.area, detection.score)
            if (len(detection.center) != 2 or len(detection.bbox) != 4
                    or not all(math.isfinite(value) for value in values)
                    or detection.color_id not in PICKUP_COLOR_IDS or detection.area <= 0
                    or min(detection.bbox[2:]) <= 0 or not 0 <= detection.score <= 1
                    or not 0 <= detection.center[0] < image_size[0]
                    or not 0 <= detection.center[1] < image_size[1]):
                raise ValueError('Invalid color detection')
        if self.timestamp is not None and timestamp <= self.timestamp:
            raise ValueError('Image timestamps must increase; reset after a clock restart')
        if (self.image_size != tuple(image_size)
                or (self.timestamp is not None and timestamp - self.timestamp > self.max_gap)):
            self.reset()
        interval = 0. if self.timestamp is None else timestamp - self.timestamp
        self.timestamp, self.image_size = timestamp, tuple(image_size)
        self.tracks = [track for track in self.tracks if timestamp - track.last_seen <= self.max_missed]
        for track in self.tracks:
            track.predict(interval, self.acceleration_std)

        # One dummy column per track allows unmatched tracks without forcing an
        # infeasible match or discarding a feasible assignment elsewhere.
        count, size = len(self.tracks), len(detections)
        costs = np.full((count, size + count), 1e6)
        for row, track in enumerate(self.tracks):
            costs[row, size + row] = 1.2
            color, _ = track.color()
            for column, detection in enumerate(detections):
                distance = np.linalg.norm(track.state[:2] - detection.center)
                ratio = max(track.area / detection.area, detection.area / track.area)
                iou, _ = box_overlap(track.bbox, detection.bbox)
                different = detection.color_id != color
                compatible_blue = {color, detection.color_id} == {3, 6}
                if (distance > self.max_distance or ratio > self.max_area_ratio
                        or (different and not compatible_blue
                            and (distance > self.max_distance * 0.3 or iou < 0.3))):
                    continue
                costs[row, column] = (
                    0.6 * distance / self.max_distance + 0.2 * (1 - iou)
                    + 0.1 * math.log(ratio) / math.log(self.max_area_ratio)
                    + (0.25 if compatible_blue else 0.4) * different)
        matched = set()
        if count:
            rows, columns = linear_sum_assignment(costs)
            for row, column in zip(rows, columns):
                track = self.tracks[row]
                if column < size and costs[row, column] < 1.2:
                    track.correct(detections[column], timestamp, self.measurement_std)
                    track.confirmed = track.confirmed or track.hit_streak >= self.min_hits
                    speed = float(np.linalg.norm(track.state[2:]))
                    track.low_speed_frames = track.low_speed_frames + 1 if speed <= self.stationary_speed else 0
                    matched.add(column)
                else:
                    track.hit_streak = track.low_speed_frames = 0
                    track.missed_frames += 1
        # Unconfirmed noise must have consecutive detections to become a track.
        self.tracks = [track for track in self.tracks if track.confirmed or track.observed]
        for index, detection in enumerate(detections):
            if index in matched:
                continue
            track = _Track(self._next_id, detection, timestamp, self.history_length, self.color_window)
            track.confirmed = self.min_hits == 1
            self.tracks.append(track)
            self._next_id += 1
        return self.snapshot()

    def snapshot(self):
        results = []
        for track in self.tracks:
            color, consistency = track.color()
            results.append({
                'track_id': track.track_id, 'color_id': color,
                'color_consistency': consistency, 'detection_score': float(track.score),
                'state': ('lost' if not track.observed else
                          'confirmed' if track.confirmed else 'tentative'),
                'confirmed': track.confirmed, 'observed': track.observed,
                'center_px': track.state[:2].tolist(),
                'measurement_center_px': list(track.measurement) if track.observed else None,
                'bbox_px': list(track.bbox), 'area_px2': float(track.area),
                'velocity_px_s': track.state[2:].tolist(),
                'position_std_px': np.sqrt(np.maximum(0., np.diag(track.covariance)[:2])).tolist(),
                'last_seen_sec': float(track.last_seen),
                'missed_sec': float(self.timestamp - track.last_seen),
                'hits': track.hits, 'missed_frames': track.missed_frames,
                'stationary': bool(track.confirmed and track.observed
                                   and track.low_speed_frames >= self.stationary_frames),
                'history_px': list(track.history),
            })
        return sorted(results, key=lambda item: item['track_id'])
