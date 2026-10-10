"""Printed ring IDs, independent of ROS, camera capture and motion control."""

from collections import deque
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

PATCH_SIZE = 160
DIGIT_SIZE = 40


def rectify_ring(frame, ring):
    ellipse = ring.ellipse
    angle = np.deg2rad(ellipse.angle_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    radius = PATCH_SIZE * 0.45
    transform = rotation @ np.diag(np.asarray(ellipse.axes) / (2 * radius)) @ rotation.T
    yy, xx = np.mgrid[:PATCH_SIZE, :PATCH_SIZE].astype(np.float32)
    offsets = np.stack((xx - PATCH_SIZE / 2, yy - PATCH_SIZE / 2), axis=-1)
    source = offsets @ transform.T + ring.center
    valid = ((source[..., 0] >= 0) & (source[..., 0] <= frame.shape[1] - 1)
             & (source[..., 1] >= 0) & (source[..., 1] <= frame.shape[0] - 1))
    patch = cv2.remap(frame, source[..., 0].astype(np.float32),
                      source[..., 1].astype(np.float32), cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_REPLICATE)
    return patch, valid


def normalize_digit(binary):
    points = cv2.findNonZero(binary)
    if points is None:
        return None
    x, y, width, height = cv2.boundingRect(points)
    side = max(width, height) + 8
    canvas = np.zeros((side, side), np.uint8)
    left, top = (side - width) // 2, (side - height) // 2
    canvas[top:top + height, left:left + width] = binary[y:y + height, x:x + width]
    return cv2.resize(canvas, (DIGIT_SIZE, DIGIT_SIZE), interpolation=cv2.INTER_AREA)


def extract_digit(patch, valid, inner_ratio=0.55):
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    yy, xx = np.mgrid[:PATCH_SIZE, :PATCH_SIZE]
    distance = np.hypot(xx - PATCH_SIZE / 2, yy - PATCH_SIZE / 2)
    radius = PATCH_SIZE * 0.45 * inner_ratio
    mask = distance < radius
    if not np.all(valid[mask]) or np.ptp(gray[mask]) < 20:
        return None
    threshold, _ = cv2.threshold(gray[mask].reshape(-1, 1), 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    boundary = mask & (distance > radius * 0.85)
    background_is_light = np.median(gray[boundary]) > threshold
    foreground = (gray <= threshold) if background_is_light else (gray > threshold)
    binary = (foreground & mask).astype(np.uint8) * 255
    count, labels, stats, centers = cv2.connectedComponentsWithStats(binary)
    choices = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        offset = np.linalg.norm(centers[index] - PATCH_SIZE / 2) / radius
        if (area < 12 or area > mask.sum() * 0.5 or height < radius * 0.35
                or height > radius * 1.8 or width > radius * 1.8 or offset > 0.55
                or np.any((labels == index) & (distance >= radius - 2))):
            continue
        choices.append((area / (1 + offset), index))
    if not choices:
        return None
    index = max(choices)[1]
    return normalize_digit((labels == index).astype(np.uint8) * 255)


def load_templates(directory, allow_builtin=True):
    """Load normalized white-on-black samples; real samples override each label."""
    directory = Path(directory)
    templates, sources = {}, {}
    for label in (1, 2, 3):
        paths = sorted(directory.glob(f'digit_{label}_*.png'))
        base = directory / f'digit_{label}.png'
        if base.exists():
            paths.insert(0, base)
        captured = []
        for path in paths:
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None or np.ptp(image) < 20:
                raise ValueError(f'Invalid digit template: {path}')
            captured.append(cv2.resize(image, (DIGIT_SIZE, DIGIT_SIZE)))
        templates[label] = captured
        sources[label] = 'captured' if captured else 'builtin'
        if captured:
            continue
        if not allow_builtin:
            raise ValueError(f'Missing printed digit {label} templates in {directory}')
        for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX,
                     cv2.FONT_HERSHEY_COMPLEX, cv2.FONT_HERSHEY_TRIPLEX):
            for thickness in (2, 3, 4):
                canvas = np.zeros((90, 90), np.uint8)
                cv2.putText(canvas, str(label), (16, 68), font, 2, 255,
                            thickness, cv2.LINE_AA)
                _, binary = cv2.threshold(canvas, 80, 255, cv2.THRESH_BINARY)
                templates[label].append(normalize_digit(binary))
    return templates, sources


def match_digit(digit, templates, minimum=0.55, margin=0.08):
    if digit is None:
        return None, 0.0, 0.0
    scores = sorted((max(float(cv2.matchTemplate(digit, template,
                         cv2.TM_CCOEFF_NORMED)[0, 0]) for template in images), label)
                    for label, images in templates.items())
    score, label = scores[-1]
    gap = score - scores[-2][0]
    return (label if score >= minimum and gap >= margin else None), score, gap


def recognize_rings(frame, rings, templates, inner_ratio=0.55, minimum=0.55, margin=0.08):
    results, crops = [], []
    for index, ring in enumerate(rings):
        patch, valid = rectify_ring(frame, ring)
        digit = extract_digit(patch, valid, inner_ratio)
        label, score, gap = match_digit(digit, templates, minimum, margin)
        results.append({
            'slot': index + 1, 'center_px': ring.center.tolist(),
            'ellipse_axes_px': list(ring.ellipse.axes),
            'ellipse_angle_deg': ring.ellipse.angle_deg,
            'contour_support': ring.support, 'diameter_bands': ring.diameter_bands,
            'digit': label, 'score': score, 'score_margin': gap,
        })
        crops.append((patch, digit))
    return results, crops


def placement_plan(report, order=(1, 2, 3)):
    """Resolve task-code ring order to measured pixels; never infer a missing ID."""
    if len(order) != 3 or set(order) != {1, 2, 3}:
        raise ValueError('Placement order must be a permutation of 1/2/3')
    if not report.get('placement_mapping_valid', report['complete_digit_mapping']):
        return []
    mapping = {ring.get('ring_id', ring['digit']): ring for ring in report['rings']}
    return [{'sequence': index + 1, 'digit': label,
             'center_px': mapping[label]['center_px'], 'slot': mapping[label]['slot']}
            for index, label in enumerate(order)]


def recognize_detection(frame, detection, templates, inner_ratio=0.55,
                        minimum=0.55, margin=0.08):
    rings = []
    if detection.detected:
        for center, (_, axes, angle) in zip(detection.centers_px, detection.ellipses):
            rings.append(SimpleNamespace(
                center=np.asarray(center), support=0, diameter_bands=0,
                ellipse=SimpleNamespace(axes=axes, angle_deg=angle)))
    rings.sort(key=lambda ring: (ring.center[0], ring.center[1]))
    results, crops = recognize_rings(frame, rings, templates, inner_ratio, minimum, margin)
    complete = len(results) == 3 and {ring['digit'] for ring in results} == {1, 2, 3}
    return {'three_ring_geometry': detection.detected,
            'complete_digit_mapping': complete, 'geometry_stable': detection.stable,
            'within_tolerance': detection.within_tolerance, 'rings': results}, crops


class DigitMappingTracker:
    """Confirm observed IDs, or the caller's guaranteed left-to-right numbering."""

    def __init__(self, frame_count=7, maximum_jitter_px=4.0, expected_digits=None):
        if frame_count < 2 or not np.isfinite(maximum_jitter_px) or maximum_jitter_px <= 0:
            raise ValueError('Invalid digit stability window')
        self.samples = deque(maxlen=frame_count)
        self.maximum_jitter_px = maximum_jitter_px
        self.image_size = None
        if expected_digits is not None and tuple(expected_digits) != (1, 2, 3):
            raise ValueError('Fixed station numbering must be left-to-right 1/2/3')
        self.expected_digits = expected_digits

    def reset(self):
        self.samples.clear()

    def update(self, report, image_size):
        if self.image_size != image_size:
            self.reset()
            self.image_size = image_size
        report['digit_mapping_stable'] = False
        report['centers_by_digit_px'] = {}
        report['stable_frames'] = 0
        ordered = sorted(report['rings'], key=lambda ring: ring['center_px'][0])
        report['numbering_conflict'] = False
        if self.expected_digits is not None:
            report['numbering_source'] = 'fixed_left_to_right'
            geometry = report['three_ring_geometry'] and len(ordered) == 3
            conflict = geometry and any(ring['digit'] not in (None, label)
                                       for ring, label in zip(ordered, self.expected_digits))
            report['numbering_conflict'] = bool(conflict)
            valid = geometry and not conflict
            for ring, label in zip(ordered, self.expected_digits):
                ring['ring_id'] = label if valid else None
        else:
            report['numbering_source'] = 'templates'
            valid = report['complete_digit_mapping']
            for ring in ordered:
                ring['ring_id'] = ring['digit'] if valid else None
        report['placement_mapping_valid'] = bool(valid)
        if not valid:
            self.reset()
            return report
        rings = sorted(ordered, key=lambda ring: ring['ring_id'])
        signature = tuple(ring['ring_id'] for ring in ordered)
        centers = np.asarray([ring['center_px'] for ring in rings])
        if self.samples:
            previous_signature, _ = self.samples[-1]
            history = np.asarray([sample[1] for sample in self.samples])
            if (signature != previous_signature
                    or np.max(np.linalg.norm(history - centers, axis=2)) > self.maximum_jitter_px):
                self.reset()
        self.samples.append((signature, centers))
        report['stable_frames'] = len(self.samples)
        if len(self.samples) == self.samples.maxlen:
            median = np.median([sample[1] for sample in self.samples], axis=0)
            report['digit_mapping_stable'] = True
            report['centers_by_digit_px'] = {str(index + 1): point.tolist()
                                            for index, point in enumerate(median)}
        return report
