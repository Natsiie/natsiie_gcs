"""Detect the circular top of the preliminary-round part before classifying color.

Rule figure 5: top diameter 30 mm, bottom diameter 50 mm, height 60 mm.
Ellipse diameters below are pixels, since no camera pose/scale is assumed here.
This is geometric evidence, not semantic proof: a clean circular shadow can
still pass. Optional side checks assume a configured image-up/down body direction.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .color_config import PICKUP_COLOR_IDS, make_all_color_masks
from .color_tracking import ColorDetection, box_overlap


@dataclass(frozen=True)
class MaterialTopConfig:
    min_minor_axis_px: float = 12.0
    max_major_axis_px: float = 120.0
    min_aspect: float = 0.35
    canny_low: int = 35
    canny_high: int = 100
    edge_close_kernel_px: int = 5
    min_contour_points: int = 16
    max_fit_error: float = 0.07
    min_contour_fill: float = 0.75
    min_edge_coverage: float = 0.70
    edge_tolerance_px: float = 2.0
    min_boundary_contrast: float = 10.0
    inner_ratio: float = 0.70
    min_color_support: float = 0.45
    min_color_margin: float = 0.08
    black_min_edge_coverage: float = 0.82
    black_min_dark_boundary: float = 0.45
    black_max_outer_support: float = 0.55
    duplicate_overlap: float = 0.60
    body_direction: int = 0
    min_body_support: float = 0.35

    def __post_init__(self):
        positives = (self.min_minor_axis_px, self.max_major_axis_px, self.max_fit_error,
                     self.edge_tolerance_px, self.min_boundary_contrast)
        fractions = (self.min_aspect, self.min_contour_fill, self.min_edge_coverage,
                     self.inner_ratio, self.min_color_support, self.black_min_edge_coverage,
                     self.black_min_dark_boundary, self.duplicate_overlap)
        if (not all(math.isfinite(value) and value > 0 for value in positives)
                or self.min_minor_axis_px >= self.max_major_axis_px
                or not all(0 < value <= 1 for value in fractions)
                or not 0 <= self.min_color_margin <= 1
                or not 0 <= self.black_max_outer_support <= 1
                or not 0 <= self.min_body_support <= 1
                or not 0 < self.canny_low < self.canny_high <= 255
                or self.edge_close_kernel_px < 1 or self.edge_close_kernel_px % 2 != 1
                or self.min_contour_points < 5
                or self.body_direction not in (-1, 0, 1)):
            raise ValueError('Invalid material top detector parameters')


def _rotation(angle):
    theta = math.radians(angle)
    return np.array([[math.cos(theta), -math.sin(theta)],
                     [math.sin(theta), math.cos(theta)]])


def _ellipse_boundary(ellipse, scale=1.0):
    center, axes, angle = ellipse
    phase = np.linspace(0, 2 * math.pi, 96, endpoint=False)
    offsets = np.column_stack((np.cos(phase), np.sin(phase))) * np.asarray(axes) * (scale / 2)
    return offsets @ _rotation(angle).T + center


def _sample(image, points):
    return cv2.remap(image, points[:, 0].astype(np.float32).reshape(1, -1),
                     points[:, 1].astype(np.float32).reshape(1, -1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(-1)


def _top_pixels(frame_shape, ellipse, ratio):
    """Return a tight crop and an analytic ellipse mask, preserving perspective."""
    center, axes, angle = ellipse
    points = _ellipse_boundary(ellipse, ratio)
    x0, y0 = np.floor(points.min(axis=0)).astype(int)
    x1, y1 = np.ceil(points.max(axis=0)).astype(int) + 1
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(frame_shape[1], x1), min(frame_shape[0], y1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    local = (np.stack((xx, yy), axis=-1) - center) @ _rotation(angle)
    normalized = local / (np.asarray(axes) * ratio / 2)
    return (slice(y0, y1), slice(x0, x1)), np.sum(normalized**2, axis=-1) <= 1


def _side_support(mask, ellipse, direction):
    """Check a widening colored strip below/above the top, using 50/30 ratio.

    Image body length is inferred only approximately from top flattening; it is
    not a 3D pose solution. Disable for near-top-down or non-vertical image views.
    """
    center, axes, _ = ellipse
    major, minor = max(axes), min(axes)
    aspect = minor / major
    length = major * (60 / 30) * math.sqrt(max(0., 1 - aspect**2))
    if length < 4:
        return None
    # Sample away from top and bottom rims to inspect the frustum body interior.
    fraction = np.linspace(0.3, 0.8, 16)
    horizontal = np.linspace(-0.55, 0.55, 13)
    radius = major * 0.5 * (1 + (50 / 30 - 1) * fraction)
    xx = center[0] + radius[:, None] * horizontal[None, :]
    yy = np.broadcast_to(center[1] + direction * length * fraction[:, None], xx.shape)
    if np.any((xx < 0) | (xx >= mask.shape[1] - 1) | (yy < 0) | (yy >= mask.shape[0] - 1)):
        return 0.0
    sampled = _sample(mask, np.column_stack((xx.reshape(-1), yy.reshape(-1))))
    return float(np.mean(sampled > 127))


def detect_material_tops(frame, color_config, config=None, diagnostics=None):
    """Find tops on the whole image; return only shape-validated, known colors.

    Optional diagnostics list includes rejected candidates and reasons for tuning.
    No fallback to unvalidated HSV blobs: a rejected shadow cannot re-enter there.
    """
    config = config or MaterialTopConfig()
    masks = make_all_color_masks(frame, color_config)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0.8)
    edges = cv2.Canny(blurred, config.canny_low, config.canny_high)
    # Gray alone attenuates red/blue top-to-side shading. Value edges retain
    # that rim; gray still supplies the outer edge against a bright background.
    value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    value = cv2.GaussianBlur(value, (3, 3), 0.8)
    edges |= cv2.Canny(value, config.canny_low, config.canny_high)
    distances = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    # Close small T-junction gaps where the top rim meets the side outline.
    # Validate coverage against the original edges, never the invented pixels.
    proposals = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                     (config.edge_close_kernel_px, config.edge_close_kernel_px)))
    contours, _ = cv2.findContours(proposals, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []

    def reject(ellipse, reason, **metrics):
        if diagnostics is not None:
            diagnostics.append({'ellipse': ellipse, 'accepted': False, 'reason': reason, **metrics})

    for contour in contours:
        if len(contour) < config.min_contour_points:
            continue
        ellipse = cv2.fitEllipse(contour)
        center, axes, angle = ellipse
        major, minor = max(axes), min(axes)
        if (minor < config.min_minor_axis_px or major > config.max_major_axis_px
                or minor / major < config.min_aspect):
            continue
        points = contour.reshape(-1, 2)
        normalized = ((points - center) @ _rotation(angle)) / (np.asarray(axes) / 2)
        error = float(np.sqrt(np.mean((np.linalg.norm(normalized, axis=1) - 1)**2)))
        fill = float(cv2.contourArea(contour)) / (math.pi * axes[0] * axes[1] / 4)
        if error > config.max_fit_error or fill < config.min_contour_fill:
            reject(ellipse, 'ellipse_fit', fit_error=error, contour_fill=fill)
            continue
        boundary = _ellipse_boundary(ellipse)
        outer = _ellipse_boundary(ellipse, 1.18)
        if np.any((outer[:, 0] < 1) | (outer[:, 0] >= frame.shape[1] - 2)
                  | (outer[:, 1] < 1) | (outer[:, 1] >= frame.shape[0] - 2)):
            reject(ellipse, 'clipped')
            continue
        coverage = float(np.mean(_sample(distances, boundary) <= config.edge_tolerance_px))
        inside_values = _sample(blurred.astype(np.float32), _ellipse_boundary(ellipse, 0.88))
        outside_values = _sample(blurred.astype(np.float32), outer)
        difference = outside_values - inside_values
        contrast = float(np.median(np.abs(difference)))
        if coverage < config.min_edge_coverage or contrast < config.min_boundary_contrast:
            reject(ellipse, 'weak_boundary', edge_coverage=coverage, boundary_contrast=contrast)
            continue
        region, disk = _top_pixels(frame.shape, ellipse, config.inner_ratio)
        count = int(disk.sum())
        if count < 20:
            reject(ellipse, 'small_interior')
            continue
        # A marker-colored top cannot become a pickup target via an overlapping
        # broad red threshold. Pixels excluded still count in the denominator.
        marker = masks[7][region] > 0
        scores = sorted((float(np.count_nonzero((masks[color][region] > 0) & disk & ~marker)) / count, color)
                        for color in PICKUP_COLOR_IDS)
        score, color = scores[-1]
        margin = score - scores[-2][0]
        if score < config.min_color_support or margin < config.min_color_margin:
            reject(ellipse, 'unknown_color', color_support=score, color_margin=margin)
            continue
        dark_boundary = float(np.mean(difference >= config.min_boundary_contrast))
        outer_black = float(np.mean(_sample(masks[5], outer) > 127))
        if color == 5 and (coverage < config.black_min_edge_coverage
                           or dark_boundary < config.black_min_dark_boundary
                           or outer_black > config.black_max_outer_support):
            reject(ellipse, 'black_shadow_check', edge_coverage=coverage,
                   dark_boundary=dark_boundary, outer_black_support=outer_black)
            continue
        body_support = None
        if config.body_direction:
            body_support = _side_support(masks[color], ellipse, config.body_direction)
            if body_support is not None and body_support < config.min_body_support:
                reject(ellipse, 'body_structure', body_support=body_support)
                continue
        x0, y0 = np.floor(boundary.min(axis=0)).astype(int)
        x1, y1 = np.ceil(boundary.max(axis=0)).astype(int)
        shape_score = float(coverage * max(0., 1 - error / config.max_fit_error))
        candidates.append(ColorDetection(
            int(color), tuple(float(value) for value in center),
            (int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)),
            float(math.pi * axes[0] * axes[1] / 4), score,
            ellipse, shape_score, 'material_top'))
        if diagnostics is not None:
            diagnostics.append({'ellipse': ellipse, 'accepted': True, 'reason': 'accepted',
                                'color_id': int(color), 'fit_error': error,
                                'edge_coverage': coverage, 'color_support': score,
                                'body_support': body_support})
    retained = []
    # Canny generally produces inner and outer contours for one physical edge.
    for candidate in sorted(candidates, key=lambda item: (item.shape_score, item.score), reverse=True):
        if any(box_overlap(candidate.bbox, old.bbox)[1] >= config.duplicate_overlap
               for old in retained):
            continue
        retained.append(candidate)
    return sorted(retained, key=lambda item: item.center[0])
