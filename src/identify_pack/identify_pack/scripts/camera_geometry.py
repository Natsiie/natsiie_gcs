"""Planar camera calibration and projection; rigid transforms use metres."""

from __future__ import annotations

import cv2
import numpy as np


def load_intrinsics(matrix_path, distortion_path):
    matrix = np.load(matrix_path, allow_pickle=False).astype(np.float64)
    distortion = np.load(distortion_path, allow_pickle=False).astype(np.float64).reshape(-1)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])):
        raise ValueError('Invalid camera matrix')
    if distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError('Invalid distortion coefficients')
    return matrix, distortion


def estimate_camera_transform(pixels, xy_mm, plane_z_mm, matrix, distortion):
    """Return gimbal_link <- camera_optical_frame from known horizontal points."""
    pixels = np.asarray(pixels, dtype=np.float64)
    xy_mm = np.asarray(xy_mm, dtype=np.float64)
    if (pixels.ndim != 2 or pixels.shape[1] != 2 or pixels.shape != xy_mm.shape
            or len(pixels) < 6 or not np.isfinite(pixels).all()
            or not np.isfinite(xy_mm).all() or not np.isfinite(plane_z_mm)):
        raise ValueError('At least six finite pixel/XY pairs are required')
    for points in (pixels, xy_mm):
        if len(np.unique(points, axis=0)) != len(points):
            raise ValueError('Duplicate calibration points')
        singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
        if singular[1] <= singular[0] * 1e-3:
            raise ValueError('Calibration points are collinear or nearly collinear')

    # Centre the plane coordinates for conditioning; restore its origin after PnP.
    origin = xy_mm.mean(axis=0) / 1000.0
    objects = np.column_stack((xy_mm / 1000.0 - origin, np.zeros(len(xy_mm))))
    result = cv2.solvePnPGeneric(objects, pixels, matrix, distortion,
                                 flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        rvec, tvec = cv2.solvePnPRefineLM(objects, pixels, matrix, distortion,
                                         rvec.copy(), tvec.copy())
        camera_from_plane, _ = cv2.Rodrigues(rvec)
        depths = (objects @ camera_from_plane.T + tvec.reshape(3))[:, 2]
        rotation = camera_from_plane.T
        translation = -rotation @ tvec.reshape(3)
        if np.any(depths <= 0) or translation[2] <= 0:
            continue
        projected, _ = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation + [origin[0], origin[1], plane_z_mm / 1000.0]
        candidates.append((float(np.sqrt(np.mean(errors ** 2))), transform, errors))
    if not candidates:
        raise ValueError('No valid pose with camera above the plane and points in front')
    _, transform, errors = min(candidates, key=lambda item: item[0])
    return transform, errors


def pixel_to_plane(pixel, matrix, distortion, target_from_camera, plane_z_m):
    """Intersect an undistorted optical ray with target-frame Z = plane_z_m."""
    pixel = np.asarray(pixel, dtype=np.float64)
    transform = np.asarray(target_from_camera, dtype=np.float64)
    if (pixel.shape != (2,) or transform.shape != (4, 4)
            or not np.isfinite(pixel).all() or not np.isfinite(transform).all()
            or not np.isfinite(plane_z_m)):
        raise ValueError('Invalid pixel, transform or plane height')
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0)):
        raise ValueError('Expected a rigid transform')
    normalized = cv2.undistortPoints(pixel.reshape(1, 1, 2), matrix, distortion)[0, 0]
    direction = rotation @ np.array([normalized[0], normalized[1], 1.0])
    origin = transform[:3, 3]
    if abs(direction[2]) < 1e-9:
        raise ValueError('Ray is parallel to the target plane')
    distance = (plane_z_m - origin[2]) / direction[2]
    if distance <= 0:
        raise ValueError('Target plane is behind the camera')
    return origin + distance * direction
