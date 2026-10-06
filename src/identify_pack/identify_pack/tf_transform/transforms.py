"""Transform parameter validation and single-axis gimbal rotation."""

import json
import math
from pathlib import Path


def finite_vector(values, length, name):
    values = tuple(float(value) for value in values)
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError(f'{name} requires {length} finite values')
    return values


def normalize_quaternion(values):
    values = finite_vector(values, 4, 'quaternion_xyzw')
    norm = math.hypot(*values)
    if norm < 1e-12:
        raise ValueError('Quaternion cannot be zero')
    return tuple(value / norm for value in values)


def yaw_quaternion(angle_rad, sign=1.0, offset_rad=0.0):
    angle_rad, sign, offset_rad = finite_vector(
        [angle_rad, sign, offset_rad], 3, 'gimbal angle')
    if sign not in (-1.0, 1.0):
        raise ValueError('angle_sign must be +1 or -1')
    yaw = sign * angle_rad + offset_rad
    if not math.isfinite(yaw):
        raise ValueError('Adjusted angle must be finite')
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_camera_pose(path, parent_frame, child_frame):
    document = json.loads(Path(path).read_text(encoding='utf-8'))
    if (document['parent_frame'] != parent_frame
            or document['child_frame'] != child_frame):
        raise ValueError('Extrinsics frame names do not match configured frames')
    translation = finite_vector(document['translation_m'], 3, 'camera translation_m')
    quaternion = normalize_quaternion(document['quaternion_xyzw'])
    return translation, quaternion
