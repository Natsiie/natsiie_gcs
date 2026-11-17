"""Stamped, numbered ring observations shared by both detection nodes."""

import copy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, PointCloud


def ring_centers_message(header, result, image_size, camera_frame):
    out = PointCloud()
    out.header = copy.deepcopy(header)
    if not out.header.frame_id:
        out.header.frame_id = camera_frame
    if result.detected and len(result.centers_px) == 3:
        out.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in result.centers_px]
    count = len(out.points)
    out.channels = [
        ChannelFloat32(name='ring_id', values=[float(i + 1) for i in range(count)]),
        ChannelFloat32(name='image_width', values=[float(image_size[0])] * count),
        ChannelFloat32(name='image_height', values=[float(image_size[1])] * count),
        ChannelFloat32(name='stable', values=[float(result.stable)]),
        ChannelFloat32(name='within_tolerance', values=[float(result.within_tolerance)]),
    ]
    return out
