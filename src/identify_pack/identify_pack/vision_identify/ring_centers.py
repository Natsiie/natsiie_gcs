"""Stamped, numbered ring observations shared by both detection nodes."""

import copy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, PointCloud


def ring_centers_message(header, result, image_size, camera_frame, digit_report=None):
    out = PointCloud()
    out.header = copy.deepcopy(header)
    if not out.header.frame_id:
        out.header.frame_id = camera_frame
    if digit_report is not None:
        if result.detected and digit_report.get('placement_mapping_valid', digit_report['complete_digit_mapping']):
            rings = sorted(digit_report['rings'], key=lambda ring: ring.get('ring_id', ring['digit']))
            centers = digit_report.get('centers_by_digit_px', {})
            out.points = [Point32(
                x=float(centers.get(str(ring.get('ring_id', ring['digit'])), ring['center_px'])[0]),
                y=float(centers.get(str(ring.get('ring_id', ring['digit'])), ring['center_px'])[1]), z=0.0)
                for ring in rings]
    elif result.detected and len(result.centers_px) == 3:
        out.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in result.centers_px]
    count = len(out.points)
    out.channels = [
        ChannelFloat32(name='ring_id', values=[float(i + 1) for i in range(count)]),
        ChannelFloat32(name='image_width', values=[float(image_size[0])] * count),
        ChannelFloat32(name='image_height', values=[float(image_size[1])] * count),
        ChannelFloat32(name='stable', values=[float(result.stable and (
            digit_report is None or digit_report['digit_mapping_stable']))]),
        ChannelFloat32(name='within_tolerance', values=[float(result.within_tolerance)]),
    ]
    if digit_report is not None:
        rings = sorted(digit_report['rings'], key=lambda ring: ring.get('ring_id', ring['digit']) or 0) if count else []
        out.channels.extend([
            ChannelFloat32(name='digit_mapping_valid', values=[float(
                digit_report.get('placement_mapping_valid', digit_report['complete_digit_mapping']))]),
            ChannelFloat32(name='digit_mapping_stable', values=[float(digit_report['digit_mapping_stable'])]),
            ChannelFloat32(name='digit_score', values=[ring['score'] for ring in rings]),
            ChannelFloat32(name='digit_margin', values=[ring['score_margin'] for ring in rings]),
            ChannelFloat32(name='digit_observed', values=[float(ring['digit'] or 0) for ring in rings]),
        ])
    return out
