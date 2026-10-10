"""Ground-plane homography and its calibration: pure-math round-trips."""

import unittest

import numpy as np

from identify_pack.scripts.calibrate_ground_homography import fit_homography
from identify_pack.scripts.ground_homography import (
    apply_homography, base_from_link_xy, link_from_base_xy, pixel_to_base_xy,
)


class GroundHomographyTests(unittest.TestCase):
    def setUp(self):
        # 已知「像素 -> gimbal_link 平面 (x,y) mm」单应，含轻微透视。
        self.homography = np.array([[0.42, 0.01, -120.0],
                                    [0.00, 0.41, -95.0],
                                    [0.0002, -0.0001, 1.0]])

    def test_yaw_rotations_are_inverse(self):
        for yaw in (-30.0, 0.0, 45.0, 90.0):
            xy = np.array([123.0, -45.0])
            np.testing.assert_allclose(
                base_from_link_xy(link_from_base_xy(xy, yaw), yaw), xy, atol=1e-9)

    def test_apply_homography_matches_manual_projection(self):
        pixel = [200.0, 150.0]
        expected = self.homography @ [pixel[0], pixel[1], 1.0]
        expected = expected[:2] / expected[2]
        np.testing.assert_allclose(apply_homography(pixel, self.homography), expected)

    def test_fit_recovers_known_homography_and_round_trips_to_base(self):
        rng = np.random.default_rng(0)
        samples = []
        for _ in range(10):
            yaw_deg = float(rng.uniform(-30, 30))
            xy_base = [float(rng.uniform(50, 300)), float(rng.uniform(-120, 120))]
            link = link_from_base_xy(xy_base, yaw_deg)
            projected = np.linalg.inv(self.homography) @ [link[0], link[1], 1.0]
            samples.append({'pixel': [projected[0] / projected[2],
                                      projected[1] / projected[2]],
                            'xy_base': xy_base, 'yaw_deg': yaw_deg})
        fitted, errors = fit_homography(samples)
        self.assertLess(float(np.sqrt(np.mean(errors ** 2))), 1e-3)
        # 单应差一个整体比例因子，逐点反投影到 base 才是真判据。
        for sample in samples:
            np.testing.assert_allclose(
                pixel_to_base_xy(sample['pixel'], fitted, sample['yaw_deg']),
                sample['xy_base'], atol=1e-2)

    def test_fit_rejects_collinear_points(self):
        samples = [{'pixel': [100.0 + i * 100.0, 100.0 + i * 100.0],
                    'xy_base': [i * 50.0, i * 50.0], 'yaw_deg': 0.0}
                   for i in range(6)]
        with self.assertRaisesRegex(ValueError, '共线'):
            fit_homography(samples)

    def test_fit_requires_six_points(self):
        samples = [{'pixel': [0.0, 0.0], 'xy_base': [0.0, 0.0], 'yaw_deg': 0.0}] * 5
        with self.assertRaisesRegex(ValueError, '至少需要 6 个点'):
            fit_homography(samples)


if __name__ == '__main__':
    unittest.main()
