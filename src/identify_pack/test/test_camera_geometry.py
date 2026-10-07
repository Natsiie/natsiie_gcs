"""Check pose direction, units, distortion and rotating-frame projection."""

import unittest

import cv2
import numpy as np

from identify_pack.scripts.camera_geometry import estimate_camera_transform, pixel_to_plane
from identify_pack.scripts.calibrate_camera_extrinsics import build_result
from identify_pack.scripts.validate_camera_extrinsics import evaluate_point


class CameraGeometryTests(unittest.TestCase):
    def setUp(self):
        self.matrix = np.array([[308.18570888, 0, 287.91412584],
                                [0, 305.1869521, 237.64880181], [0, 0, 1]])
        self.distortion = np.array([-0.0183379299, -0.0479921394,
                                    -0.0000797842365, -0.0295371602, 0.166088446])
        self.plane_z = 0.04
        rotation = cv2.Rodrigues(np.array([0.10, -0.06, 0.12]))[0] @ np.diag([1, -1, -1])
        self.transform = np.eye(4)
        self.transform[:3, :3] = rotation
        self.transform[:3, 3] = [0.12, -0.03, 0.4]
        self.points = np.array([[x, y, self.plane_z]
                                for x in (-0.01, 0.1, 0.21)
                                for y in (-0.12, -0.01, 0.1)])
        inverse_rotation = rotation.T
        rvec = cv2.Rodrigues(inverse_rotation)[0]
        tvec = -inverse_rotation @ self.transform[:3, 3]
        self.pixels = cv2.projectPoints(self.points, rvec, tvec,
                                       self.matrix, self.distortion)[0].reshape(-1, 2)

    def test_extrinsics_recover_camera_pose_in_metres(self):
        transform, errors = estimate_camera_transform(
            self.pixels, self.points[:, :2] * 1000, self.plane_z * 1000,
            self.matrix, self.distortion)
        np.testing.assert_allclose(transform, self.transform, atol=1e-6)
        self.assertLess(max(errors), 1e-6)

    def test_distorted_pixel_projects_to_known_plane(self):
        for pixel, expected in zip(self.pixels, self.points):
            actual = pixel_to_plane(pixel, self.matrix, self.distortion,
                                    self.transform, self.plane_z)
            np.testing.assert_allclose(actual, expected, atol=1e-5)

    def test_rotated_gimbal_projects_into_base(self):
        base_from_gimbal = np.eye(4)
        base_from_gimbal[:3, :3] = cv2.Rodrigues(np.array([0.0, 0.0, np.pi / 2]))[0]
        base_from_gimbal[:3, 3] = [0.02, -0.01, 0.05]
        actual = pixel_to_plane(self.pixels[4], self.matrix, self.distortion,
                                base_from_gimbal @ self.transform, self.plane_z + 0.05)
        expected = (base_from_gimbal @ np.append(self.points[4], 1))[:3]
        np.testing.assert_allclose(actual, expected, atol=1e-5)

    def test_result_records_tf_direction_and_samples(self):
        samples = [{'pixel': pixel.tolist(), 'xy_mm': (point[:2] * 1000).tolist()}
                   for pixel, point in zip(self.pixels, self.points)]
        result = build_result(samples, self.plane_z * 1000, self.matrix,
                              self.distortion, (640, 480))
        self.assertEqual(result['parent_frame'], 'gimbal_link')
        self.assertEqual(result['child_frame'], 'camera_optical_frame')
        np.testing.assert_allclose(result['translation_m'], [0.12, -0.03, 0.4], atol=1e-6)
        self.assertAlmostEqual(np.linalg.norm(result['quaternion_xyzw']), 1.0)
        self.assertEqual(result['samples'], samples)

    def test_collinear_points_rejected(self):
        with self.assertRaisesRegex(ValueError, 'collinear'):
            estimate_camera_transform(self.pixels, [[index, 0] for index in range(9)],
                                      0, self.matrix, self.distortion)

    def test_independent_point_errors_in_millimetres(self):
        point = np.array([0.08, 0.03, 0.02])
        rotation = self.transform[:3, :3].T
        rvec = cv2.Rodrigues(rotation)[0]
        tvec = -rotation @ self.transform[:3, 3]
        pixel = cv2.projectPoints(point.reshape(1, 3), rvec, tvec,
                                  self.matrix, self.distortion)[0].reshape(2)
        calibration = {
            'camera_matrix': self.matrix.tolist(), 'dist_coeffs': self.distortion.tolist(),
            'parent_from_camera_matrix': self.transform.tolist(), 'image_size': [640, 480],
        }
        result = evaluate_point({'pixel': pixel.tolist(), 'xy_mm': [77.0, 34.0]},
                                calibration, 20.0)
        np.testing.assert_allclose(result['predicted_xyz_mm'], [80, 30, 20], atol=1e-4)
        self.assertAlmostEqual(result['dx_mm'], 3.0, places=4)
        self.assertAlmostEqual(result['dy_mm'], -4.0, places=4)
        self.assertAlmostEqual(result['xy_error_mm'], 5.0, places=4)

    def test_validation_rejects_out_of_bounds_pixel(self):
        calibration = {
            'camera_matrix': self.matrix.tolist(), 'dist_coeffs': self.distortion.tolist(),
            'parent_from_camera_matrix': self.transform.tolist(), 'image_size': [640, 480],
        }
        with self.assertRaisesRegex(ValueError, 'bounds'):
            evaluate_point({'pixel': [640, 0], 'xy_mm': [0, 0]}, calibration, 40)

    def test_plane_behind_camera_rejected(self):
        with self.assertRaisesRegex(ValueError, 'behind'):
            pixel_to_plane(self.pixels[4], self.matrix, self.distortion, self.transform, 1.0)

    def test_parallel_ray_rejected(self):
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(np.array([0.0, np.pi / 2, 0.0]))[0]
        with self.assertRaisesRegex(ValueError, 'parallel'):
            pixel_to_plane(self.matrix[:2, 2], self.matrix, self.distortion, transform, 0)


if __name__ == '__main__':
    unittest.main()
