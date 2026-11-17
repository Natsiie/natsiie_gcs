"""Check gripper-plane pose recovery and multi-position GUI sampling."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from identify_pack.scripts.calibration import calibrate_camera_extrinsics_arm as arm


class ArmExtrinsicsTests(unittest.TestCase):
    def setUp(self):
        self.matrix = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
        self.distortion = np.array([0.02, -0.01, 0.001, -0.002, 0.0])
        self.objects = np.array([[x, 0., z] for x in (100., 160., 220.)
                                 for z in (-100., -40., 20.)])
        self.rotation = cv2.Rodrigues(np.array([0.3, -0.2, 0.1]))[0]
        self.translation = np.array([-0.16, 0.02, 0.7])
        self.pixels = cv2.projectPoints(
            self.objects / 1000., cv2.Rodrigues(self.rotation)[0], self.translation,
            self.matrix, self.distortion)[0].reshape(-1, 2)

    def test_recovers_camera_pose_direction_and_units(self):
        transform, rms, errors = arm.solve_camera_transform(
            self.objects, self.pixels, self.matrix, self.distortion)
        expected = np.eye(4)
        expected[:3, :3] = self.rotation.T
        expected[:3, 3] = -self.rotation.T @ self.translation
        np.testing.assert_allclose(transform, expected, atol=1e-6)
        self.assertLess(rms, 1e-6)
        self.assertLess(max(errors), 1e-6)

    def test_result_records_explicit_verification_plane(self):
        samples = [{'xyz_mm': point.tolist(), 'pixel': pixel.tolist()}
                   for point, pixel in zip(self.objects, self.pixels)]
        result = arm.build_result(samples, self.matrix, self.distortion, (640, 480), -15.)
        self.assertEqual(result['calibration_plane_z_mm'], -15.)
        self.assertEqual(result['parent_frame'], 'gimbal_link')
        self.assertEqual(result['calibration_method'], 'gripper_xz')
        self.assertAlmostEqual(np.linalg.norm(result['quaternion_xyzw']), 1.)
        with self.assertRaises(ValueError):
            arm.build_result(samples, self.matrix, self.distortion, (640, 480), np.nan)

    def test_rejects_line_and_near_line_in_either_coordinate_system(self):
        for epsilon in (0., 1e-5):
            line = np.column_stack((np.arange(9.), epsilon * np.arange(9.) ** 2))
            objects = self.objects.copy()
            objects[:, [0, 2]] = line
            with self.subTest(epsilon=epsilon, coordinates='objects'):
                with self.assertRaisesRegex(ValueError, '共线'):
                    arm.solve_camera_transform(objects, self.pixels, self.matrix, self.distortion)
            with self.subTest(epsilon=epsilon, coordinates='pixels'):
                with self.assertRaisesRegex(ValueError, '共线'):
                    arm.solve_camera_transform(self.objects, line, self.matrix, self.distortion)

    def test_rejects_duplicates_invalid_shape_and_nonzero_y(self):
        duplicate_objects = self.objects.copy()
        duplicate_objects[1] = duplicate_objects[0]
        duplicate_pixels = self.pixels.copy()
        duplicate_pixels[1] = duplicate_pixels[0]
        nonzero_y = self.objects.copy()
        nonzero_y[:, 1] = 5.
        invalid = self.objects.copy()
        invalid[0, 0] = np.nan
        for objects, pixels in ((duplicate_objects, self.pixels),
                                (self.objects, duplicate_pixels),
                                (nonzero_y, self.pixels), (invalid, self.pixels),
                                (self.objects[:5], self.pixels[:5]),
                                (self.objects, self.pixels[:, :1])):
            with self.subTest(objects=objects, pixels=pixels):
                with self.assertRaises(ValueError):
                    arm.solve_camera_transform(objects, pixels, self.matrix, self.distortion)

    def test_reported_narrow_pixel_distribution_is_rejected(self):
        objects = [[65, 0, 65], [220, 0, 65], [75, 0, 65],
                   [185, 0, 190], [155, 0, 190], [148, 0, 190]]
        pixels = [[321, 241], [318, 144], [329, 397],
                  [305, 98], [318, 303], [329, 343]]
        with self.assertRaisesRegex(ValueError, '二维展开比例 3.6%'):
            arm.solve_camera_transform(objects, pixels, self.matrix, self.distortion)

    def test_sampling_resumes_ignores_mouse_motion_and_undoes(self):
        args = SimpleNamespace(device='/dev/test_camera', width=640, height=480)
        camera = Mock()
        camera.isOpened.return_value = True
        frame_count = 0

        def read_frame():
            nonlocal frame_count
            frame_count += 1
            return True, np.full((480, 640, 3), frame_count, dtype=np.uint8)

        camera.read.side_effect = read_frame
        callbacks = []
        displayed = []
        actions = [ord('s')]
        inputs = []
        for index, (point, pixel) in enumerate(zip(self.objects, self.pixels)):
            u, v = np.rint(pixel).astype(int)
            actions.append((cv2.EVENT_LBUTTONDOWN, u, v))  # Live clicks are ignored.
            actions.append(ord(' '))
            actions.append((cv2.EVENT_MOUSEMOVE, u, v))
            actions.append((cv2.EVENT_RBUTTONDOWN, u, v))
            actions.append((cv2.EVENT_LBUTTONDOWN, u, v))
            inputs.append(f'{point[0]} {point[2]}')
            if index == 0:
                actions.append(ord('u'))
                actions.append((cv2.EVENT_LBUTTONDOWN, u, v))
                inputs.append(f'{point[0]} {point[2]}')
            actions.append(ord(' '))
        actions.append(ord('s'))  # Saving is allowed in live mode.
        action_iterator = iter(actions)

        def wait_key(_delay):
            action = next(action_iterator)
            if isinstance(action, tuple):
                callbacks[0](*action, 0, None)
                return -1
            return action

        with patch.object(arm.cv2, 'VideoCapture', return_value=camera), \
                patch.object(arm.cv2, 'namedWindow'), \
                patch.object(arm.cv2, 'setMouseCallback', side_effect=lambda _, cb: callbacks.append(cb)), \
                patch.object(arm.cv2, 'imshow', side_effect=lambda _, frame: displayed.append(int(frame[0, 0, 0]))), \
                patch.object(arm.cv2, 'waitKey', side_effect=wait_key), \
                patch.object(arm.cv2, 'destroyAllWindows') as destroy, \
                patch('builtins.input', side_effect=inputs) as input_mock, \
                patch('builtins.print'):
            samples = arm.collect_samples(args, self.matrix, self.distortion, [])
        self.assertEqual(len(samples), 9)
        self.assertEqual(input_mock.call_count, 10)
        np.testing.assert_allclose([sample['xyz_mm'] for sample in samples], self.objects)
        self.assertEqual(displayed[2], displayed[3])  # Freeze shows the exact frame clicked.
        self.assertGreater(len(set(displayed)), 9)  # Live frames resume between positions.
        self.assertEqual(camera.read.call_count, len(actions))
        camera.release.assert_called_once()
        destroy.assert_called_once()

    def test_quit_releases_camera(self):
        args = SimpleNamespace(device='0', width=640, height=480)
        camera = Mock()
        camera.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        with patch.object(arm.cv2, 'VideoCapture', return_value=camera), \
                patch.object(arm.cv2, 'namedWindow'), \
                patch.object(arm.cv2, 'setMouseCallback'), \
                patch.object(arm.cv2, 'imshow'), \
                patch.object(arm.cv2, 'waitKey', return_value=ord('q')), \
                patch.object(arm.cv2, 'destroyAllWindows'), patch('builtins.print'):
            self.assertIsNone(arm.collect_samples(args, self.matrix, self.distortion, []))
        camera.release.assert_called_once()


if __name__ == '__main__':
    unittest.main()
