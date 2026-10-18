"""Synthetic top faces, projected frustum structure and shadow distractors."""

from dataclasses import replace
import unittest

import cv2
import numpy as np

from identify_pack.scripts.color_config import build_default_color_config
from identify_pack.scripts.color_tracking import ColorMultiTracker, detect_color_objects
from identify_pack.scripts.material_top_detector import MaterialTopConfig, detect_material_tops


def background():
    return np.full((480, 640, 3), 240, np.uint8)


def top(image, center, axes=(30, 22), angle=0, color=(0, 0, 210)):
    cv2.ellipse(image, center, axes, angle, 0, 360, color, -1, cv2.LINE_AA)


class MaterialTopTests(unittest.TestCase):
    def setUp(self):
        self.colors = build_default_color_config()

    def detect(self, image, config=None, diagnostics=None):
        return detect_material_tops(image, self.colors, config, diagnostics)

    def test_all_six_colors_detected_from_top_centers(self):
        image = background()
        colors = ((0, 0, 210), (0, 210, 210), (120, 0, 0),
                  (0, 150, 0), (20, 20, 20), (210, 140, 140))
        for i, color in enumerate(colors):
            top(image, (65 + 100 * i, 240), color=color)
        result = self.detect(image)
        self.assertEqual([item.color_id for item in result], list(range(1, 7)))
        self.assertTrue(all(item.detector == 'material_top' for item in result))
        self.assertTrue(all(item.ellipse is not None and item.shape_score > 0 for item in result))
        np.testing.assert_allclose([item.center for item in result],
                                   [(65 + 100 * i, 240) for i in range(6)], atol=2)

    def test_irregular_black_shadow_is_not_promoted_from_hsv_blob(self):
        image = background()
        top(image, (160, 130), color=(20, 20, 20))
        polygon = np.array([[280, 300], [310, 280], [400, 290], [480, 330],
                            [430, 350], [350, 325], [300, 335]], np.int32)
        cv2.fillPoly(image, [polygon], (20, 20, 20))
        self.assertGreaterEqual(len(detect_color_objects(image, self.colors)), 2)
        result = self.detect(image)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].color_id, 5)
        np.testing.assert_allclose(result[0].center, (160, 130), atol=2)

    def test_top_with_narrow_shadow_tail_does_not_shift_center_to_tail(self):
        image = background()
        cv2.rectangle(image, (175, 126), (240, 134), (20, 20, 20), -1)
        top(image, (160, 130), color=(20, 20, 20))
        config = replace(MaterialTopConfig(), max_fit_error=0.1)
        result = self.detect(image, config)
        # A merged tail may make the contour invalid. Rejecting it is preferable
        # to emitting the shifted centroid that the blob detector would publish.
        self.assertTrue(all(np.linalg.norm(np.array(item.center) - (160, 130)) < 3 for item in result))

    def test_top_centers_remain_separate_for_one_two_or_three_objects(self):
        for count in (1, 2, 3):
            with self.subTest(count=count):
                image = background()
                for x in (100, 300, 500)[:count]:
                    top(image, (x, 200), color=(0, 0, 210))
                result = self.detect(image)
                self.assertEqual(len(result), count)
                # Inside and outside Canny edges must not duplicate one top.
                self.assertTrue(all(abs(item.center[1] - 200) < 2 for item in result))

    def test_rotated_oblique_top_and_clipped_or_flat_targets(self):
        image = background()
        top(image, (200, 200), axes=(35, 18), angle=37)
        top(image, (400, 200), axes=(45, 6))
        top(image, (5, 300))
        result = self.detect(image)
        self.assertEqual(len(result), 1)
        np.testing.assert_allclose(result[0].center, (200, 200), atol=2)

    def test_unknown_ambiguous_and_marker_tops_are_rejected(self):
        image = background()
        top(image, (100, 200), color=(140, 140, 140))  # gray, not black
        top(image, (300, 200), color=(170, 100, 220))  # pink marker
        colors = build_default_color_config()
        colors['colors'][1]['ranges'] = colors['colors'][0]['ranges']
        top(image, (500, 200))  # equally red and yellow under deliberately overlapping calibration
        diagnostics = []
        result = detect_material_tops(image, colors, diagnostics=diagnostics)
        self.assertEqual(result, [])
        self.assertTrue(any(item['reason'] == 'unknown_color' for item in diagnostics))

    def test_soft_dark_ellipse_is_rejected_by_edge_sharpness(self):
        image = background()
        top(image, (300, 240), axes=(50, 28), color=(20, 20, 20))
        image = cv2.GaussianBlur(image, (41, 41), 10)
        result = self.detect(image)
        self.assertEqual(result, [])

    def test_colored_rectangles_are_not_accepted_as_tops(self):
        image = background()
        cv2.rectangle(image, (100, 180), (160, 225), (0, 0, 210), -1)
        cv2.rectangle(image, (300, 180), (350, 220), (20, 20, 20), -1)
        self.assertEqual(self.detect(image), [])

    def test_size_range_and_invalid_options(self):
        image = background()
        top(image, (300, 240), axes=(45, 30))
        self.assertEqual(self.detect(image, replace(MaterialTopConfig(), max_major_axis_px=50)), [])
        for options in ({'min_aspect': 0}, {'canny_low': 120}, {'body_direction': 2},
                        {'min_minor_axis_px': 140}, {'min_color_margin': -0.1},
                        {'edge_close_kernel_px': 4}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                MaterialTopConfig(**options)

    def test_side_structure_rejects_isolated_top_only_when_enabled(self):
        image = background()
        top(image, (200, 130), axes=(30, 15))
        self.assertEqual(len(self.detect(image)), 1)
        diagnostics = []
        self.assertEqual(self.detect(image, replace(MaterialTopConfig(), body_direction=1), diagnostics), [])
        self.assertTrue(any(item['reason'] == 'body_structure' for item in diagnostics))

    def test_side_structure_accepts_projected_frustum(self):
        # The top and side share their outline at T-junctions. All colors,
        # including a darker black side, must retain the actual top center.
        shades = (((0, 0, 210), (0, 0, 120)), ((0, 210, 210), (0, 120, 120)),
                  ((120, 0, 0), (70, 0, 0)), ((0, 150, 0), (0, 70, 0)),
                  ((35, 35, 35), (5, 5, 5)), ((210, 140, 140), (160, 107, 107)))
        for direction in (1, -1):
            for color_id, (face, side) in enumerate(shades, 1):
                with self.subTest(direction=direction, color_id=color_id):
                    image = background()
                    end = 240 + direction * 100
                    body = np.array([[170, 240], [230, 240], [250, end], [150, end]], np.int32)
                    cv2.fillPoly(image, [body], side)
                    top(image, (200, 240), axes=(30, 15), color=face)
                    result = self.detect(image, replace(MaterialTopConfig(), body_direction=direction))
                    self.assertEqual(len(result), 1)
                    self.assertEqual(result[0].color_id, color_id)
                    np.testing.assert_allclose(result[0].center, (200, 240), atol=2)

    def test_tops_feed_tracker_and_lost_predictions_have_no_current_ellipse(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        for frame in range(3):
            image = background()
            top(image, (100 + frame * 3, 200))
            result = tracker.update(self.detect(image), 10 + frame * 0.1, (640, 480))
        self.assertEqual(result[0]['state'], 'confirmed')
        self.assertEqual(result[0]['detector'], 'material_top')
        self.assertIsNotNone(result[0]['top_ellipse_px'])
        lost = tracker.update([], 10.3, (640, 480))
        self.assertEqual(lost[0]['state'], 'lost')
        self.assertIsNone(lost[0]['top_ellipse_px'])


if __name__ == '__main__':
    unittest.main()
