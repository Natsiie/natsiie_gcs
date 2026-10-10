"""Exercise actual ring images, polarity, affine crops and rejected mappings."""

from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from identify_pack.scripts import ring_station_digit_test as detector
from identify_pack.scripts.ring_station_detector import RingStationDetector
from identify_pack.scripts.ring_station_digits import (
    DigitMappingTracker, placement_plan, recognize_detection,
)


def station_image(digits=(3, 2, 1), dark=False):
    image = np.full((480, 640, 3), 20 if dark else 245, np.uint8)
    ink = (245, 245, 245) if dark else (20, 20, 20)
    for x, digit in zip((150, 320, 490), digits):
        for radius in (40, 45, 50, 55, 60, 65):
            cv2.circle(image, (x, 240), radius, ink, 2, cv2.LINE_AA)
        if digit is not None:
            (width, height), _ = cv2.getTextSize(str(digit), cv2.FONT_HERSHEY_SIMPLEX, 1.1, 2)
            cv2.putText(image, str(digit), (x - width // 2, 240 + height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, ink, 2, cv2.LINE_AA)
    return image


class RingStationDigitTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = detector.load_ring_station_config(
            detector.ROOT / 'ring_station_detector.json')
        self.templates, _ = detector.load_templates(Path(self.directory.name))

    def process(self, image):
        return detector.process_frame(image, self.config, self.templates)

    def test_printed_numbers_override_left_to_right_order_in_both_polarities(self):
        for dark in (False, True):
            with self.subTest(dark=dark):
                result, _ = self.process(station_image(dark=dark))
                self.assertTrue(result['three_ring_geometry'])
                self.assertTrue(result['complete_digit_mapping'])
                self.assertEqual([ring['digit'] for ring in result['rings']], [3, 2, 1])
                np.testing.assert_allclose([ring['center_px'] for ring in result['rings']],
                                           [[150, 240], [320, 240], [490, 240]], atol=2)

    def test_oblique_ellipses_keep_shape_and_read_digits(self):
        image = cv2.warpAffine(station_image(), np.array([[1., 0., 0.], [0., 0.75, 60.]]),
                               (640, 480), borderValue=(245, 245, 245))
        result, _ = self.process(image)
        self.assertTrue(result['complete_digit_mapping'])
        self.assertEqual([ring['digit'] for ring in result['rings']], [3, 2, 1])
        for ring in result['rings']:
            axes = ring['ellipse_axes_px']
            self.assertLess(min(axes) / max(axes), 0.85)

    def test_empty_center_is_unknown_and_is_not_completed_by_geometry(self):
        result, crops = self.process(station_image((3, None, 1)))
        self.assertTrue(result['three_ring_geometry'])
        self.assertFalse(result['complete_digit_mapping'])
        self.assertEqual([ring['digit'] for ring in result['rings']], [3, None, 1])
        self.assertIsNone(crops[1][1])

    def test_duplicate_numbers_are_not_a_complete_mapping(self):
        result, _ = self.process(station_image((2, 2, 1)))
        self.assertTrue(result['three_ring_geometry'])
        self.assertEqual([ring['digit'] for ring in result['rings']], [2, 2, 1])
        self.assertFalse(result['complete_digit_mapping'])

    def test_partial_and_blank_frames_do_not_reuse_old_centers(self):
        self.process(station_image())
        result, _ = self.process(station_image((1,)))
        self.assertFalse(result['three_ring_geometry'])
        self.assertFalse(result['complete_digit_mapping'])
        self.assertEqual(len(result['rings']), 1)
        result, _ = self.process(np.full((480, 640, 3), 245, np.uint8))
        self.assertEqual(result['rings'], [])

    def test_ambiguous_templates_and_clipped_center_are_unknown(self):
        _, crops = self.process(station_image())
        digit = crops[0][1]
        label, _, gap = detector.match_digit(digit, {1: [digit], 2: [digit], 3: [digit]})
        self.assertIsNone(label)
        self.assertEqual(gap, 0)
        patch, _ = crops[0]
        self.assertIsNone(detector.extract_digit(patch, np.zeros((160, 160), bool)))

    def test_captured_template_reload_overrides_only_its_label(self):
        _, crops = self.process(station_image())
        directory = Path(self.directory.name)
        self.assertTrue(cv2.imwrite(str(directory / 'digit_3_sample.png'), crops[0][1]))
        templates, sources = detector.load_templates(directory)
        self.assertEqual(sources, {1: 'builtin', 2: 'builtin', 3: 'captured'})
        label, score, _ = detector.match_digit(crops[0][1], templates)
        self.assertEqual(label, 3)
        self.assertGreater(score, 0.99)

    def test_task_code_resolves_measured_centers_by_digit(self):
        args = detector.parse_args(['--task-code', '156+123+516+231', '--batch', '2'])
        self.assertEqual(args.order, (2, 3, 1))
        report, _ = self.process(station_image())
        plan = placement_plan(report, args.order)
        self.assertEqual([item['slot'] for item in plan], [2, 1, 3])
        np.testing.assert_allclose([item['center_px'] for item in plan],
                                   [[320, 240], [150, 240], [490, 240]], atol=2)

    def test_stability_resets_on_loss_digit_reordering_and_motion(self):
        tracker = DigitMappingTracker(3, 4)
        for index in range(3):
            report, _ = self.process(station_image())
            tracker.update(report, (640, 480))
            self.assertEqual(report['digit_mapping_stable'], index == 2)
        report, _ = self.process(station_image((1, 2, 3)))
        tracker.update(report, (640, 480))
        self.assertFalse(report['digit_mapping_stable'])
        self.assertEqual(report['stable_frames'], 1)
        for ring in report['rings']:
            ring['center_px'][0] += 20
        tracker.update(report, (640, 480))
        self.assertEqual(report['stable_frames'], 1)
        report, _ = self.process(station_image((1, None, 3)))
        tracker.update(report, (640, 480))
        self.assertFalse(report['digit_mapping_stable'])
        self.assertEqual(report['centers_by_digit_px'], {})
        self.assertEqual(report['stable_frames'], 0)

    def test_fixed_left_to_right_numbering_handles_covered_digits(self):
        tracker = DigitMappingTracker(3, 4, expected_digits=(1, 2, 3))
        for _ in range(3):
            report, _ = self.process(station_image((1, None, 3)))
            tracker.update(report, (640, 480))
        self.assertFalse(report['complete_digit_mapping'])
        self.assertTrue(report['placement_mapping_valid'])
        self.assertTrue(report['digit_mapping_stable'])
        self.assertEqual([ring['digit'] for ring in report['rings']], [1, None, 3])
        self.assertEqual([ring['ring_id'] for ring in report['rings']], [1, 2, 3])
        plan = placement_plan(report, (2, 3, 1))
        np.testing.assert_allclose([item['center_px'] for item in plan],
                                   [[320, 240], [490, 240], [150, 240]], atol=2)
        report, _ = self.process(station_image((1,)))
        tracker.update(report, (640, 480))
        self.assertFalse(report['placement_mapping_valid'])
        self.assertFalse(report['digit_mapping_stable'])

    def test_fixed_order_conflicting_digit_rejects_mapping(self):
        tracker = DigitMappingTracker(3, 4, expected_digits=(1, 2, 3))
        for digits in ((3, 2, 1), (1, 1, 3)):
            report, _ = self.process(station_image(digits))
            tracker.update(report, (640, 480))
            self.assertTrue(report['numbering_conflict'])
            self.assertFalse(report['placement_mapping_valid'])
            self.assertEqual(placement_plan(report), [])

    def test_oblique_ros_geometry_keeps_ellipse_for_digit_rectification(self):
        image = cv2.warpAffine(station_image(), np.array([[1., 0., 0.], [0., .65, 84.]]),
                               (640, 480), borderValue=(245, 245, 245))
        geometry = RingStationDetector(self.config, use_hough=True)
        result = geometry.detect(image)
        report, _ = recognize_detection(image, result, self.templates)
        self.assertTrue(report['complete_digit_mapping'])
        self.assertEqual([ring['digit'] for ring in report['rings']], [3, 2, 1])
        self.assertLess(min(result.ellipses[1][1]) / max(result.ellipses[1][1]), .75)

    def test_base_template_names_and_missing_labels_in_production(self):
        report, crops = self.process(station_image())
        directory = Path(self.directory.name)
        for ring, (_, digit) in zip(report['rings'], crops):
            cv2.imwrite(str(directory / f"digit_{ring['digit']}.png"), digit)
        templates, sources = detector.load_templates(directory, allow_builtin=False)
        self.assertEqual(sources, {1: 'captured', 2: 'captured', 3: 'captured'})
        (directory / 'digit_3.png').unlink()
        with self.assertRaises(ValueError):
            detector.load_templates(directory, allow_builtin=False)

    def test_real_photo_not_used_for_the_selected_templates(self):
        snapshots = sorted((detector.ROOT / 'ring_station_digit_snapshots').glob('*_raw.png'))
        if len(snapshots) < 2:
            self.skipTest('Local real-camera snapshots are unavailable')
        templates, _ = detector.load_templates(detector.ROOT / 'ring_station_digit_templates',
                                                allow_builtin=False)
        # Exclude samples from the test photo; otherwise matching would be tautological.
        stem = snapshots[-1].stem.removesuffix('_raw')
        templates = {label: [cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
                             for path in sorted((detector.ROOT / 'ring_station_digit_templates').glob(
                                 f'digit_{label}_*.png')) if stem not in path.name]
                     for label in (1, 2, 3)}
        geometry = RingStationDetector(self.config, use_hough=True)
        image = cv2.imread(str(snapshots[-1]))
        for _ in range(self.config.stable_frame_count):
            result = geometry.detect(image)
        report, _ = recognize_detection(image, result, templates)
        self.assertTrue(result.stable)
        self.assertTrue(report['complete_digit_mapping'])
        self.assertEqual([ring['digit'] for ring in report['rings']], [1, 2, 3])


if __name__ == '__main__':
    unittest.main()
