"""Detection, identity association and track lifecycle on synthetic observations."""

import unittest

import cv2
import numpy as np

from identify_pack.scripts.color_config import build_default_color_config
from identify_pack.scripts.color_tracking import ColorDetection, ColorMultiTracker, detect_color_objects


def detection(x, y=100., color=1, area=900.):
    return ColorDetection(color, (float(x), float(y)), (x - 15., y - 15., 30., 30.), area, 0.9)


class ColorTrackingTests(unittest.TestCase):
    def test_detects_three_objects_without_fixed_roi_or_color_lock(self):
        image = np.full((480, 640, 3), 245, np.uint8)
        for x, color in zip((100, 300, 540), ((0, 0, 220), (0, 220, 220), (120, 0, 0))):
            cv2.circle(image, (x, 350), 30, color, -1)
        result = detect_color_objects(image, build_default_color_config())
        self.assertEqual([item.color_id for item in result], [1, 2, 3])
        np.testing.assert_allclose([item.center for item in result], [[100, 350], [300, 350], [540, 350]], atol=1)

    def test_blue_shadow_is_one_object_and_disjoint_light_blue_stays_separate(self):
        image = np.full((480, 640, 3), 245, np.uint8)
        cv2.rectangle(image, (100, 100), (129, 150), (120, 0, 0), -1)
        cv2.rectangle(image, (130, 100), (150, 150), (220, 150, 150), -1)
        cv2.rectangle(image, (300, 100), (350, 150), (220, 150, 150), -1)
        result = detect_color_objects(image, build_default_color_config())
        self.assertEqual([item.color_id for item in result], [3, 6])
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0].center[0], 125, delta=1)

    def test_pink_background_border_and_tiny_noise_are_excluded(self):
        image = np.full((480, 640, 3), 245, np.uint8)
        cv2.rectangle(image, (0, 0), (639, 100), (0, 0, 220), -1)
        cv2.circle(image, (100, 200), 30, (170, 100, 220), -1)
        cv2.circle(image, (300, 200), 3, (0, 220, 220), -1)
        self.assertEqual(detect_color_objects(image, build_default_color_config()), [])

    def test_three_tracks_keep_ids_when_detection_order_changes(self):
        tracker = ColorMultiTracker()
        ids = None
        for frame in range(8):
            objects = [detection(100 + frame * 3, color=1),
                       detection(300 - frame * 2, color=2), detection(500 + frame, color=3)]
            result = tracker.update(objects[::-1] if frame % 2 else objects, 10 + frame * 0.1, (640, 480))
            mapping = {item['color_id']: item['track_id'] for item in result}
            ids = mapping if ids is None else ids
            self.assertEqual(mapping, ids)
        self.assertTrue(all(item['confirmed'] and item['observed'] for item in result))
        self.assertGreater(result[0]['velocity_px_s'][0], 20)

    def test_same_color_crossing_tracks_use_velocity(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        for frame in range(9):
            result = tracker.update([detection(100 + frame * 10), detection(260 - frame * 10)],
                                    10 + frame * 0.1, (640, 480))
        # After both objects meet, order switches; identities follow their velocity.
        result = tracker.update([detection(170), detection(190)], 10.9, (640, 480))
        self.assertGreater(result[0]['center_px'][0], result[1]['center_px'][0])

    def test_occlusion_retains_id_but_prediction_is_explicit(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        tracker.update([detection(100)], 10., (640, 480))
        previous = tracker.update([detection(110)], 10.1, (640, 480))[0]
        lost = tracker.update([], 10.2, (640, 480))[0]
        self.assertEqual(lost['state'], 'lost')
        self.assertFalse(lost['observed'])
        self.assertFalse(lost['stationary'])
        self.assertIsNone(lost['measurement_center_px'])
        self.assertGreater(lost['center_px'][0], previous['center_px'][0])
        current = tracker.update([detection(130)], 10.3, (640, 480))[0]
        self.assertEqual(current['track_id'], previous['track_id'])
        self.assertTrue(current['observed'])
        self.assertEqual(tracker.update([], 10.9, (640, 480)), [])

    def test_one_removed_object_does_not_reset_other_ids(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        objects = [detection(100, color=1), detection(300, color=2), detection(500, color=3)]
        tracker.update(objects, 10., (640, 480))
        before = tracker.update(objects, 10.1, (640, 480))
        for frame in range(2, 9):
            result = tracker.update(objects[1:], 10 + frame * 0.1, (640, 480))
        self.assertEqual([item['track_id'] for item in result], [item['track_id'] for item in before[1:]])

    def test_tentative_noise_requires_consecutive_hits(self):
        tracker = ColorMultiTracker()
        original = tracker.update([detection(100)], 10., (640, 480))[0]
        self.assertEqual(original['state'], 'tentative')
        self.assertEqual(tracker.update([], 10.1, (640, 480)), [])
        new = tracker.update([detection(100)], 10.2, (640, 480))[0]
        self.assertNotEqual(original['track_id'], new['track_id'])

    def test_color_flicker_does_not_immediately_change_identity_or_color(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2)
        tracker.update([detection(100, color=3)], 10., (640, 480))
        before = tracker.update([detection(100, color=3)], 10.1, (640, 480))[0]
        result = tracker.update([detection(100, color=6)], 10.2, (640, 480))[0]
        self.assertEqual(result['track_id'], before['track_id'])
        self.assertEqual(result['color_id'], 3)
        self.assertAlmostEqual(result['color_consistency'], 2 / 3)

    def test_far_wrong_color_is_unmatched_and_creates_a_new_track(self):
        tracker = ColorMultiTracker(min_confirmed_hits=1)
        tracker.update([detection(100)], 10., (640, 480))
        result = tracker.update([detection(160, color=2)], 10.1, (640, 480))
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]['state'], 'lost')
        self.assertEqual(result[1]['color_id'], 2)

    def test_size_change_gap_and_reset_never_reuse_ids(self):
        tracker = ColorMultiTracker()
        first = tracker.update([detection(100)], 10., (640, 480))[0]['track_id']
        second = tracker.update([detection(100)], 10.1, (320, 240))[0]['track_id']
        third = tracker.update([detection(100)], 12., (320, 240))[0]['track_id']
        tracker.reset()
        fourth = tracker.update([detection(100)], 1., (320, 240))[0]['track_id']
        self.assertEqual(len({first, second, third, fourth}), 4)

    def test_duplicate_timestamp_and_invalid_detection_do_not_advance_tracks(self):
        tracker = ColorMultiTracker()
        tracker.update([detection(100)], 10., (640, 480))
        before = tracker.snapshot()
        for stamp, objects in ((10., [detection(110)]), (9., [detection(110)]),
                               (10.1, [detection(float('nan'))])):
            with self.assertRaises(ValueError):
                tracker.update(objects, stamp, (640, 480))
            self.assertEqual(tracker.snapshot(), before)

    def test_stationary_requires_observed_confirmed_history(self):
        tracker = ColorMultiTracker(min_confirmed_hits=2, stationary_frames=3)
        for frame in range(4):
            result = tracker.update([detection(100)], 10 + frame * 0.1, (640, 480))
        self.assertTrue(result[0]['stationary'])
        result = tracker.update([], 10.4, (640, 480))
        self.assertFalse(result[0]['stationary'])


if __name__ == '__main__':
    unittest.main()
