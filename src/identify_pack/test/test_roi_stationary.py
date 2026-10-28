import unittest

from identify_pack.scripts.roi_stationary import RoiStationaryCounter


class RoiStationaryCounterTests(unittest.TestCase):
    def test_initial_stillness_counts_without_prior_motion(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=3)
        for index in range(3):
            result = counter.update((1, 100., 100.), 10. + index * .04)
        self.assertTrue(result['ready'])
        self.assertEqual(result['state'], 'READY')
        self.assertEqual(result['count'], 3)

    def test_seconds_are_converted_with_measured_fps(self):
        counter = RoiStationaryCounter(hold_sec=0.6)
        for index in range(40):
            result = counter.update((1, 100., 100.), 10. + index * .04)
        # 25 fps × 0.6 s = 15 帧
        self.assertEqual(result['required_frames'], 15)

    def test_fixed_fps_overrides_measurement(self):
        counter = RoiStationaryCounter(hold_sec=0.6, fixed_fps=10.)
        result = counter.update((1, 100., 100.), 10.)
        self.assertEqual(result['required_frames'], 6)

    def test_position_jump_restarts_count(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=3)
        for index, x in enumerate((100., 101., 102.)):
            result = counter.update((1, x, 100.), 10. + index * .04)
        self.assertTrue(result['ready'])
        result = counter.update((1, 140., 100.), 10.12)
        self.assertFalse(result['ready'])
        self.assertEqual(result['count'], 1)

    def test_two_missing_frames_hold_then_third_resets(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=3, miss_frames=2)
        for index in range(3):
            result = counter.update((1, 100., 100.), 10. + index * .04)
        self.assertTrue(result['ready'])
        for index in range(2):
            result = counter.update(None, 10.12 + index * .04)
            self.assertTrue(result['ready'])
            self.assertEqual(result['count'], 3)
            self.assertEqual(result['state'], 'MISSING')
        result = counter.update(None, 10.20)
        self.assertFalse(result['ready'])
        self.assertEqual(result['count'], 0)
        self.assertEqual(result['state'], 'WAITING')

    def test_grabbed_object_resets_for_next_one(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=2, miss_frames=1)
        for index in range(2):
            counter.update((1, 100., 100.), 10. + index * .04)
        counter.update(None, 10.08)
        result = counter.update(None, 10.12)
        self.assertFalse(result['ready'])
        self.assertEqual(result['count'], 0)
        result = counter.update((2, 100., 100.), 10.16)
        self.assertEqual(result['count'], 1)
        self.assertFalse(result['ready'])

    def test_color_change_restarts_count(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=3)
        counter.update((1, 100., 100.), 10.)
        result = counter.update((2, 100., 100.), 10.04)
        self.assertEqual(result['count'], 1)
        self.assertFalse(result['ready'])

    def test_long_frame_gap_restarts_count(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=3,
                                       max_frame_interval=0.2)
        counter.update((1, 100., 100.), 10.)
        result = counter.update((1, 100., 100.), 10.04)
        self.assertEqual(result['count'], 2)
        result = counter.update((1, 100., 100.), 11.)
        self.assertEqual(result['count'], 1)
        self.assertFalse(result['ready'])

    def test_invalid_timestamp_resets(self):
        counter = RoiStationaryCounter(hold_sec=0.0, hold_frames=2)
        counter.update((1, 100., 100.), 10.)
        result = counter.update((1, 100., 100.), 0.)
        self.assertEqual(result['count'], 0)
        self.assertFalse(result['ready'])


if __name__ == '__main__':
    unittest.main()
