import itertools
import math
import random
import subprocess
import sys
import unittest

from identify_pack.scripts.motion import TurntableMotionTracker


class MotionTests(unittest.TestCase):
    def test_assignment_preserves_minimum_distance_matching(self):
        rng = random.Random(42)
        for _ in range(20):
            previous = [(i % 2, rng.random() * 20, rng.random() * 20) for i in range(5)]
            current = [(i % 2, rng.random() * 20, rng.random() * 20) for i in range(5)]
            distances = []
            for order in itertools.permutations(range(5)):
                if any(previous[i][0] != current[j][0] for i, j in enumerate(order)):
                    continue
                candidate = [math.hypot(previous[i][1] - current[j][1],
                                        previous[i][2] - current[j][2])
                             for i, j in enumerate(order)]
                distances.append(candidate)
            expected = max(min(distances, key=sum)) / 0.1
            actual = TurntableMotionTracker._match_speed(previous, current, 0.1, 80.)
            self.assertAlmostEqual(actual, expected)

    def test_impossible_assignment_returns_none(self):
        for current in ([(2, 0., 0.), (2, 1., 0.)],
                        [(1, 100., 0.), (1, 101., 0.)],
                        [(1, 0., 0.), (1, 100., 0.)]):
            self.assertIsNone(TurntableMotionTracker._match_speed(
                [(1, 0., 0.), (1, 1., 0.)], current, 0.1, 80.))

    def test_many_blobs_finish_without_factorial_search(self):
        code = '''
from identify_pack.scripts.motion import TurntableMotionTracker
previous = [(1, i * 10., 0.) for i in range(24)]
current = [(1, i * 10. + 1., 0.) for i in reversed(range(24))]
tracker = TurntableMotionTracker()
tracker.update(previous, 10.)
result = tracker.update(current, 10.1)
assert abs(result['speed'] - 10.) < 1e-6, result
'''
        subprocess.run([sys.executable, '-c', code], check=True, timeout=5)

    def test_motion_then_stop_state_is_preserved(self):
        tracker = TurntableMotionTracker(required_moving_frames=2, required_low_speed_frames=2)
        for index, x in enumerate((0., 5., 10., 10., 10.)):
            result = tracker.update([(1, x, 0.)], 10. + index * 0.1)
        self.assertTrue(result['ready'])
        self.assertEqual(result['state'], 'READY')


if __name__ == '__main__':
    unittest.main()
