import unittest

from identify_pack.scripts.arrival import GripperArrivalDetector

NS = 1_000_000_000


class ArrivalTests(unittest.TestCase):
    def test_absent_keeps_waiting(self):
        d = GripperArrivalDetector(min_confirm_sec=5.0)
        self.assertFalse(d.update(False, 0 * NS))
        self.assertFalse(d.arrived)
        self.assertIsNone(d.present_since_ns)

    def test_short_presence_does_not_trigger(self):
        d = GripperArrivalDetector(min_confirm_sec=5.0)
        self.assertFalse(d.update(True, 0 * NS))
        self.assertFalse(d.update(True, 4 * NS))
        self.assertFalse(d.arrived)

    def test_continuous_presence_triggers_once(self):
        d = GripperArrivalDetector(min_confirm_sec=5.0)
        self.assertFalse(d.update(True, 0 * NS))
        self.assertTrue(d.update(True, 5 * NS))
        self.assertTrue(d.arrived)
        self.assertFalse(d.update(True, 6 * NS))  # 锁存，不再重复触发

    def test_interruption_resets_timer(self):
        d = GripperArrivalDetector(min_confirm_sec=5.0)
        d.update(True, 0 * NS)
        d.update(True, 3 * NS)
        d.update(False, 4 * NS)            # 消失 -> 清零
        self.assertIsNone(d.present_since_ns)
        d.update(True, 5 * NS)             # 重新开始
        self.assertFalse(d.arrived)
        self.assertFalse(d.update(True, 9 * NS))   # 只累计了 4s
        self.assertTrue(d.update(True, 10 * NS))   # 满 5s

    def test_reset_allows_retrigger(self):
        d = GripperArrivalDetector(min_confirm_sec=5.0)
        d.update(True, 0 * NS)
        self.assertTrue(d.update(True, 5 * NS))
        d.reset()
        self.assertFalse(d.arrived)
        self.assertIsNone(d.present_since_ns)
        d.update(True, 20 * NS)
        self.assertTrue(d.update(True, 25 * NS))


if __name__ == '__main__':
    unittest.main()
