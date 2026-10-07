import unittest

from identify_pack.scripts.pick_task import PickTask, parse_task


class PickTaskTests(unittest.TestCase):
    def test_example_batch_colors_and_ring_mapping(self):
        plan = parse_task('156+123+516+231')
        self.assertEqual([(p.sequence, p.batch, p.color, p.ring) for p in plan],
                         [(1, 1, 1, 1), (2, 1, 5, 2), (3, 1, 6, 3),
                          (4, 2, 5, 2), (5, 2, 1, 3), (6, 2, 6, 1)])

    def test_invalid_task_is_rejected(self):
        for code in ('156', '156+123+516', '156+123+516+231x',
                     '156+124+516+231', '156+113+516+231',
                     '176+123+516+231', '１５６+123+516+231'):
            with self.subTest(code=code), self.assertRaises(ValueError):
                parse_task(code)

    def test_completion_controls_sequence_and_between_batch_wait(self):
        task = PickTask()
        self.assertFalse(task.accepts(1, 20))
        self.assertTrue(task.load('156+123+516+231', 10))
        for i, expected in enumerate((1, 5, 6)):
            self.assertFalse(task.accepts(2, 100 + i))
            self.assertTrue(task.accepts(expected, 100 + i))
            task.sent(100 + i)
            self.assertFalse(task.accepts(expected, 200 + i))
            self.assertFalse(task.complete(i + 2, 200 + i))
            self.assertTrue(task.complete(i + 1, 100 + i))
            self.assertFalse(task.complete(i + 1, 200 + i))
        self.assertEqual(task.state, 'WAIT_NEXT_BATCH')
        self.assertFalse(task.accepts(5, 500))
        self.assertFalse(task.next_batch(2, 500))
        self.assertTrue(task.next_batch(1, 500))
        self.assertFalse(task.accepts(5, 499))
        for seq, color in ((4, 5), (5, 1), (6, 6)):
            self.assertTrue(task.accepts(color, 600 + seq))
            task.sent(600 + seq)
            self.assertTrue(task.complete(seq, 600 + seq))
        self.assertEqual(task.state, 'DONE')
        self.assertIsNone(task.expected)
        self.assertFalse(task.accepts(6, 999))

    def test_repeated_qr_does_not_reset_progress(self):
        task = PickTask()
        task.load('156+123+516+231', 10)
        task.sent(11)
        task.complete(1, 12)
        self.assertFalse(task.load('156+123+516+231', 20))
        self.assertEqual(task.expected.color, 5)
        with self.assertRaises(ValueError):
            task.load('123+123+456+231', 30)
        self.assertEqual(task.expected.color, 5)
