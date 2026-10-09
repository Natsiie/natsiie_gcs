import json
from pathlib import Path
import tempfile
import unittest

from identify_pack.scripts.place_task import PlaceTask


class PlaceTaskTests(unittest.TestCase):
    def task(self):
        task = PlaceTask(timeout_sec=5.)
        task.load('156+231+516+312')
        return task

    def test_mapping_and_separate_progress_through_both_layers(self):
        task = self.task()
        heights = []
        commands = []
        for stage, batch in (('ring_station', 1), ('stacked_ring', 1),
                             ('ring_station', 2), ('stacked_ring', 2)):
            task.start(task.task_id, stage, batch)
            for index in range(3):
                pick = task.expected
                count = task.stack_count[pick.ring - 1] if stage == 'stacked_ring' else 0
                command = task.prepare((10., 20., -42.5 + count * 60.), count + 1,
                                       'gimbal_base', index)
                commands.append(command)
                self.assertFalse(task.finish('old-task', command['command_id'], True))
                self.assertFalse(task.finish(task.task_id, command['command_id'] + 1, True))
                self.assertEqual(task.progress[stage][batch - 1], index)
                if stage == 'stacked_ring':
                    heights.append(command['z_mm'])
                self.assertTrue(task.finish(task.task_id, command['command_id'], True))
                self.assertFalse(task.finish(task.task_id, command['command_id'], True))
        self.assertEqual(task.state, 'DONE')
        self.assertEqual(task.stack_count, [2, 2, 2])
        self.assertEqual(heights, [-42.5] * 3 + [17.5] * 3)
        self.assertEqual([(c['color'], c['ring']) for c in commands[:3]], [(1, 2), (5, 3), (6, 1)])
        self.assertEqual([(c['color'], c['ring']) for c in commands[6:9]], [(5, 3), (1, 1), (6, 2)])

    def test_phase_order_and_duplicate_qr_do_not_reset_progress(self):
        task = self.task()
        with self.assertRaises(ValueError):
            task.start(task.task_id, 'stacked_ring', 2)
        task.start(task.task_id, 'ring_station', 1)
        command = task.prepare((1., 2., 3.), 1, 'base', 1.)
        self.assertFalse(task.load(task.code))
        with self.assertRaises(ValueError):
            task.start(task.task_id, 'ring_station', 1)
        with self.assertRaises(ValueError):
            task.prepare((4., 5., 6.), 1, 'base', 2.)
        self.assertEqual(task.pending, command)
        with self.assertRaises(ValueError):
            task.finish(task.task_id, command['command_id'], 'false')

    def test_failure_timeout_retry_and_late_confirmation(self):
        task = self.task()
        task.start(task.task_id, 'ring_station', 1)
        command = task.prepare((1., 2., 3.), 1, 'base', 10.)
        self.assertFalse(task.check_timeout(14.9))
        self.assertTrue(task.check_timeout(15.))
        self.assertEqual(task.state, 'ERROR')
        self.assertEqual(task.progress['ring_station'], [0, 0])
        self.assertEqual(task.retry(task.task_id, command['command_id'], 20.), command)
        self.assertTrue(task.finish(task.task_id, command['command_id'], False))
        self.assertEqual(task.pending, command)
        self.assertTrue(task.finish(task.task_id, command['command_id'], True))
        self.assertEqual(task.progress['ring_station'], [1, 0])

    def test_restart_retains_pending_and_never_automatically_resends(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'progress.json'
            task = self.task()
            task.start(task.task_id, 'ring_station', 1)
            command = task.prepare((1., 2., 3.), 1, 'base', 1.)
            task.save(path)
            restored = PlaceTask.restore(path)
            self.assertEqual(restored.state, 'ERROR')
            self.assertEqual(restored.pending, command)
            self.assertTrue(restored.finish(task.task_id, command['command_id'], True))
            restored.save(path)
            ready = PlaceTask.restore(path)
            self.assertEqual(ready.state, 'IDLE')
            ready.start(ready.task_id, 'ring_station', 1)
            self.assertEqual(ready.expected.ring, 3)

    def test_corrupt_checkpoint_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'progress.json'
            task = self.task()
            task.save(path)
            data = json.loads(path.read_text())
            data['stack_count'] = [1, 0, 0]
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                PlaceTask.restore(path)

    def test_stacking_only_first_batch_is_base_and_second_is_layer_two(self):
        task = PlaceTask(phases=('stacked_ring:1', 'stacked_ring:2'))
        task.load('156+231+516+312')
        for batch in (1, 2):
            task.start(task.task_id, 'stacked_ring', batch)
            for index in range(3):
                ring = task.expected.ring
                layer = task.stack_count[ring - 1] + 1
                self.assertEqual(layer, batch)
                command = task.prepare((0., 0., -42.5 + (layer - 1) * 60), layer, 'base', index)
                task.finish(task.task_id, command['command_id'], True)
        self.assertEqual(task.state, 'DONE')
        self.assertEqual(task.stack_count, [2, 2, 2])


if __name__ == '__main__':
    unittest.main()
