"""Confirmed placement progress, independent of pickup and ROS."""

import copy
import json
import math
import os
from pathlib import Path
import tempfile
import uuid

from identify_pack.scripts.pick_task import parse_task


STAGES = ('ring_station', 'stacked_ring')
DEFAULT_PHASES = ('ring_station:1', 'stacked_ring:1', 'ring_station:2', 'stacked_ring:2')


def positive_integer(value, name):
    if type(value) is not int or value <= 0:
        raise ValueError(f'{name} must be a positive integer')
    return value


class PlaceTask:
    def __init__(self, timeout_sec=30.0, phases=DEFAULT_PHASES):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('Placement timeout must be finite and positive')
        self.timeout_sec = timeout_sec
        self.phases = tuple(phases)
        allowed = set(DEFAULT_PHASES)
        if (not self.phases or len(set(self.phases)) != len(self.phases)
                or any(phase not in allowed for phase in self.phases)
                or any(f'{stage}:2' in self.phases and (
                    f'{stage}:1' not in self.phases
                    or self.phases.index(f'{stage}:1') > self.phases.index(f'{stage}:2'))
                       for stage in STAGES)):
            raise ValueError('Phases must be unique; batch two requires an earlier batch one')
        self.task_id = None
        self.code = None
        self.plan = ()
        self.state = 'WAIT_TASK'
        self.stage = None
        self.batch = None
        self.progress = {stage: [0, 0] for stage in STAGES}
        self.stack_count = [0, 0, 0]
        self.command_counter = 0
        self.pending = None
        self.sent_at = None
        self.detail = ''

    def load(self, code):
        plan = parse_task(code)
        if self.code is not None:
            if code != self.code:
                raise ValueError('Another task is already loaded')
            return False
        self.code, self.plan = code, plan
        self.task_id = uuid.uuid4().hex
        self.state = 'IDLE'
        return True

    @property
    def expected(self):
        if self.stage is None:
            return None
        index = self.progress[self.stage][self.batch - 1]
        if index == 3:
            return None
        return self.plan[(self.batch - 1) * 3 + index]

    def next_phase(self):
        for phase in self.phases:
            stage, batch_text = phase.split(':')
            batch = int(batch_text)
            if self.progress[stage][batch - 1] < 3:
                return stage, batch
        return None

    def start(self, task_id, stage, batch):
        if task_id != self.task_id or self.task_id is None:
            raise ValueError('Task ID does not match')
        positive_integer(batch, 'batch')
        if self.state != 'IDLE' or (stage, batch) != self.next_phase():
            raise ValueError('Start requires the next unfinished phase and no active command')
        self.stage, self.batch = stage, batch
        self.state = 'WAIT_TARGET'
        self.detail = ''

    def prepare(self, xyz_mm, layer, frame_id, now):
        if self.state != 'WAIT_TARGET' or self.expected is None:
            raise ValueError('No placement target is expected')
        if len(xyz_mm) != 3 or not all(math.isfinite(v) for v in xyz_mm):
            raise ValueError('Placement coordinates must be finite XYZ millimetres')
        expected_layer = self.stack_count[self.expected.ring - 1] + 1 if self.stage == 'stacked_ring' else 1
        if type(layer) is not int or layer != expected_layer or not frame_id:
            raise ValueError('Placement layer or base frame does not match')
        self.command_counter += 1
        pick = self.expected
        self.pending = {
            'task_id': self.task_id, 'command_id': self.command_counter,
            'stage': self.stage, 'batch': self.batch, 'sequence': pick.sequence,
            'color': pick.color, 'ring': pick.ring, 'layer': layer,
            'frame_id': frame_id, 'x_mm': float(xyz_mm[0]),
            'y_mm': float(xyz_mm[1]), 'z_mm': float(xyz_mm[2]),
        }
        self.sent_at = now
        self.state = 'WAIT_PLACE_DONE'
        return copy.deepcopy(self.pending)

    def finish(self, task_id, command_id, success):
        positive_integer(command_id, 'command_id')
        if type(success) is not bool:
            raise ValueError('success must be a boolean')
        if (self.pending is None or task_id != self.task_id
                or command_id != self.pending['command_id']
                or self.state not in ('WAIT_PLACE_DONE', 'ERROR')):
            return False
        if not success:
            self.state = 'ERROR'
            self.detail = 'Placement failed; progress retained'
            return True
        if self.stage == 'stacked_ring':
            self.stack_count[self.pending['ring'] - 1] += 1
        self.progress[self.stage][self.batch - 1] += 1
        self.pending = None
        self.sent_at = None
        self.detail = ''
        if self.progress[self.stage][self.batch - 1] == 3:
            self.stage = self.batch = None
            self.state = 'DONE' if self.next_phase() is None else 'IDLE'
        else:
            self.state = 'WAIT_TARGET'
        return True

    def check_timeout(self, now):
        if (self.state == 'WAIT_PLACE_DONE' and self.sent_at is not None
                and now - self.sent_at >= self.timeout_sec):
            self.state = 'ERROR'
            self.detail = 'Placement confirmation timed out; completion is unknown'
            return True
        return False

    def retry(self, task_id, command_id, now):
        positive_integer(command_id, 'command_id')
        if (self.state != 'ERROR' or self.pending is None
                or task_id != self.task_id or command_id != self.pending['command_id']):
            raise ValueError('Retry requires the retained failed command')
        self.state = 'WAIT_PLACE_DONE'
        self.sent_at = now
        self.detail = ''
        return copy.deepcopy(self.pending)

    def status(self):
        pick = self.expected
        return copy.deepcopy({
            'task_id': self.task_id, 'task_code': self.code, 'state': self.state,
            'stage': self.stage, 'batch': self.batch,
            'place_index': self.progress[self.stage][self.batch - 1] if self.stage else None,
            'sequence': pick.sequence if pick else None,
            'color': pick.color if pick else None, 'ring': pick.ring if pick else None,
            'progress': self.progress, 'stack_count': self.stack_count,
            'command_counter': self.command_counter, 'pending_command': self.pending,
            'next_phase': self.next_phase() if self.code else None, 'detail': self.detail,
            'phases': self.phases,
        })

    def save(self, path):
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        name = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent,
                                             encoding='utf-8', delete=False) as stream:
                name = stream.name
                json.dump({'version': 1, **self.status()}, stream, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, destination)
        finally:
            if name is not None and os.path.exists(name):
                os.unlink(name)

    @classmethod
    def restore(cls, path, timeout_sec=30.0, phases=DEFAULT_PHASES):
        data = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        if data['version'] != 1 or not data['task_code']:
            raise ValueError('Invalid placement checkpoint')
        task = cls(timeout_sec, phases)
        if list(task.phases) != data['phases']:
            raise ValueError('Configured phases differ from the saved task')
        task.load(data['task_code'])
        if not isinstance(data['task_id'], str) or len(data['task_id']) != 32:
            raise ValueError('Invalid checkpoint task ID')
        task.task_id = data['task_id']
        for stage in STAGES:
            counts = data['progress'][stage]
            if len(counts) != 2 or any(type(n) is not int or not 0 <= n <= 3 for n in counts):
                raise ValueError('Invalid placement progress')
        task.progress = data['progress']
        phases = []
        for phase in DEFAULT_PHASES:
            stage, batch = phase.split(':')
            if phase not in task.phases and task.progress[stage][int(batch) - 1] != 0:
                raise ValueError('Unexpected completed phase')
        for phase in task.phases:
            stage, batch = phase.split(':')
            phases.append(task.progress[stage][int(batch) - 1])
        unfinished = False
        for count in phases:
            if unfinished and count != 0:
                raise ValueError('Checkpoint phases are out of order')
            unfinished = unfinished or count < 3
        expected_counts = [0, 0, 0]
        for batch in (0, 1):
            for pick in task.plan[batch * 3:batch * 3 + task.progress['stacked_ring'][batch]]:
                expected_counts[pick.ring - 1] += 1
        if data['stack_count'] != expected_counts:
            raise ValueError('Checkpoint stack counts disagree with completed placements')
        task.stack_count = expected_counts
        task.command_counter = data['command_counter']
        if type(task.command_counter) is not int or task.command_counter < sum(phases):
            raise ValueError('Invalid command counter')
        task.stage, task.batch = data['stage'], data['batch']
        task.state = data['state']
        if task.state not in ('IDLE', 'DONE', 'WAIT_TARGET', 'WAIT_PLACE_DONE', 'ERROR'):
            raise ValueError('Invalid checkpoint state')
        if task.state in ('IDLE', 'DONE'):
            if (task.stage is not None or task.batch is not None
                    or data['pending_command'] is not None
                    or (task.state == 'DONE') != (task.next_phase() is None)):
                raise ValueError('Invalid inactive checkpoint')
        elif (task.stage, task.batch) != task.next_phase():
            raise ValueError('Checkpoint active phase does not match progress')
        task.pending = data['pending_command']
        if task.pending is not None:
            pick = task.expected
            layer = task.stack_count[pick.ring - 1] + 1 if task.stage == 'stacked_ring' else 1
            expected = dict(task_id=task.task_id, command_id=task.command_counter,
                            stage=task.stage, batch=task.batch, sequence=pick.sequence,
                            color=pick.color, ring=pick.ring, layer=layer)
            if (task.state not in ('WAIT_PLACE_DONE', 'ERROR')
                    or any(task.pending.get(key) != value for key, value in expected.items())
                    or not task.pending.get('frame_id')
                    or not all(math.isfinite(task.pending[key]) for key in ('x_mm', 'y_mm', 'z_mm'))):
                raise ValueError('Invalid pending command')
            task.state = 'ERROR'
            task.detail = 'Restarted with an unconfirmed command; reconcile or retry explicitly'
        elif task.state in ('WAIT_PLACE_DONE', 'ERROR'):
            raise ValueError('Checkpoint is missing its pending command')
        elif task.state == 'WAIT_TARGET':
            task.state = 'IDLE'
            task.stage = task.batch = None
            task.detail = 'Restarted between placements; start the unfinished phase explicitly'
        return task
