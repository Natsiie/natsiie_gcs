"""Task-code parsing and confirmed, ordered pickup state (no ROS or hardware)."""

import re
from dataclasses import dataclass


@dataclass(frozen=True)
class Pick:
    sequence: int
    batch: int
    color: int
    ring: int


def parse_task(text):
    if not re.fullmatch(r'[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}', text):
        raise ValueError('任务码需要四组三位数，例如 156+123+516+231')
    colors1, rings1, colors2, rings2 = text.split('+')
    for colors, rings in ((colors1, rings1), (colors2, rings2)):
        if set(rings) != set('123'):
            raise ValueError('放置环号必须是 1、2、3 的排列')
    return tuple(Pick(batch * 3 + index + 1, batch + 1, int(color), int(ring))
                 for batch, (colors, rings) in enumerate(((colors1, rings1), (colors2, rings2)))
                 for index, (color, ring) in enumerate(zip(colors, rings)))


class PickTask:
    def __init__(self, wait_between_batches=True):
        self.code = None
        self.picks = ()
        self.index = 0
        self.state = 'WAIT_TASK'
        self.wait_between_batches = wait_between_batches
        self.cutoff_ns = 0

    @property
    def expected(self):
        return self.picks[self.index] if self.index < len(self.picks) else None

    def load(self, text, now_ns):
        picks = parse_task(text)
        if self.code is not None:
            if self.code != text:
                raise ValueError('已有任务，忽略其他任务码；新任务请重启任务节点')
            return False
        self.code, self.picks = text, picks
        self.cutoff_ns = now_ns
        self.state = 'WAIT_MATCH'
        return True

    def accepts(self, color, stamp_ns):
        return (self.state == 'WAIT_MATCH' and stamp_ns > self.cutoff_ns
                and self.expected is not None and color == self.expected.color)

    def sent(self, stamp_ns):
        self.cutoff_ns = max(self.cutoff_ns, stamp_ns)
        self.state = 'WAIT_PICK_DONE'

    def complete(self, sequence, now_ns):
        if (self.state != 'WAIT_PICK_DONE' or self.expected is None
                or sequence != self.expected.sequence):
            return False
        self.index += 1
        self.cutoff_ns = now_ns
        if self.index == len(self.picks):
            self.state = 'DONE'
        elif self.index == 3 and self.wait_between_batches:
            self.state = 'WAIT_NEXT_BATCH'
        else:
            self.state = 'WAIT_MATCH'
        return True

    def next_batch(self, completed_batch, now_ns):
        if self.state != 'WAIT_NEXT_BATCH' or completed_batch != 1:
            return False
        self.cutoff_ns = now_ns
        self.state = 'WAIT_MATCH'
        return True
