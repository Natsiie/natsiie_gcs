#!/usr/bin/env python3
"""夹爪到位判定（纯逻辑，无 ROS / 无 GUI）。

粉色标记（夹爪舵机外壳）进入 ROI 并被连续识别满 ``min_confirm_sec`` 秒，
判定夹爪到位。粉色消失即清零连续计时（只认可「持续可见」），到位后锁存，
直到显式 ``reset()``（供下一次抓取复用）。
"""

from __future__ import annotations


class GripperArrivalDetector:
    """连续观察粉色标记是否在 ROI 内，达到时长阈值判定到位。"""

    def __init__(self, min_confirm_sec: float = 5.0):
        self.min_confirm_ns = int(min_confirm_sec * 1_000_000_000)
        self.reset()

    def reset(self) -> None:
        """清空连续计时与到位锁存（下一次抓取复用）。"""
        self.present_since_ns = None
        self.arrived = False

    def update(self, present: bool, now_ns: int) -> bool:
        """每帧喂入「粉色是否在 ROI 内」，返回是否触发到位（上升沿）。

        ``now_ns`` 为单调递增的纳秒时间戳（如 ``time.monotonic_ns()``）。
        只在首次跨过 ``min_confirm_sec`` 阈值的那一帧返回 True，之后锁存。
        """
        if self.arrived:
            return False
        if not present:
            self.present_since_ns = None
            return False
        if self.present_since_ns is None:
            self.present_since_ns = now_ns
        if now_ns - self.present_since_ns >= self.min_confirm_ns:
            self.arrived = True
            return True
        return False
