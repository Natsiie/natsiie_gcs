#!/usr/bin/env python3
"""夹爪到位判定（纯逻辑，无 ROS / 无 GUI）。

粉色标记（夹爪舵机外壳）进入 ROI 并被连续识别满 ``min_confirm_sec`` 秒，
判定夹爪到位。粉色消失即清零连续计时（只认可「持续可见」），到位后锁存，
直到显式 ``reset()``（供下一次抓取复用）。
"""

from __future__ import annotations


class GripperArrivalDetector:
    """连续观察粉色标记是否在 ROI 内，达到时长阈值判定到位。

    触发后自动重新武装：粉色离开 ROI 后，下一次进入重新计满阈值即再次触发，
    避免同一段粉色反复触发（重复计时）。
    """

    def __init__(self, min_confirm_sec: float = 5.0):
        self.min_confirm_ns = int(min_confirm_sec * 1_000_000_000)
        self.reset()

    def reset(self) -> None:
        """清空连续计时与到位状态（下一次抓取复用）。"""
        self.present_since_ns = None
        self.arrived = False

    def update(self, present: bool, now_ns: int) -> bool:
        """每帧喂入「粉色是否在 ROI 内」，返回是否触发到位（上升沿）。

        ``now_ns`` 为单调递增的纳秒时间戳（如 ``time.monotonic_ns()``）。
        连续满 ``min_confirm_sec`` 秒触发一次；触发后等粉色消失（present=False）
        重新武装，下一次进入再重新计时，不会同一段粉色反复触发。
        """
        if self.arrived:
            if not present:
                # 粉色已离开：重新武装，等待下一次进入
                self.arrived = False
                self.present_since_ns = None
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
