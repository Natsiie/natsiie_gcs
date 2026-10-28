"""ROI 内物块静止帧计数。

用「物块在 ROI 内连续保持位置不变」的帧数判断是否停到位：

- 每帧输入一个主色块 ``(color_id, cx, cy)`` 或 ``None``；
- 相对基准点的位移不超过 ``max_pixel_move`` 时计数 +1；
- 位置跳变、颜色变化、连续丢失超过 ``miss_frames`` 帧或帧间隔过大时清零；
- 计数达到阈值后 ``ready=True``，下游据此放行抓取。

阈值有两种给法：``hold_frames > 0`` 时直接按帧数（便于实测调参），
否则按 ``hold_sec × 实测 fps`` 换算，帧率变化时实际停稳时长基本不变。

纯逻辑模块，无 ROS / 无 OpenCV，便于单测和复用。
"""

from __future__ import annotations

import math


class RoiStationaryCounter:
    """统计物块在 ROI 内保持不动的连续帧数。"""

    def __init__(self, hold_sec: float = 0.6, hold_frames: int = 0,
                 max_pixel_move: float = 10.0, miss_frames: int = 2,
                 fixed_fps: float = 0.0, min_frame_interval: float = 0.02,
                 max_frame_interval: float = 0.5):
        numbers = (hold_sec, max_pixel_move, fixed_fps,
                   min_frame_interval, max_frame_interval)
        if (not all(math.isfinite(value) for value in numbers)
                or hold_sec < 0 or max_pixel_move <= 0 or fixed_fps < 0
                or min_frame_interval <= 0
                or max_frame_interval < min_frame_interval):
            raise ValueError('Invalid stationary parameters')
        if (type(hold_frames) is not int or hold_frames < 0
                or type(miss_frames) is not int or miss_frames < 0):
            raise ValueError('hold_frames and miss_frames must be non-negative ints')
        if hold_frames == 0 and hold_sec == 0:
            raise ValueError('hold_sec or hold_frames must be positive')
        self.hold_sec = float(hold_sec)
        self.hold_frames = hold_frames
        self.max_pixel_move = float(max_pixel_move)
        self.miss_frames = miss_frames
        self.fixed_fps = float(fixed_fps)
        self.min_frame_interval = float(min_frame_interval)
        self.max_frame_interval = float(max_frame_interval)
        self._fps = self.fixed_fps
        self.reset()

    def reset(self) -> None:
        """清空基准点和计数。相机断流、ROI 无效或换目标时调用。"""
        self._anchor = None          # (color_id, cx, cy)
        self._count = 0
        self._misses = 0
        self._last_timestamp = None

    @property
    def required_frames(self) -> int:
        """当前需要的连续帧数；fps 未测出时按 25 帧/秒估算。"""
        if self.hold_frames > 0:
            return self.hold_frames
        fps = self.fixed_fps if self.fixed_fps > 0 else (
            self._fps if self._fps > 0 else 25.0)
        return max(1, int(round(self.hold_sec * fps)))

    def _result(self) -> dict:
        required = self.required_frames
        ready = self._count >= required
        if self._anchor is None:
            state = 'WAITING'
        elif self._misses > 0:
            # 容忍期内 ready 仍保持，但状态如实反映这一帧没检测到目标。
            state = 'MISSING'
        elif ready:
            state = 'READY'
        else:
            state = 'TRACKING'
        return {
            'state': state,
            'count': self._count,
            'required_frames': required,
            'fps': self._fps,
            'ready': ready,
            'missing_frames': self._misses,
        }

    def update(self, blob, timestamp: float) -> dict:
        """处理一帧；``blob`` 为 ``(color_id, cx, cy)`` 或 ``None``。"""
        if not math.isfinite(timestamp) or timestamp <= 0:
            # 没有可用时间戳就无法证明连续性，直接清空。
            self.reset()
            return self._result()

        if self._last_timestamp is not None:
            interval = timestamp - self._last_timestamp
            if interval > self.max_frame_interval:
                # 掉帧太久，无法确认这段间隔里物块没动，重新计时。
                self._anchor = None
                self._count = 0
                self._misses = 0
            elif self.min_frame_interval <= interval and self.fixed_fps <= 0:
                instant = 1.0 / interval
                self._fps = instant if self._fps <= 0 else (
                    0.8 * self._fps + 0.2 * instant)
        self._last_timestamp = timestamp

        if blob is None:
            self._misses += 1
            if self._misses > self.miss_frames:
                self._anchor = None
                self._count = 0
            return self._result()

        color_id, cx, cy = blob
        self._misses = 0
        if self._anchor is None:
            self._anchor = (int(color_id), float(cx), float(cy))
            self._count = 1
        else:
            anchor_color, anchor_x, anchor_y = self._anchor
            distance = math.hypot(float(cx) - anchor_x, float(cy) - anchor_y)
            if int(color_id) != anchor_color or distance > self.max_pixel_move:
                # 换了物块或已经转走：以当前位置为新基准重新计数。
                self._anchor = (int(color_id), float(cx), float(cy))
                self._count = 1
            else:
                self._count += 1
        return self._result()
