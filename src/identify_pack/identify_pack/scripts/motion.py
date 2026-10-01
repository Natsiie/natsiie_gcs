#!/usr/bin/env python3
"""转盘转动/停稳判定（移植自 cyqm material_pickup_detector 的运动状态机）。

跨帧匹配色块质心，算最大位移速度，走「先见转动 → 再见停稳」状态机。
关键设计（与 cyqm 一致）：初始静止不算停稳，必须先观察到转动。

纯逻辑模块，无 ROS / 无 GUI，便于单测和复用。
"""

from __future__ import annotations

import itertools
import math


class TurntableMotionTracker:
    """追踪色块跨帧运动，判定转盘是否转动过、是否接近停稳。

    输入为每帧的色块列表 ``[(color_id, cx, cy), ...]``（至少含颜色和质心），
    ``update()`` 返回一个状态 dict，含 ``state`` / ``speed`` / ``ready`` 等。
    """

    def __init__(self, moving_speed: float = 30.0,
                 required_moving_frames: int = 2,
                 near_stop_speed: float = 25.0,
                 required_low_speed_frames: int = 3,
                 max_match_distance: float = 80.0,
                 min_frame_interval: float = 0.02,
                 max_frame_interval: float = 0.25):
        self.moving_speed = float(moving_speed)
        self.required_moving_frames = int(required_moving_frames)
        self.near_stop_speed = float(near_stop_speed)
        self.required_low_speed_frames = int(required_low_speed_frames)
        self.max_match_distance = float(max_match_distance)
        self.min_frame_interval = float(min_frame_interval)
        self.max_frame_interval = float(max_frame_interval)
        self.reset()

    def reset(self) -> None:
        """清除跨帧基线；转动事实也一起清除（摄像头断开等场景用）。"""
        self._prev = None        # [(color_id, cx, cy), ...]
        self._prev_t = None
        self._motion_confirmed = False
        self._moving_frames = 0
        self._low_speed_frames = 0

    @staticmethod
    def _match_speed(prev, current, interval_s, max_match_distance):
        """把当前色块匹配到上一帧（颜色一致 + 最近），返回最大位移速度 px/s。"""
        if not prev or not current or len(prev) != len(current):
            return None
        best_dists = None
        best_total = math.inf
        # 色块数量通常很少，穷举排列比引入依赖更简单可靠。
        for order in itertools.permutations(range(len(current))):
            dists = []
            ok = True
            for old_i, new_i in enumerate(order):
                if prev[old_i][0] != current[new_i][0]:
                    ok = False
                    break
                d = math.hypot(current[new_i][1] - prev[old_i][1],
                               current[new_i][2] - prev[old_i][2])
                if d > max_match_distance:
                    ok = False
                    break
                dists.append(d)
            if ok:
                total = sum(dists)
                if total < best_total:
                    best_total = total
                    best_dists = dists
        if best_dists is None:
            return None
        return max(best_dists) / interval_s

    def update(self, blobs, timestamp: float) -> dict:
        """处理一帧色块，返回运动状态 dict。

        ``blobs``：``[(color_id, cx, cy), ...]``；``timestamp``：单调秒。
        """
        current = sorted((int(b[0]), float(b[1]), float(b[2])) for b in blobs)
        result = {
            'state': 'WAITING_FOR_MOTION',
            'speed': 0.0,
            'motion_confirmed': self._motion_confirmed,
            'moving_frames': self._moving_frames,
            'required_moving_frames': self.required_moving_frames,
            'low_speed_frames': self._low_speed_frames,
            'required_low_speed_frames': self.required_low_speed_frames,
            'ready': False,
        }
        if self._prev_t is None:
            # 首帧只建立跟踪基线，尚不能判断运动。
            self._prev = current
            self._prev_t = timestamp
            return result

        interval_s = timestamp - self._prev_t
        baseline_state = ('ACQUIRING' if self._motion_confirmed
                          else 'WAITING_FOR_MOTION')
        if not (self.min_frame_interval <= interval_s <= self.max_frame_interval):
            self._moving_frames = 0
            self._low_speed_frames = 0
            result['state'] = baseline_state
        else:
            speed = self._match_speed(self._prev, current, interval_s,
                                      self.max_match_distance)
            if speed is None:
                # 数量或颜色变化：重建测速基线，但保留本周期已经转动的事实。
                self._moving_frames = 0
                self._low_speed_frames = 0
                result['state'] = baseline_state
            else:
                result['speed'] = speed
                if speed >= self.moving_speed:
                    self._moving_frames += 1
                    self._low_speed_frames = 0
                    if self._moving_frames >= self.required_moving_frames:
                        self._motion_confirmed = True
                        self._moving_frames = self.required_moving_frames
                        result['state'] = 'MOVING'
                    else:
                        result['state'] = 'CONFIRMING_MOTION'
                elif not self._motion_confirmed:
                    # 初始静止不能视为停稳，必须先见过运动。
                    self._moving_frames = 0
                    self._low_speed_frames = 0
                    result['state'] = 'WAITING_FOR_MOTION'
                elif speed > self.near_stop_speed:
                    self._low_speed_frames = 0
                    result['state'] = 'SETTLING'
                else:
                    self._low_speed_frames += 1
                    if self._low_speed_frames >= self.required_low_speed_frames:
                        result['state'] = 'READY'
                        result['ready'] = True
                    else:
                        result['state'] = 'NEAR_STOP'

        result['motion_confirmed'] = self._motion_confirmed
        result['moving_frames'] = self._moving_frames
        result['low_speed_frames'] = self._low_speed_frames
        self._prev = current
        self._prev_t = timestamp
        return result
