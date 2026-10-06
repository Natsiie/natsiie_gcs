#!/usr/bin/env python3
"""ROI 选择器：拖框定义多个 ROI、读写 ROI_Range.txt、叠加绘制。

把 color_detect.py 里的画 ROI 逻辑封装成可复用类。circle_detect 节点
只读文件（用 color_config.load_roi_file），不依赖本类。
"""

from __future__ import annotations

import cv2

try:
    from identify_pack.scripts.color_config import load_roi_file
except ImportError:  # 直接在源码目录运行
    from color_config import load_roi_file  # type: ignore


class RoiSelector:
    """管理多个 ROI 矩形：加载文件、拖框添加、写回文件、叠加绘制。

    用法::

        selector = RoiSelector('ROI_Range.txt', max_rois=3)
        selector.load()                              # 默认读取文件里已有 ROI
        cv2.setMouseCallback(win, on_mouse, selector)
        ...
        rois = selector.rois or None                 # 空列表 = 全画面
        frame = selector.draw(frame)
        selector.save() / selector.clear()
    """

    def __init__(self, roi_file: str = 'ROI_Range.txt', max_rois: int = 3):
        self.roi_file = roi_file
        self.max_rois = max_rois
        self.rois = []          # [(x, y, w, h), ...]
        self.drawing = False    # 是否正在拖框
        self._start = None      # 拖框起点 (x, y)
        self._current = None    # 拖框当前 (x, y)

    def load(self) -> list:
        """读取 ROI 文件里的 ROI（每行 ``x,y,w,h``），返回列表。"""
        self.rois = load_roi_file(self.roi_file)
        return self.rois

    def save(self) -> None:
        """把当前所有 ROI 写回文件，每行一个 ``x,y,w,h``。"""
        with open(self.roi_file, 'w', encoding='utf-8') as f:
            for (x, y, w, h) in self.rois:
                f.write(f'{x},{y},{w},{h}\n')

    def clear(self) -> None:
        """清空所有 ROI 并同步清空文件。"""
        self.rois = []
        self.drawing = False
        self._start = None
        self._current = None
        self.save()

    def start_drawing(self) -> bool:
        """进入拖框模式；已满 max_rois 时返回 False。"""
        if len(self.rois) >= self.max_rois:
            return False
        self.drawing = True
        self._start = None
        self._current = None
        return True

    def on_mouse(self, event, x, y, flags, param) -> None:
        """拖框模式下的鼠标回调：左键拖框定义一个 ROI 并写回文件。"""
        if not self.drawing:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self._start = (x, y)
            self._current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self._start is not None:
            self._current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._start is not None:
            x1, y1 = self._start
            x2, y2 = self._current
            rx, ry = min(x1, x2), min(y1, y2)
            rw, rh = abs(x2 - x1), abs(y2 - y1)
            self._start = None
            self._current = None
            self.drawing = False
            if rw < 5 or rh < 5:
                print('ROI 拖框太小，已忽略')
                return
            if len(self.rois) >= self.max_rois:
                print(f'已有 {self.max_rois} 个 ROI，请先 clear()')
                return
            self.rois.append((rx, ry, rw, rh))
            self.save()
            print(f'ROI{len(self.rois)} 已设置: ({rx}, {ry}, {rw}, {rh}) '
                  f'-> 已写入 {self.roi_file}')

    def draw(self, frame):
        """在画面上叠加绘制 ROI 矩形与拖框过程，返回同一画面。"""
        for i, (x, y, w, h) in enumerate(self.rois, start=1):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f'ROI{i}', (x, max(14, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        if self.drawing and self._start is not None and self._current is not None:
            cv2.rectangle(frame, self._start, self._current, (0, 255, 255), 1)
        return frame
