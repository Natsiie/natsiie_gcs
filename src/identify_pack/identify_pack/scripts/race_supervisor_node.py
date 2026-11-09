#!/usr/bin/env python3
"""比赛流程 supervisor：按批次/区域切换检测节点并启动放置阶段。

流程（初赛 6 件物料，4 个区域阶段）：
  QR 解码 -> 转盘对位 -> 颜色识别(3 次 PickDone) ->
  粗加工区(三圆环) -> 暂存区(叠料) -> 转盘对位 -> 颜色识别(3 次) ->
  粗加工区(三圆环) -> 暂存区叠料(叠料检测) -> 结束

检测节点常驻但默认休眠（不订阅图像）；本节点通过 Bool 使能话题控制。
基础设施节点（相机、串口桥、pixel_to_base、place_task）常驻，不重开相机。
5 帧像素均值由 place_task 在收到 start 后自行累积，这里只负责发 start。

阶段与 place_task 的 phase_order 一致：
  ring_station:1 -> stacked_ring:1 -> ring_station:2 -> stacked_ring:2
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt32

PHASES = (
    ('ring_station', 1),
    ('stacked_ring', 1),
    ('ring_station', 2),
    ('stacked_ring', 2),
)
STAGE_NAME = {1: 'turntable', 2: 'ring_station', 3: 'stacked_ring'}


class RaceSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('race_supervisor_node')

        for name, value in {
            'qr_topic': '/QR_Decode',
            'align_done_topic': '/arm/align_done',
            'pick_done_topic': '/arm/pick_done',
            'place_result_topic': '/arm/place_result',
            'place_control_topic': '/arm/place_control',
            'turntable_enable_topic': '/turntable/enable',
            'color_enable_topic': '/color/enable',
            'ring_enable_topic': '/ring_station/enable',
            'stacked_enable_topic': '/stacked_ring/enable',
            'picks_per_batch': 3,
            'placements_per_phase': 3,
        }.items():
            self.declare_parameter(name, value)

        self.picks_per_batch = int(self.param('picks_per_batch'))
        self.placements_per_phase = int(self.param('placements_per_phase'))

        self.enable_pubs = {
            'turntable': self.create_publisher(
                Bool, self.param('turntable_enable_topic'), 10),
            'color': self.create_publisher(
                Bool, self.param('color_enable_topic'), 10),
            'ring_station': self.create_publisher(
                Bool, self.param('ring_enable_topic'), 10),
            'stacked_ring': self.create_publisher(
                Bool, self.param('stacked_enable_topic'), 10),
        }
        self.place_control_pub = self.create_publisher(
            String, self.param('place_control_topic'), 10)

        self.create_subscription(
            String, self.param('qr_topic'), self._on_qr, 10)
        self.create_subscription(
            String, self.param('align_done_topic'), self._on_align_done, 10)
        self.create_subscription(
            UInt32, self.param('pick_done_topic'), self._on_pick_done, 10)
        self.create_subscription(
            String, self.param('place_result_topic'), self._on_place_result, 10)

        self.phase_index = 0
        self.step = 'WAIT_QR'
        self.pick_seqs = set()
        self.place_keys = set()
        self.log('等待二维码解码')

    def param(self, name):
        return self.get_parameter(name).value

    def log(self, text):
        self.get_logger().info(
            f'[supervisor] step={self.step} phase={self.phase_index} {text}')

    def _phase(self):
        return PHASES[self.phase_index] if self.phase_index < len(PHASES) else None

    def _enable(self, active):
        """只使能 active 检测节点，其余全部关闭。"""
        for name, publisher in self.enable_pubs.items():
            publisher.publish(Bool(data=(name == active)))
        self.log(f'检测节点: {active or "全部关闭"}')

    def _on_qr(self, message):
        if self.step != 'WAIT_QR' or not message.data.strip():
            return
        self.log(f'二维码任务码: {message.data.strip()}')
        self.phase_index = 0
        self.step = 'TURNTABLE'
        self._enable('turntable')

    def _on_align_done(self, message):
        try:
            done = json.loads(message.data)
            stage = STAGE_NAME[int(done['stage'])]
            success = bool(done['success'])
        except (ValueError, KeyError, TypeError):
            return
        if not success:
            self.log(f'{stage} 矫正未成功，继续等待')
            return

        if self.step == 'TURNTABLE' and stage == 'turntable':
            self._enable('color')
            self.step = 'COLOR'
            self.pick_seqs.clear()
            self.log('转盘矫正完成，开始颜色识别')
            return

        phase = self._phase()
        if self.step == 'ALIGN' and phase is not None and stage == phase[0]:
            self._start_placement(phase)

    def _start_placement(self, phase):
        self.place_control_pub.publish(String(data=json.dumps({
            'action': 'start', 'stage': phase[0], 'batch': phase[1]})))
        self.step = 'PLACE'
        self.place_keys.clear()
        self.log(f'{phase[0]} 矫正完成，已启动放置（place_task 累积 5 帧均值）')

    def _on_pick_done(self, message):
        if self.step != 'COLOR':
            return
        sequence = int(message.data)
        self.pick_seqs.add(sequence)
        self.log(f'PickDone {sequence}，本批已 {len(self.pick_seqs)}/{self.picks_per_batch}')
        if len(self.pick_seqs) < self.picks_per_batch:
            return
        phase = self._phase()
        if phase is None:
            return
        self._enable(phase[0])
        self.step = 'ALIGN'
        self.log(f'本批颜色识别完成，等待 {phase[0]} 区域矫正完成')

    def _on_place_result(self, message):
        if self.step != 'PLACE':
            return
        try:
            result = json.loads(message.data)
            if not bool(result['success']):
                return
            key = (str(result['task_id']), int(result['command_id']))
        except (ValueError, KeyError, TypeError):
            return
        if key in self.place_keys:
            return
        self.place_keys.add(key)
        self.log(
            f'放置完成 command={key[1]}，本阶段 '
            f'{len(self.place_keys)}/{self.placements_per_phase}')
        if len(self.place_keys) < self.placements_per_phase:
            return
        self._phase_done()

    def _phase_done(self):
        self.phase_index += 1
        self.place_keys.clear()
        if self.phase_index >= len(PHASES):
            self.step = 'DONE'
            self._enable(None)
            self.log('全部阶段完成')
            return
        if self.phase_index == 2:
            # 第一批两个区域完成，回原料区；转盘对位后进入第二批颜色识别。
            self.step = 'TURNTABLE'
            self._enable('turntable')
            self.log('第一批完成，等待转盘对位（第二批）')
            return
        phase = self._phase()
        self.step = 'ALIGN'
        self._enable(phase[0])
        self.log(f'进入下一阶段 {phase[0]}:{phase[1]}，等待矫正完成')


def main(args=None):
    rclpy.init(args=args)
    node = RaceSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
