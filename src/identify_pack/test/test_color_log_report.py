import unittest

from identify_pack.scripts.tools.color_log_report import (
    build_rows, format_table, parse_bridge_lines,
)


class ColorLogReportTests(unittest.TestCase):
    def test_report_matches_arrivals_to_grab_commands(self):
        color_events = [
            {'event': 'start', 'wall_time': 100.0},
            {'event': 'centers', 'wall_time': 100.2,
             'points': [[100.0, 200.0]],
             'channels': {'color_id': [1.0], 'area_px2': [500.0]}},
            {'event': 'arrived', 'wall_time': 100.5, 'data': True},
            {'event': 'centers', 'wall_time': 110.2,
             'points': [[300.0, 200.0]],
             'channels': {'color_id': [4.0], 'area_px2': [400.0]}},
            {'event': 'arrived', 'wall_time': 110.5, 'data': True},
            {'event': 'stop', 'wall_time': 120.0},
        ]
        bridge_lines = [
            "[WARN] [100.6] [serial_bridge_node]: 颜色顺序匹配：序号 1 颜色 1，"
            '转盘已到位，已下发 GrabCmd\n',
            "[WARN] [100.7] [serial_bridge_node]: TX GrabCmd(0x09) len=6: '1'\n",
            "[INFO] [101.0] [serial_bridge_node]: RX PickDone: seq=1\n",
            "[WARN] [110.6] [serial_bridge_node]: 颜色 4 与当前期望 4 不符"
            '（state=WAIT_PICK_DONE），跳过该目标\n',
            "[INFO] [111.0] [serial_bridge_node]: RX PickDone: seq=2\n",
        ]
        bridge_events = parse_bridge_lines(bridge_lines)
        rows = build_rows(color_events, bridge_events)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['color'], 1)
        self.assertEqual(rows[0]['status'], '已发')
        self.assertEqual(rows[0]['seq'], 1)
        self.assertEqual(rows[0]['pick_done'], 1)
        self.assertEqual(rows[1]['color'], 4)
        self.assertEqual(rows[1]['status'], '跳过')
        table = format_table(rows)
        self.assertIn('已发', table)
        self.assertIn('跳过', table)


if __name__ == '__main__':
    unittest.main()
