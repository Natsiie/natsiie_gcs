#!/usr/bin/env python3
"""把一次颜色识别的 JSONL + 串口桥日志整理成表格。

输入：
  - color_debug_*.jsonl（color_log_node 生成；debug_color_log:=true）
  - 串口桥日志（~/.ros/log/python3_*.log，自动按时间窗口查找）

输出每行：
  到达时间 | 主色 | 像素中心 | 面积 | GrabCmd | 序号/颜色 | PickDone | 备注

用法：
  python3 color_log_report.py [color_debug.jsonl] [--bridge-log PATH ...]
  ros2 run identify_pack color_log_report -- <同上>
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import sys
from datetime import datetime

try:
    from identify_pack.scripts.color_config import COLOR_NAMES
except Exception:  # 独立运行时没有安装包也能用
    COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}

COLOR_LABELS = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}
BRIDGE_NODE = 'serial_bridge_node'
ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
LOG_RE = re.compile(
    r'\[(INFO|WARN|ERROR|DEBUG)\]\s*\[(\d+(?:\.\d+)?)\]\s*\[([^\]]+)\]:\s*(.*)')
ACCEPT_RE = re.compile(r'颜色顺序匹配：序号 (\d+) 颜色 (\d+)')
REJECT_RE = re.compile(r'颜色 (\d+) 与当前期望 (\d+) 不符')
STATE_RE = re.compile(r'收到夹爪到位，但任务状态为 ([A-Z_]+)')
PICK_DONE_RE = re.compile(r'RX PickDone: seq=(\d+)')


def color_name(color_id):
    cid = int(color_id)
    return COLOR_LABELS.get(cid, COLOR_NAMES.get(cid, str(cid)))


def primary_color(event):
    """centers 事件里按面积最大的色块取主色。"""
    channels = event.get('channels', {})
    ids = channels.get('color_id', [])
    areas = channels.get('area_px2', [])
    points = event.get('points', [])
    if not ids:
        return None
    index = max(
        range(len(ids)),
        key=lambda i: areas[i] if i < len(areas) else 0.0)
    x = points[index][0] if index < len(points) else None
    y = (points[index][1]
         if index < len(points) and len(points[index]) > 1 else None)
    area = areas[index] if index < len(areas) else None
    return int(ids[index]), x, y, area


def read_color_log(path):
    events = []
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def latest_color_log():
    pattern = os.path.expanduser(
        '~/natsiie_gcs/log/color_debug/color_debug_*.jsonl')
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(
            '未找到 color_debug_*.jsonl；请用 debug_color_log:=true 启动')
    return files[-1]


def parse_bridge_lines(lines):
    """解析串口桥日志，返回按时间排序的关键事件。"""
    events = []
    for raw in lines:
        line = ANSI_RE.sub('', raw).strip()
        match = LOG_RE.match(line)
        if match is None:
            continue
        _level, stamp, node, message = match.groups()
        if node != BRIDGE_NODE:
            continue
        event = {
            'time': float(stamp), 'kind': None,
            'seq': None, 'color': None, 'expected': None,
            'note': message,
        }
        match = ACCEPT_RE.search(message)
        if match:
            event.update(kind='accepted',
                         seq=int(match.group(1)),
                         color=int(match.group(2)))
        elif 'TX GrabCmd' in message:
            event['kind'] = 'grabcmd'
        elif '暂缓 GrabCmd' in message:
            event['kind'] = 'held'
        elif (match := REJECT_RE.search(message)):
            event.update(kind='rejected', color=int(match.group(1)),
                         expected=int(match.group(2)))
        elif (match := STATE_RE.search(message)):
            event['kind'] = 'rejected'
            event['note'] = f'状态 {match.group(1)}，未发 GrabCmd'
        elif (match := PICK_DONE_RE.search(message)):
            event.update(kind='pickdone', seq=int(match.group(1)))
        else:
            continue
        events.append(event)
    return sorted(events, key=lambda item: item['time'])


def find_bridge_events(since, until):
    """在 ~/.ros/log 中查找时间窗口内的串口桥事件。"""
    selected = []
    sources = []
    pattern = os.path.expanduser('~/.ros/log/python3_*.log')
    for path in glob.glob(pattern):
        try:
            with open(path, encoding='utf-8', errors='replace') as stream:
                events = parse_bridge_lines(stream.readlines())
        except OSError:
            continue
        window = [event for event in events
                  if since - 5.0 <= event['time'] <= until + 30.0]
        if window:
            selected.extend(window)
            sources.append((path, len(window)))
    return sorted(selected, key=lambda item: item['time']), sources


def build_rows(color_events, bridge_events, window_after=15.0):
    arrivals = [event for event in color_events
                if event.get('event') == 'arrived' and event.get('data')]
    primary = None
    rows = []
    for index, arrival in enumerate(arrivals):
        time_s = float(arrival.get('wall_time', 0.0))
        # 取该到达之前最近的一次 centers 主色
        for event in color_events:
            if (event.get('event') == 'centers'
                    and float(event.get('wall_time', 0.0)) <= time_s):
                candidate = primary_color(event)
                if candidate is not None:
                    primary = candidate
        next_s = (float(arrivals[index + 1].get('wall_time', time_s + window_after))
                  if index + 1 < len(arrivals) else time_s + window_after)
        upper = min(time_s + window_after, next_s)
        candidates = [event for event in bridge_events
                      if time_s - 1.0 <= event['time'] < upper]

        status = '—'
        seq = None
        accepted_color = None
        note = ''
        for event in candidates:
            if event['kind'] in ('accepted', 'grabcmd'):
                status = '已发'
                seq = event.get('seq')
                accepted_color = event.get('color')
                note = event['note']
                break
            if event['kind'] in ('held', 'rejected'):
                status = '暂缓' if event['kind'] == 'held' else '跳过'
                note = event['note']

        pick_done = next(
            (event['seq'] for event in candidates
             if event['kind'] == 'pickdone' and event['time'] >= time_s), None)
        color_id, x, y, area = primary if primary else (None, None, None, None)
        rows.append({
            'time': time_s,
            'color': color_id,
            'x': x, 'y': y, 'area': area,
            'status': status, 'seq': seq,
            'accepted_color': accepted_color,
            'pick_done': pick_done,
            'note': note,
        })
    return rows


def format_table(rows):
    header = ('#', '时间', '主色', '像素中心', '面积', 'GrabCmd',
              '序号/颜色', 'PickDone', '备注')
    table = [header]
    for index, row in enumerate(rows, start=1):
        color = ('—' if row['color'] is None
                 else f"{row['color']}{color_name(row['color'])}")
        center = ('—' if row['x'] is None
                  else f"({row['x']:.0f},{row['y']:.0f})")
        area = '—' if row['area'] is None else f"{row['area']:.0f}"
        seq_color = ('—' if row['seq'] is None
                     else f"{row['seq']}/{row['accepted_color'] or '?'}")
        pick = '—' if row['pick_done'] is None else str(row['pick_done'])
        note = row['note'].replace('\n', ' ')[:60]
        table.append((
            str(index),
            datetime.fromtimestamp(row['time']).strftime('%H:%M:%S'),
            color, center, area, row['status'], seq_color, pick, note))
    widths = [max(len(line[column]) for line in table)
              for column in range(len(header))]
    lines = []
    for line in table:
        lines.append('  '.join(
            cell.ljust(widths[column])
            for column, cell in enumerate(line)))
    return '\n'.join(lines)


def write_csv(rows, stream):
    writer = csv.writer(stream)
    writer.writerow(['index', 'wall_time', 'time', 'color', 'color_name',
                     'x', 'y', 'area', 'grab_cmd', 'seq', 'accepted_color',
                     'pick_done', 'note'])
    for index, row in enumerate(rows, start=1):
        writer.writerow([
            index, row['time'],
            datetime.fromtimestamp(row['time']).strftime('%Y-%m-%d %H:%M:%S'),
            row['color'], color_name(row['color']) if row['color'] else '',
            row['x'], row['y'], row['area'], row['status'],
            row['seq'], row['accepted_color'], row['pick_done'], row['note'],
        ])


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='颜色识别 JSONL + 串口桥日志 -> 到达/抓取表格')
    parser.add_argument('color_log', nargs='?',
                        help='color_debug_*.jsonl；省略时取最新一份')
    parser.add_argument('--bridge-log', action='append', default=[],
                        help='串口桥日志路径；可重复，省略时自动查找')
    parser.add_argument('--window-sec', type=float, default=15.0,
                        help='每次到达后查找 GrabCmd 的时间窗口（秒）')
    parser.add_argument('--csv', action='store_true', help='输出 CSV')
    args = parser.parse_args(argv)

    path = args.color_log or latest_color_log()
    color_events = read_color_log(path)
    stamps = [float(event['wall_time']) for event in color_events
              if 'wall_time' in event]
    since, until = (min(stamps), max(stamps)) if stamps else (0.0, 0.0)

    if args.bridge_log:
        lines = []
        for bridge_path in args.bridge_log:
            with open(bridge_path, encoding='utf-8',
                      errors='replace') as stream:
                lines.extend(stream.readlines())
        bridge_events = parse_bridge_lines(lines)
        sources = [(bridge_path, '指定') for bridge_path in args.bridge_log]
    else:
        bridge_events, sources = find_bridge_events(since, until)

    rows = build_rows(color_events, bridge_events, args.window_sec)
    print(f'颜色日志: {path}')
    for source, count in sources:
        print(f'串口桥日志: {source}（{count} 条关键事件）')
    print(f'到达事件: {len(rows)} 次')
    if args.csv:
        write_csv(rows, sys.stdout)
    else:
        print(format_table(rows))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
