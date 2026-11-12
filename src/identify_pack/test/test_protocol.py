import unittest

from identify_pack.serial.protocol import (
    GIMBAL_YAW_TYPE, PLACE_CONTROL_TYPE, REGION_READY_TYPE, extract_frames,
    make_frame, parse_frame, parse_place_control, parse_region_ready,
)


class ProtocolTests(unittest.TestCase):
    def test_gimbal_yaw_frame(self):
        frame = make_frame(GIMBAL_YAW_TYPE, '90.0')
        self.assertEqual(frame, bytes.fromhex('ff 07 04 39 30 2e 30 14 fe'))
        self.assertEqual(parse_frame(frame), (0x07, '90.0', b''))

    def test_partial_and_concatenated_frames(self):
        first = make_frame(0x07, '-30.5')
        second = make_frame(0x01, 'hello')
        frames, remainder = extract_frames(first[:4])
        self.assertEqual(frames, [])
        self.assertEqual(remainder, first[:4])
        frames, remainder = extract_frames(remainder + first[4:] + second)
        self.assertEqual(frames, [(0x07, '-30.5'), (0x01, 'hello')])
        self.assertEqual(remainder, b'')

    def test_bad_frame_does_not_block_following_frame(self):
        bad = bytearray(make_frame(0x07, '1.0'))
        bad[-2] ^= 0x01
        good = make_frame(0x07, '2.0')
        frames, remainder = extract_frames(bytes(bad) + good)
        self.assertEqual(frames, [(0x07, '2.0')])
        self.assertEqual(remainder, b'')

    def test_invalid_type_is_rejected(self):
        with self.assertRaises(ValueError):
            make_frame(256, 'angle')

    def test_place_control_start_and_retry(self):
        self.assertEqual(parse_place_control('start,2,1'),
                         {'action': 'start', 'stage': 'stacked_ring', 'batch': 1})
        self.assertEqual(parse_place_control('start,1,2'),
                         {'action': 'start', 'stage': 'ring_station', 'batch': 2})
        self.assertEqual(parse_place_control('retry,7'), {'action': 'retry', 'command_id': 7})
        frame = make_frame(PLACE_CONTROL_TYPE, 'start,2,1')
        self.assertEqual(parse_frame(frame)[0], PLACE_CONTROL_TYPE)

    def test_region_ready_stage(self):
        self.assertEqual(parse_region_ready('2'), {'stage': 2})
        self.assertEqual(parse_region_ready('3'), {'stage': 3})
        frame = make_frame(REGION_READY_TYPE, '3')
        self.assertEqual(parse_frame(frame)[0], REGION_READY_TYPE)
        for bad in ('1', '4', '', '2,1', 'x'):
            with self.assertRaises(ValueError):
                parse_region_ready(bad)

    def test_place_control_rejects_bad_payloads(self):
        for bad in ('start', 'start,2', 'start,2,1,9', 'start,x,1', 'start,3,1',
                    'start,2,0', 'start,2,3', 'retry', 'retry,0', 'retry,-1',
                    'stop,2,1', ''):
            with self.assertRaises(ValueError):
                parse_place_control(bad)


if __name__ == '__main__':
    unittest.main()
