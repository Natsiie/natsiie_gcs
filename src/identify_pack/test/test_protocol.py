import unittest

from identify_pack.serial.protocol import (
    GIMBAL_YAW_TYPE, extract_frames, make_frame, parse_frame,
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


if __name__ == '__main__':
    unittest.main()
