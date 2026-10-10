#!/usr/bin/env python3
"""Fused ring/digit entry point; see RING_STATION_DIGIT_TEST.md.

Both stations are observed left-to-right as 1/2/3. Geometry locates the rings;
printed digit templates verify the numbering. No motion commands are sent.

python3 ring_digit_detect.py --device /dev/camera_upper --show --place-order 231
python3 ring_digit_detect.py --device /dev/camera_upper --capture
"""

try:
    from .ring_station_digit_test import main
except ImportError:
    from ring_station_digit_test import main


if __name__ == '__main__':
    raise SystemExit(main())
