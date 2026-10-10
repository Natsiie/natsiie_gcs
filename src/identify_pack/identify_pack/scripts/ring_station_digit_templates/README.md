# Printed Ring Templates

White foreground on black background, normalized to 40x40 pixels by
`ring_station_digits.normalize_digit`. Each label has three samples extracted
from the saved camera photos in `../ring_station_digit_snapshots/`:

- `20261010_112118_147303282_raw.png`
- `20261010_112123_034298165_raw.png`
- `20261010_113446_780547157_raw.png`

The photo labels were visually checked: left 1, middle 2, right 3. Samples keep
the source filename for traceability. The legacy `../digit_templates/` contains
mislabeled shapes and is not loaded by the fused detector.

Capture replacement samples with `ring_digit_detect.py --capture`. Select a
ring, check the extracted crop, then press its actual digit key. The production
node requires templates for all three labels.
