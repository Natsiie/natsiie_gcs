from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class FirmwarePlacementTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'Host GCC is unavailable')
    def test_actual_firmware_parser_deduplication_and_feedback(self):
        tests = Path(__file__).resolve().parent
        devices = tests.parents[2] / 'stm32_firmware' / 'Devices'
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'firmware-place'
            subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra',
                            '-I', str(tests / 'firmware_stubs'), '-I', str(devices),
                            str(devices / 'Serial.c'), str(tests / 'firmware_place_test.c'),
                            '-lm', '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary)], check=True, capture_output=True)
