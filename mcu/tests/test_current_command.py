"""Compile and exercise the actual MCU command policy with the native C++ compiler."""
import subprocess
import tempfile
import unittest
from pathlib import Path


class CurrentCommandTests(unittest.TestCase):
    def test_validation_expiry_and_ticker_wrap(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'test_current_command')
            subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(root / 'lib/fast_realtime_thread'),
                            str(root / 'tests/test_current_command.cpp'), '-o', binary],
                           check=True)
            subprocess.run([binary], check=True)


if __name__ == '__main__':
    unittest.main()
