"""Exercise missed releases in the actual Python loop without SPI hardware."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('control_loop', Path(__file__).resolve().parents[1] / 'python/main.py')
loop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


class PeriodicTests(unittest.TestCase):
    def check_delay(self, work_delay, wake_delay, expected_starts, expected_skips):
        now = 0
        starts, commands = [], []
        sleeps = 0

        def read(spi):
            starts.append(now)
            return (0.0,) * 4

        def send(spi, setpoint, enabled, mode):
            nonlocal now
            commands.append(enabled)
            now += work_delay if len(commands) == 1 else 100_000

        def sleep(seconds):
            nonlocal now, sleeps
            now += round(seconds * 1e9) + (wake_delay if sleeps == 0 else 0)
            sleeps += 1

        with patch.object(loop, 'PERIOD_US', 250), patch.object(loop, 'Trun', 0.00075), \
             patch.object(loop, 'read_measurements', read), patch.object(loop, 'send_command', send), \
             patch.object(loop.time, 'monotonic_ns', lambda: now), \
             patch.object(loop.time, 'perf_counter', lambda: now / 1e9), \
             patch.object(loop.time, 'sleep', sleep):
            log = loop.make_log()
            loop.run(None, log)
        self.assertEqual(starts, expected_starts)
        self.assertEqual(commands, [False, True, True])
        self.assertEqual(log['skipped_releases'], expected_skips)
        self.assertEqual(log['count'], 3)

    def test_work_overrun_skips_old_releases(self):
        self.check_delay(1_000_000, 0, [0, 1_250_000, 1_500_000], 4)

    def test_late_wakeup_executes_only_latest_release(self):
        self.check_delay(100_000, 1_000_000, [0, 1_250_000, 1_500_000], 4)

    def test_small_wakeup_delay_does_not_shift_the_schedule(self):
        self.check_delay(100_000, 1_000, [0, 251_000, 500_000], 0)
