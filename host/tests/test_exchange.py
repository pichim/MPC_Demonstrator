"""Exercise the production single-exchange loops without accessing SPI hardware."""
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST / 'python'))
spec = importlib.util.spec_from_file_location('single_exchange', HOST / 'python/main.py')
loop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop)


class FakeSPI:
    def __init__(self, fault_at=0, fault=None):
        self.commands = []
        self.now = 0
        self.closed = False
        self.fault_at, self.fault = fault_at, fault

    def transfer(self, tx):
        assert len(tx) == 18 and tx[0] == loop.SPI_HEADER_COMMAND
        assert loop.crc8(tx[:-1]) == tx[-1]
        self.commands.append(struct.unpack('<4f', tx[1:-1]))
        self.now += 100_000
        n = len(self.commands)
        if n == self.fault_at:
            if isinstance(self.fault, BaseException):
                raise self.fault
            return self.fault
        return loop.frame(loop.SPI_HEADER_REPLY, (n, -n, n + 0.5, -n - 0.5))

    def sleep(self, seconds):
        self.now += round(seconds * 1e9)

    def close(self):
        self.closed = True


class ExchangeTests(unittest.TestCase):
    def setUp(self):
        # Test fixtures stay independent of the student's operating settings.
        for name, value in [('Trun', 0.003), ('PERIOD_US', 1000), ('SETPOINT', 0.08),
                            ('CURRENT_LIMIT_A', 1.0), ('VOLTAGE_LIMIT_V', 24.0)]:
            p = patch.object(loop, name, value)
            p.start()
            self.addCleanup(p.stop)

    def clocks(self, spi):
        for name, value in [('monotonic_ns', lambda: spi.now),
                            ('perf_counter', lambda: spi.now / 1e9), ('sleep', spi.sleep)]:
            p = patch.object(loop.time, name, value)
            p.start()
            self.addCleanup(p.stop)

    def test_helper_uses_one_command_and_returns_its_telemetry(self):
        spi = FakeSPI()
        self.assertEqual(loop.exchange(spi, 0.25, True, 1), (1, -1, 1.5, -1.5))
        self.assertEqual(spi.commands, [(0.25, 1, 1, 0)])

    def test_loop_count_limits_telemetry_and_timing(self):
        for mode, setpoint, expected in [(0, 2.0, 1.0), (0, -2.0, -1.0),
                                          (1, 30.0, 24.0), (1, -30.0, -24.0)]:
            with self.subTest(mode=mode, setpoint=setpoint), \
                 patch.object(loop, 'MODE', mode), patch.object(loop, 'SETPOINT', setpoint):
                spi = FakeSPI()
                self.clocks(spi)
                log = loop.make_log()
                loop.run(spi, log)
                self.assertEqual(spi.commands, [(0, 0, mode, 0),
                                               (expected, 1, mode, 0), (expected, 1, mode, 0)])
                self.assertEqual(log['count'], 3)
                self.assertEqual(log['skipped_releases'], 0)
                for name, values in [('voltage_V', [1, 2, 3]), ('current_A', [-1, -2, -3]),
                                     ('motor_position_rad', [1.5, 2.5, 3.5]),
                                     ('motor_velocity_rad_s', [-1.5, -2.5, -3.5]),
                                     ('sent_setpoint', [0, expected, expected]),
                                     ('sent_enable', [0, 1, 1]), ('sent_mode', [mode] * 3),
                                     ('spi', [0.1] * 3), ('dt', [0, 1, 1]),
                                     ('time_s', [0.0001, 0.0011, 0.0021])]:
                    for actual, wanted in zip(log[name], values):
                        self.assertAlmostEqual(actual, wanted, msg=name)
                self.assertEqual(spi.now, 2_100_000)  # No sleep after the final cycle.

    def test_one_cycle_only_sends_disabled_command(self):
        spi = FakeSPI()
        self.clocks(spi)
        with patch.object(loop, 'Trun', 0.001):
            loop.run(spi)
        self.assertEqual(spi.commands, [(0, 0, loop.MODE, 0)])
        self.assertEqual(spi.now, 100_000)

    def test_nonfinite_next_command_stops_before_enabled_transmission(self):
        for value in [float('nan'), float('inf'), -float('inf')]:
            spi = FakeSPI()
            self.clocks(spi)
            log = loop.make_log()
            with patch.object(loop, 'SETPOINT', value), self.assertRaises(RuntimeError):
                loop.run(spi, log)
            self.assertEqual(spi.commands, [(0, 0, loop.MODE, 0)])
            self.assertEqual(log['count'], 1)

    def test_main_disables_and_closes_on_completion_error_or_interrupt(self):
        valid = loop.frame(loop.SPI_HEADER_REPLY, (0, 0, 0, 0))
        cases = [(0, None, 0, 3), (1, valid[:-1], 1, 0), (1, OSError('startup'), 1, 0),
                 (2, OSError('injected'), 1, 1),
                 (2, KeyboardInterrupt(), 0, 1), (2, valid[:-1], 1, 1),
                 (2, valid[:-1] + bytes([valid[-1] ^ 1]), 1, 1),
                 (2, loop.frame(0x7F, (0, 0, 0, 0)), 1, 1),
                 (2, loop.frame(loop.SPI_HEADER_REPLY, (float('nan'), 0, 0, 0)), 1, 1),
                 (4, OSError('disable failed'), 1, 3)]
        for fault_at, fault, status, logged in cases:
            with self.subTest(fault_at=fault_at, fault=fault):
                spi = FakeSPI(fault_at, fault)
                self.clocks(spi)
                with patch('spi_nss.NssSPI', return_value=spi), patch.object(sys, 'argv', ['main.py']), \
                     patch.object(loop, 'report') as report, redirect_stdout(io.StringIO()), \
                     redirect_stderr(io.StringIO()):
                    self.assertEqual(loop.main(), status)
                self.assertEqual(len(spi.commands), 4 if not fault_at else min(fault_at + 1, 4))
                self.assertEqual(spi.commands[0], (0, 0, loop.MODE, 0))
                self.assertEqual(spi.commands[-1], (0, 0, loop.MODE, 0))
                self.assertTrue(spi.closed)
                self.assertEqual(report.call_args.args[0]['count'], logged)


class CppExchangeTests(unittest.TestCase):
    def test_production_loop_and_shutdown_with_fake_syscalls(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'src').mkdir()
            (root / 'tests').mkdir()
            test = root / 'tests/test_exchange.cpp'
            test.write_text((HOST / 'tests/test_exchange.cpp').read_text())
            binary = str(root / 'test_exchange')
            # Fix settings only, like patch.object above; leave production logic
            # unchanged. Five cycles let the stop-at-three case end early.
            settings = dict(SPI_SPEED_HZ='30000000', PERIOD_US='1000', Trun='0.005',
                            PRINT_EVERY='1', CURRENT_LIMIT_A='1.0f', VOLTAGE_LIMIT_V='24.0f')
            cases = [(0, '0.08f', '0.08f'), (0, '2.0f', '1.0f'), (0, '-2.0f', '-1.0f'),
                     (1, '30.0f', '24.0f'), (1, '-30.0f', '-24.0f')]
            for mode, setpoint, expected in cases:
                with self.subTest(mode=mode, setpoint=setpoint):
                    source = (HOST / 'src/main.cpp').read_text()
                    for name, value in dict(settings, MODE=str(mode), SETPOINT=setpoint).items():
                        source, count = re.subn(r'^#define\s+' + name + r'\s+[^\n]*',
                                                '#define ' + name + ' ' + value, source, flags=re.M)
                        self.assertEqual(count, 1, 'Missing/duplicate setting: ' + name)
                    (root / 'src/main.cpp').write_text(source)
                    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
                                    '-DTEST_EXPECTED_SETPOINT=' + expected,
                                    '-I' + str(HOST / 'lib/Spi'), str(test),
                                    '-Wl,--wrap=open,--wrap=close,--wrap=ioctl,--wrap=clock_gettime,--wrap=clock_nanosleep',
                                    '-o', binary], check=True, timeout=60)
                    subprocess.run([binary], check=True, timeout=10)
