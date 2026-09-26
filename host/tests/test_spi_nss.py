"""Verify ioctl framing and cleanup without accessing SPI hardware."""
import ctypes
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from spi_nss import NssSPI, _Transfer, _MESSAGE_2


class NssTests(unittest.TestCase):
    def test_setup_and_payload_share_one_message(self):
        calls = []
        def ioctl(fd, request, data, mutate=False):
            if request != _MESSAGE_2:
                return 0
            segments = (_Transfer * 2).from_buffer_copy(data)
            self.assertEqual(ctypes.sizeof(_Transfer), 32)
            self.assertEqual(segments[0].length, 0)
            self.assertEqual(segments[0].delay_usecs, 30)
            self.assertEqual([s.cs_change for s in segments], [0, 0])
            self.assertEqual(segments[1].bits_per_word, 8)
            self.assertEqual(segments[1].speed_hz, 30_000_000)
            calls.append(ctypes.string_at(segments[1].tx_buf, segments[1].length))
            ctypes.memmove(segments[1].rx_buf, b'abc', 3)
            return 3
        with patch('spi_nss.os.open', return_value=99), \
             patch('spi_nss.os.close') as close, \
             patch('spi_nss.fcntl.ioctl', side_effect=ioctl):
            link = NssSPI(30_000_000)
            self.assertEqual(link.transfer(b'xyz'), b'abc')
            link.close()
            link.close()
            close.assert_called_once_with(99)
        self.assertEqual(calls, [b'xyz'])

    def test_errors_are_not_retried(self):
        for result in [0, OSError('SPI failed')]:
            with self.subTest(result=result), \
                 patch('spi_nss.os.open', return_value=99), \
                 patch('spi_nss.os.close'), \
                 patch('spi_nss.fcntl.ioctl') as ioctl:
                link = NssSPI(30_000_000)
                ioctl.reset_mock()
                if isinstance(result, Exception):
                    ioctl.side_effect = result
                else:
                    ioctl.return_value = result
                with self.assertRaises(OSError):
                    link.transfer(b'xyz')
                self.assertEqual(ioctl.call_count, 1)
                self.assertGreater(link.last_end, 0)
                link.close()

    def test_configuration_failure_closes_fd(self):
        with patch('spi_nss.os.open', return_value=99), \
             patch('spi_nss.os.close') as close, \
             patch('spi_nss.fcntl.ioctl', side_effect=OSError('configuration failed')):
            with self.assertRaises(OSError):
                NssSPI(30_000_000)
            close.assert_called_once_with(99)
