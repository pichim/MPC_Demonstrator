"""Protocol ordering and reply validation, independent of transport hardware."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import main as protocol


class ProtocolTests(unittest.TestCase):
    def test_sensor_read_precedes_controller_command_in_run(self):
        log = []
        def read(link):
            log.append('read')
            return 1.0, 2.0, 3.0
        def send(link, current, enabled):
            log.append(('command', current, enabled))
            raise KeyboardInterrupt
        with patch.object(protocol, 'read_measurements', read), \
             patch.object(protocol, 'send_command', send), patch('builtins.print'):
            with self.assertRaises(KeyboardInterrupt):
                protocol.run(object())
        self.assertEqual(log, ['read', ('command', 0.0, False)])

    def test_invalid_measurements_prevent_command(self):
        with patch.object(protocol, 'read_measurements', side_effect=RuntimeError('CRC')), \
             patch.object(protocol, 'send_command') as send, patch('builtins.print'):
            with self.assertRaises(RuntimeError):
                protocol.run(object())
            send.assert_not_called()

    def test_reply_crc_and_finite_values(self):
        valid = protocol.frame(protocol.SPI_HEADER_REPLY, (1., 2., 3.))
        self.assertEqual(protocol.decode_reply(valid), (1., 2., 3.))
        for bad in (valid[:-1], valid[:-1] + bytes([valid[-1] ^ 1]),
                    protocol.frame(protocol.SPI_HEADER_REPLY, (float('nan'), 0., 0.))):
            with self.assertRaises(RuntimeError):
                protocol.decode_reply(bad)


if __name__ == '__main__':
    unittest.main()
