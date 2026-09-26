"""Fixed-length NSS transport: one kernel-managed setup delay and one SPI burst."""
import ctypes
import fcntl
import os
import struct
import time

# Keep these margins aligned with host/config.h; see docs/DEVELOPMENT.md.
SETUP_US = 30
INACTIVE_US = 30

class _Transfer(ctypes.Structure):
    _fields_ = [('tx_buf', ctypes.c_uint64), ('rx_buf', ctypes.c_uint64),
                ('length', ctypes.c_uint32), ('speed_hz', ctypes.c_uint32),
                ('delay_usecs', ctypes.c_uint16), ('bits_per_word', ctypes.c_uint8),
                ('cs_change', ctypes.c_uint8), ('tx_nbits', ctypes.c_uint8),
                ('rx_nbits', ctypes.c_uint8), ('word_delay_usecs', ctypes.c_uint8),
                ('pad', ctypes.c_uint8)]

assert ctypes.sizeof(_Transfer) == 32
# Linux SPI_IOC_MESSAGE(2): write ioctl, two 32-byte spi_ioc_transfer records.
_MESSAGE_2 = 0x40000000 | (2 * ctypes.sizeof(_Transfer) << 16) | (ord('k') << 8)

class NssSPI:
    def __init__(self, speed, setup_us=SETUP_US, inactive_us=INACTIVE_US):
        if not 1 <= speed <= 30_000_000 or not 1 <= setup_us <= 65535 or inactive_us < 30:
            raise ValueError('Invalid SPI speed or NSS timing')
        self.fd = None
        self.speed = speed
        self.setup_us = setup_us
        self.inactive_ns = inactive_us * 1000
        self.last_end = 0
        try:
            self.fd = os.open('/dev/spidev0.0', os.O_RDWR | os.O_CLOEXEC)
            fcntl.ioctl(self.fd, 0x40016b01, bytes([0]))  # mode 0
            fcntl.ioctl(self.fd, 0x40016b03, bytes([8]))  # bits per word
            fcntl.ioctl(self.fd, 0x40046b04, struct.pack('=I', speed))
        except BaseException:
            self.close()
            raise

    def transfer(self, frame):
        return self._exchange(bytes(frame), self.setup_us)

    def _exchange(self, frame, setup_us):
        if self.fd is None:
            raise RuntimeError('SPI transport is closed')
        tx = ctypes.create_string_buffer(frame, max(1, len(frame)))
        rx = ctypes.create_string_buffer(max(1, len(frame)))
        transfers = (_Transfer * 2)()
        transfers[0].delay_usecs = setup_us  # No clocks, NSS remains asserted.
        transfers[1].tx_buf = ctypes.addressof(tx)
        transfers[1].rx_buf = ctypes.addressof(rx)
        transfers[1].length = len(frame)
        transfers[1].speed_hz = self.speed
        transfers[1].bits_per_word = 8
        request = bytearray(bytes(transfers))
        while time.monotonic_ns() - self.last_end < self.inactive_ns:
            pass
        try:
            count = fcntl.ioctl(self.fd, _MESSAGE_2, request, True)
        finally:
            self.last_end = time.monotonic_ns()
        if count != len(frame):
            raise OSError('Short SPI exchange')
        return rx.raw[:len(frame)]

    def cancel_selection(self, wait_for_timeout=False):
        """Test NSS without clocks, optionally held beyond the MCU timeout."""
        self._exchange(b'', 35000 if wait_for_timeout else self.setup_us)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
