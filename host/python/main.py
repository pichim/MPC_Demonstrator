"""Pi SPI master for MPC Demonstrator. Run only with matching SPI firmware."""

from array import array
import csv
import math
import struct
import sys
import time

SPI_HEADER_READ = 0x57
SPI_HEADER_COMMAND = 0x55
SPI_HEADER_REPLY = 0x45
SPI_NUM_FLOATS = 3
SPI_MSG_SIZE = 14
SPI_SPEED_HZ = 30_000_000
PERIOD_US = 1000
CURRENT_A = 0.08
Trun = 20.0  # Nominal seconds; converted to a rounded integer cycle count.
PRINT_EVERY = 1  # Samples per report after stopping; 0 prints one overall window.


CRC8_TAB = [
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
]


def crc8(data):
    crc = 0
    for byte in data:
        crc = CRC8_TAB[crc ^ byte]
    return crc


def frame(header, values):
    data = bytes([header]) + struct.pack("<3f", *values)
    return data + bytes([crc8(data)])


READ_FRAME = frame(SPI_HEADER_READ, (0.0, 0.0, 0.0))


def decode_reply(reply):
    if len(reply) != SPI_MSG_SIZE or reply[0] != SPI_HEADER_REPLY or crc8(reply[:-1]) != reply[-1]:
        raise RuntimeError("Invalid SPI reply (length/header/CRC); stopping.")
    values = struct.unpack("<3f", reply[1:-1])
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Non-finite SPI measurement; stopping.")
    return values


def read_measurements(spi):
    return decode_reply(spi.transfer(READ_FRAME))


def send_command(spi, current, enable):
    # Reply is prepared before receipt of this command; not an application ACK.
    return decode_reply(spi.transfer(frame(SPI_HEADER_COMMAND, (current, float(enable), 0.0))))


def exchange(spi, current, enable):
    measurements = read_measurements(spi)
    send_command(spi, current, enable)
    return measurements


def make_log():
    if not math.isfinite(Trun) or Trun <= 0 or not isinstance(PERIOD_US, int) or PERIOD_US <= 0 or not isinstance(PRINT_EVERY, int) or PRINT_EVERY < 0 or not math.isfinite(CURRENT_A) or abs(CURRENT_A) > 3.4028234663852886e38:
        raise ValueError("Invalid runtime, period, reporting interval or current")
    capacity = math.floor(Trun * 1_000_000 / PERIOD_US + 0.5)
    if capacity < 1:
        raise ValueError("Trun must round to at least one cycle")
    # Fixed storage; add sensor/command arrays here when data logging is needed.
    return {"dt": array("d", [0.0]) * capacity, "spi": array("d", [0.0]) * capacity, "count": 0}


def run(spi, log=None):
    if log is None:
        log = make_log()
    period = PERIOD_US / 1_000_000
    previous = None
    enabled = False
    while log["count"] < len(log["spi"]):
        start = time.perf_counter()
        motor, pendulum, measured_current = read_measurements(spi)
        read_end = time.perf_counter()
        # Future controller computation belongs here, after receiving sensors.
        current = CURRENT_A if enabled else 0.0
        command_start = time.perf_counter()
        send_command(spi, current, enabled)
        now = time.perf_counter()
        i = log["count"]
        log["dt"][i] = (now - previous) * 1000 if previous is not None else 0.0
        log["spi"][i] = (read_end - start + now - command_start) * 1000
        log["count"] = i + 1
        previous = now
        enabled = True  # First exchange always sends zero current, disabled.
        remaining = start + period - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)


def report(log, cpu_s, wall_s):
    # Formatting and output happen only after the final disable and SPI close.
    count = log["count"]
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(["cycle", "dt_n", "dt_min_ms", "dt_mean_ms", "dt_max_ms", "spi_n", "spi_min_ms", "spi_mean_ms", "spi_max_ms", "cpu_s", "wall_s"])
    for first in range(0, count, PRINT_EVERY or max(count, 1)):
        last = min(first + (PRINT_EVERY or count), count)
        fields = [last]
        for name in ("dt", "spi"):
            values = log[name][max(first, 1) if name == "dt" else first : last]
            fields.append(len(values))
            if values:
                fields.extend(f"{v:.4f}" for v in (min(values), sum(values) / len(values), max(values)))
            else:
                fields.extend(["0.0000"] * 3)
        # Run totals repeated as metadata, not per-window CPU measurements.
        writer.writerow(fields + [f"{cpu_s:.4f}", f"{wall_s:.4f}"])


def main():
    from spi_nss import NssSPI

    if len(sys.argv) != 1:
        print("No run options: edit host/python/main.py.", file=sys.stderr)
        return 1
    log = make_log()
    spi = NssSPI(SPI_SPEED_HZ)
    status = 0
    failure = None
    wall_start, cpu_start = time.perf_counter(), time.process_time()
    try:
        run(spi, log)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        failure = error
        status = 1
    finally:
        cpu_s = time.process_time() - cpu_start
        wall_s = time.perf_counter() - wall_start
        try:
            send_command(spi, 0.0, False)
        except (OSError, RuntimeError, KeyboardInterrupt) as error:
            print(f"Final disable unconfirmed: {error}", file=sys.stderr, flush=True)
            status = 1
        spi.close()
    if failure is not None:
        print(failure, file=sys.stderr, flush=True)
    report(log, cpu_s, wall_s)
    return status


if __name__ == "__main__":
    sys.exit(main())
