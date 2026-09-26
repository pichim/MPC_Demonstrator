"""Bounded SPI framing/recovery check. Sends only zero-current disabled commands."""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import main as protocol


def run(spi, count, period):
    batch_start = time.perf_counter()
    for index in range(count):
        start = time.perf_counter()
        try:
            protocol.exchange(spi, 0.0, False)
        except (RuntimeError, OSError) as error:
            raise RuntimeError(f'exchange {index + 1}/{count}: {error}') from error
        remaining = start + period - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
    return (time.perf_counter() - batch_start) / count * 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=2000)
    parser.add_argument('--speed', type=int, default=protocol.SPI_SPEED_HZ)
    parser.add_argument('--period-us', type=int, default=round(protocol.PERIOD_S * 1e6))
    parser.add_argument('--fault-rounds', type=int, default=10)
    parser.add_argument('--faults', action='store_true', help='Inject malformed frames, cancellation, and timeout; require NSS recovery.')
    args = parser.parse_args()
    if min(args.count, args.speed, args.period_us, args.fault_rounds) <= 0:
        parser.error('count, speed, period and fault rounds must be positive')
    from spi_nss import NssSPI
    spi = NssSPI(args.speed)
    try:
        average_ms = run(spi, args.count, args.period_us / 1e6)
        print(f'PASS baseline: {args.count} exchanges, {args.speed} Hz SPI, '
              f'{args.period_us} us target period, NSS, actual mean cycle={average_ms:.3f} ms', flush=True)
        if args.faults:
            disabled = protocol.frame(protocol.SPI_HEADER_COMMAND, (0.0, 0.0, 0.0))
            bad_crc = disabled[:-1] + bytes([disabled[-1] ^ 1])
            cases = [(f'{size}-byte frame', (disabled * 2)[:size]) for size in (1, 7, 13, 15, 28)]
            cases.append(('bad CRC', bad_crc))
            cases.append(('unknown header', protocol.frame(0x7f, (0.0, 0.0, 0.0))))
            for iteration in range(args.fault_rounds):
                for label, frame in cases:
                    spi.transfer(frame)
                    try:
                        run(spi, 100, args.period_us / 1e6)
                    except RuntimeError as error:
                        raise RuntimeError(f'round {iteration + 1}, after {label}: {error}') from error
                spi.cancel_selection()
                run(spi, 100, args.period_us / 1e6)
                spi.cancel_selection(wait_for_timeout=True)
                run(spi, 100, args.period_us / 1e6)
                print(f'PASS recovery round {iteration + 1}/{args.fault_rounds}: '
                      f'{len(cases)} frame faults + cancellation + timeout, 100 valid exchanges after each', flush=True)

    finally:
        # Every normal command and injected command prefix above is disabled.
        spi.close()


if __name__ == '__main__':
    main()
