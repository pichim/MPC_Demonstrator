# UART protocol and timing

This host preserves the existing MCU wire format: 115200 baud, 8 data bits,
no parity, one stop bit, and no flow control.

## Packets

Host to MCU (5 bytes):

| Byte offset | Type | Meaning |
|---|---|---|
| 0–3 | 32-bit float | Current command in amperes |
| 4 | Unsigned byte | Enable: 1 enables, other values disable |

MCU to host (12 bytes):

| Byte offset | Type | Meaning |
|---|---|---|
| 0–3 | 32-bit float | Motor angle |
| 4–7 | 32-bit float | Pendulum angle |
| 8–11 | 32-bit float | Measured current |

Both ends use little-endian IEEE-754 floats. The host copies individual float
bytes rather than transmitting padded C++ structs. It requires a little-endian
target with this float representation.

There is no framing marker, length field, sequence number, or CRC. Lost or inserted
bytes can destroy alignment; the host does not attempt automatic resynchronization.

## Timing

The intended host communication rate is 500 Hz (2 ms). The existing MCU uses a
200 µs communication polling ticker and a separate 50 µs current-control ticker.
These faster MCU loops are intentional and remain unchanged. The MCU responds
when it receives a command; its polling rate is not the UART transaction rate.

The imported host sends an initial command, waits for a complete response, then
sends its next command. It does not enforce a 2 ms period. The 500 Hz constant
only sets the displayed expected rate. When sample output is enabled, packet-to-packet time is measured using
`std::chrono::steady_clock` after each complete response, so it includes serial
transfer and scheduling effects. Hardware validation is needed to establish the
actual rate and jitter.

## Failures

The serial implementation uses a 20 ms read timeout for each underlying wait/read,
not a single deadline covering the entire packet. Partial reads are accumulated.
Linux waits for readable data with `poll`; Windows waits for the requested bytes
or the per-call timeout with `ReadFile`. Reported serial errors or a read timeout
stop the communication loop. Windows writes have a 20 ms timeout per `WriteFile`
call; Linux writes have no explicit timeout. This is not a hard real-time deadline.

The MCU contains a nominal 0.3 s communication watchdog. This is firmware behavior,
not a host shutdown guarantee: the MCU's blocking receive of an incomplete command
can prevent its watchdog counter from advancing. The initial host always sends
disabled commands. Firmware behavior and the protocol are unchanged by this migration.
