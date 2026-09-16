# UART protocol and timing

The GPIO UART connection uses 230400 baud, 8 data bits,
no parity, one stop bit, and no flow control. Both host and MCU must use this
baud rate. Packet layout is unchanged.

## Packets

Host to MCU (5 bytes):

| Byte offset | Type | Meaning |
|---|---|---|
| 0–3 | 32-bit float | Current command in amperes |
| 4 | Unsigned byte | Enable: 1 enables, other values disable |

MCU to host (12 bytes):

| Byte offset | Type | Meaning |
|---|---|---|
| 0–3 | 32-bit float | Motor angle in radians |
| 4–7 | 32-bit float | Pendulum angle in radians |
| 8–11 | 32-bit float | Measured current in amperes |

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

The host sends an initial disabled command and waits for a complete response.
It then sends enabled 0.05 A commands, waiting until at least 2 ms after the
previous command start before each send. Late cycles do not trigger catch-up
bursts. The pacing runs even when sample printing is disabled.
When sample output is enabled, packet-to-packet time is measured using
`std::chrono::steady_clock` after each complete response, so it includes serial
transfer and scheduling effects. Hardware validation is needed to establish the
actual rate and jitter.

The displayed `current_cmd` and `enable` describe the command sent before the
reported response. The measured current is a separate MCU sensor value and is
not expected to match the command exactly at every sample.

## Failures

The serial implementation uses a 20 ms read timeout for each underlying wait/read,
not a single deadline covering the entire packet. Partial reads are accumulated.
Linux waits for readable data with `poll`. Reported serial errors or a read
timeout stop the communication loop. Writes have no explicit timeout. This is
not a hard real-time deadline.

The MCU contains a nominal 0.3 s communication watchdog. This is firmware behavior,
not a host shutdown guarantee: the MCU's blocking receive of an incomplete command
can prevent its watchdog counter from advancing. Only the startup command is
disabled; the subsequent constant-current test enables the motor. Firmware
behavior and the protocol are unchanged by this host test.
