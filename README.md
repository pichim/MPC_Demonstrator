# MPC Demonstrator

Raspberry Pi SPI master and NUCLEO-F446RE firmware for a future MPC controller.
The Pi reads telemetry, computes a command (currently a fixed test setpoint),
then sends it. The MCU runs the current controller every **50 µs**.

Development uses VS Code over SSH on the Pi. Currently no motor or sensors are
attached to the Nucleo; communication tests work, but telemetry is not meaningful
physical measurement. The experiment history, failed approaches, timing tables,
research sources and remaining limitations are in
[Development and validation](docs/DEVELOPMENT.md).

## Repository layout

- `mcu/`: firmware, local Mbed dependency, MCU tests, and MCU build output.
- `host/`: C++ and Python clients, host tests, and host build output.
- `docs/`: development and validation records.

Terminal builds belong in `mcu/build` and `host/build`; VS Code uses
`mcu/build/NUCLEO_F446RE-Develop` for the MCU.
Historical local checkpoints remain in `mcu/build/checkpoints/`.

## Wiring

Power off before rewiring. Use short 3.3 V signal wires and common ground;
connect no power pins between boards. ST-LINK USB powers/programs the Nucleo.

| Pi 5 physical pin | Signal | Nucleo-F446RE |
| --- | --- | --- |
| 19 / GPIO10 | MOSI | PC3 / CN7-37 |
| 21 / GPIO9 | MISO | PC2 / CN7-35 |
| 23 / GPIO11 | SCK | PB10 / CN10-25 |
| 24 / GPIO8, CE0 | NSS | PB12 / CN10-16 |
| 6 | GND | CN7-8 |

Pin reference: [ST UM1724](https://www.st.com/resource/en/user_manual/dm00105823.pdf).
Previous REQUEST/READY and UART command wires are unused. REQUEST/READY wires
may remain connected, but the old clients are incompatible with this firmware.

## Build and run

Run these commands from the repository root. The MCU commands use the existing
Arm toolchain and Python environment installed on this Pi, with the same
**NUCLEO_F446RE / Develop** settings as the default VS Code build:

```sh
export PATH=/usr/local/gcc-arm/bin:$PATH

cmake -S mcu -B mcu/build -G Ninja \
  -DMBED_TARGET=NUCLEO_F446RE \
  -DCMAKE_BUILD_TYPE=Develop \
  -DUPLOAD_METHOD=MBED \
  -DPython3_EXECUTABLE="$PWD/mcu/mbed-os/venv/bin/python3"

cmake --build mcu/build --target MPC_Demonstrator -j2
```

**Ctrl+Shift+B also flashes the board.** To build and flash from the terminal,
stop SPI clients and mount the Nucleo drive first (the VS Code **Mount NUCLEO**
task can do this), then run:

```sh
cmake --build mcu/build --target flash-MPC_Demonstrator -j2
```

Alternatively, copy `mcu/build/MPC_Demonstrator.bin` onto the `NOD_F446RE` drive.

VS Code uses `.vscode/cmake-variants.yaml` for the MCU board and build type,
and builds in `mcu/build/NUCLEO_F446RE-Develop`. Replace `mcu/build` with that
path in the commands above to share its build directory. Otherwise, the terminal
and VS Code maintain separate build outputs.

Reconfigure CMake after adding/removing library sources; its source glob is
resolved at configure time.

Pi SPI0 must be enabled (`dtparam=spi=on` in the active boot `config.txt`) and
`/dev/spidev0.0` present. No custom kernel or device-tree overlay is required.
Run **only one client at a time**.

Python uses only its standard library:

```sh
sudo chrt -f 50 python3 -u host/python/main.py 2>&1 | tee host/spi_timing.txt
```

It targets **1 kHz**, requests **30 MHz SPI mode 0**, and sends **0.08 A enabled**
after the first zero-current disabled cycle. Edit settings in `host/python/main.py`.
`PRINT_EVERY = 500` prints every 500 cycles; set it to zero to omit periodic
printing and interval collection. Ctrl+C attempts a final disable and closes SPI.

For quieter timing measurements and the future native controller:

```sh
cmake -S host -B host/build -DCMAKE_BUILD_TYPE=Release
cmake --build host/build -j2
sudo chrt -f 50 host/build/mpc_spi \
  --count 10000 --period-us 1000 --enable --csv /tmp/jitter-nss.csv
```

C++ defaults to 10,000 cycles, 1 kHz, 30 MHz and **disabled** commands. `--enable`
uses 0.08 A; `--current A` changes the test setpoint. Its first cycle is disabled.
Normal completion, Ctrl+C/SIGTERM and transport errors attempt a final disable.
An exchange reply is not confirmation that a command was applied.

Both clients use the same wire protocol, READ → compute → COMMAND ordering, and
30 µs NSS setup/high margins. The C++ command above matches Python's default
rate and enabled setpoint, but stops after 10,000 cycles. Python runs until stopped
and prints telemetry periodically; C++ preallocates timing samples and reports
after completion. Python has no graceful SIGTERM handler; use Ctrl+C for its
final-disable attempt. Different runtime and printing overhead means timing
distributions need not match.

`dt` is the interval between command-reply completions. C++ also reports `read`
and `work` (READ start through COMMAND completion), work above the target period,
and CPU use as a percentage of one core. CSV output happens after closing SPI.
Timing excludes MPC computation. Periods are relative to each cycle's start;
late cycles do not trigger catch-up bursts. `chrt` does not provide a hard deadline.

Measured C++ mean work is about **155 µs**: 15.5% of a 1 ms period, leaving about
845 µs on average for computation and scheduling margin. This is not guaranteed
spare time. Python has different overhead; printed `dt` is not communication work.
See the [matched results](docs/DEVELOPMENT.md#nss-results) for percentiles and limits.

## Code and ownership

| Location | Responsibility |
| --- | --- |
| `mcu/include/config.h` | MCU pins, periods, priorities and controller settings |
| `mcu/src/main.cpp` | Construct hardware/tasks and start them |
| `mcu/lib/fast_realtime_thread/` | 50 µs High2 task: sensors, current control, outputs, expiry |
| `mcu/lib/IO_handler/` | Encoders, current ADC, PWM, direction and enable |
| `mcu/lib/SPISlaveDMA/` | NSS handlers, immutable DMA frame, latest-command mailbox, High1 recovery worker |
| `host/python/main.py`, `host/python/spi_nss.py` | Python control-loop entry point and NSS transport |
| `host/main.cpp`, `host/spi_nss.h`, `host/protocol.h` | Native loop, NSS transport and protocol |
| `mcu/tests/` | MCU command-policy regression test |
| `host/tests/` | Python regression tests and disabled-command hardware checks |

Libraries inherited from the last commit remain in `mcu/lib/`, including currently
unused utilities. They are retained for future coursework.

Add Pi controller computation between READ and COMMAND at the marked location in
`host/python/main.py` or `host/main.cpp`. The MCU current task owns controller/filter
state and outputs. Short critical sections protect coherent snapshots and the
single latest command; there is no command queue or current-loop mutex.

NSS falling copies the latest published telemetry and arms normal-mode DMA.
The Pi submits one `SPI_IOC_MESSAGE(2)`: a zero-byte **30 µs delay** segment, then
a 14-byte full-duplex burst, with NSS low throughout. NSS rising validates and
publishes the received command. The current task never writes the active DMA
buffer. Before another exchange the client enforces at least **30 µs NSS high**;
controller computation can cover that interval. Settings are in `host/config.h`
and `host/python/spi_nss.py`; keep them aligned.

There are exactly two useful bursts per cycle: **READ → compute → COMMAND**.
No ARM transfer or extra signaling is used. Timing margins are minimum waits;
Linux may extend them. The MCU rejects selections held beyond 20 ms. Normal frame
handling and bounded peripheral reset run in the NSS interrupt; the worker polls
for timeout and performs fallback recovery. No HAL polling loop runs in the IRQ.

## Wire protocol and command expiry

Every frame is 14 bytes: one header, three little-endian float32 values, CRC-8
(poly 0x07, initial value zero, over the first 13 bytes).

| Frame | Header | Three payload values |
| --- | --- | --- |
| READ | `0x57` | All twelve payload bytes zero |
| COMMAND | `0x55` | Current in A, enable exactly 0 or 1, reserved zero |
| REPLY | `0x45` | Motor angle in rad, pendulum angle in rad, current in A |

Both exchanges return telemetry selected before receiving that request. COMMAND's
reply is not an application ACK. There is no sequence number or sample timestamp,
so 50 µs sampling is not a guarantee of 50 µs data age at the Pi.

Only CRC-valid commands with finite current, valid enable and reserved zero refresh
the **300 ms command expiry**, measured from MCU receipt. Reads and corrupt frames
do not refresh it. Invalid command values disable output when consumed; a newer
command can overwrite an unconsumed one. Repeated valid commands refresh expiry.
Disable/expiry clears enable and PWM and resets the controller/filter. Direction
and PWM are prepared before asserting enable. GPA mode also requires valid enabled
commands. Expiry depends on the current task running, and cannot protect against
an MCU stall. Current magnitude is not constrained by this protocol; motor limits
and the reserved fault input still need consideration before physical operation.

## Validation and diagnostics

Run regression checks without hardware:

```sh
python3 -m unittest discover -s mcu/tests -v
python3 -m unittest discover -s host/tests -v
ctest --test-dir host/build --output-on-failure
```

Run disabled-command hardware recovery checks, sequentially:

```sh
sudo chrt -f 50 host/build/mpc_spi --count 10000 --period-us 500 --faults
sudo chrt -f 50 python3 host/tests/check_spi.py --count 1000 --period-us 500 --faults
```

Each injects short/extra frames, corrupt CRC, unknown header, cancellation and
held-NSS timeout, then checks 100 normal cycles after each fault. C++ recovery
cycles use 1 ms; Python uses the selected period. These test framing/recovery,
not physical control or every command's application. Clients stop on bad replies;
restart after a fault. Arbitrary resets are not transparently recoverable.

SPI error/recovery counters are available through `SpiSlaveDMA::getDiagnostics()`
for debugging. PB5 marks current-task execution. Temporary timing instrumentation
and comparison scripts are documented in the development record rather than kept
in the student code.
`MPC_PERFORM_GPA_MEAS` retains the existing optional frequency-response mode; it
has been compile-checked, not validated with a motor.

The full [development record](docs/DEVELOPMENT.md) preserves all available test
reports, unresolved cases, research and local rollback locations. Further testing
should include real controller computation, electrical NSS/SCK timing and physical
sample-to-actuation latency before claiming a deadline or selecting motor limits.
