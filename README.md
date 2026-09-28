# MPC Demonstrator

Raspberry Pi SPI master and NUCLEO-F446RE firmware for a future MPC controller.
The Pi reads telemetry, computes a command (currently a fixed test setpoint),
then sends it. The MCU runs a motor task every **50 µs**, selecting either current
control or direct voltage. Only the motor is used; there is no pendulum path.

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

| Pi 5 physical pin | Signal | Nucleo-F446RE  |
| ----------------- | ------ | -------------- |
| 19 / GPIO10       | MOSI   | PC3 / CN7-37   |
| 21 / GPIO9        | MISO   | PC2 / CN7-35   |
| 23 / GPIO11       | SCK    | PB10 / CN10-25 |
| 24 / GPIO8, CE0   | NSS    | PB12 / CN10-16 |
| 6                 | GND    | CN7-8          |

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

Alternatively, copy `mcu/build/MPC_Demonstrator.bin` onto the mounted Nucleo drive (`/media/pi/NODE_F446RE` on this Pi).

VS Code uses `.vscode/cmake-variants.yaml` for the MCU board and build type,
and builds in `mcu/build/NUCLEO_F446RE-Develop`. Replace `mcu/build` with that
path in the commands above to share its build directory. Otherwise, the terminal
and VS Code maintain separate build outputs.

Reconfigure CMake after adding/removing library sources; its source glob is
resolved at configure time.

Pi SPI0 must be enabled (`dtparam=spi=on` in the active boot `config.txt`) and
`/dev/spidev0.0` present. No custom kernel or device-tree overlay is required.
Run **only one client at a time**.

Build C++ from the repository root:

```sh
cmake -S host -B host/build -DCMAKE_BUILD_TYPE=Release
cmake --build host/build -j2
```

Run either client from the repository root, one at a time. In Bash, enable
pipeline error reporting so a client failure is not hidden by `tee`:

```sh
set -o pipefail
```

`tee -i` keeps capturing the buffered report when Ctrl+C stops the client:

For C++:

```sh
sudo taskset -c 3 chrt -f 50 host/build/mpc_spi | tee -i host/spi_timing_cpp.txt
```

For Python:

```sh
sudo taskset -c 3 chrt -f 50 python3 -u host/python/main.py | tee -i host/spi_timing_python.txt
```

### Optional CPU-core isolation

CPU 3 can be excluded from normal scheduler load balancing and used for the pinned host control process. Default device interrupts are directed to CPUs 0–2. This can reduce scheduling jitter, although some interrupts and kernel activity can still occur on CPU 3.

Create a backup and edit the kernel command line:

```bash
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.backup
sudo nano /boot/firmware/cmdline.txt
```

Append the following parameters to the existing line:

```text
isolcpus=3 irqaffinity=0-2
```

The file must remain a single line. Reboot afterwards:

```bash
sudo reboot
```

Verify the configuration after rebooting:

```bash
cat /proc/cmdline
cat /sys/devices/system/cpu/isolated
cat /proc/irq/default_smp_affinity
```

The expected isolated CPU is `3`; the expected default interrupt mask for CPUs 0–2 is `7`.

Check whether `irqbalance` is active:

```bash
systemctl is-active irqbalance
```

If it reports `active`, disable it so that it cannot redistribute interrupts onto CPU 3:

```bash
sudo systemctl disable --now irqbalance
```

The existing launch commands explicitly place the host process on the isolated core:

```bash
sudo taskset -c 3 chrt -f 50 host/build/mpc_spi \
  | tee -i host/spi_timing_cpp.txt
```

```bash
sudo taskset -c 3 chrt -f 50 python3 -u host/python/main.py \
  | tee -i host/spi_timing_python.txt
```

CPU isolation is optional: the clients also run without it. It does not provide an interrupt-free core or turn the standard Raspberry Pi OS kernel into a PREEMPT_RT kernel.

The checked kernel (`6.12.109+rpt-rpi-2712`) has `CONFIG_PREEMPT=y` and `CONFIG_NO_HZ_IDLE=y`, but `CONFIG_PREEMPT_RT` and `CONFIG_NO_HZ_FULL` are disabled. Full tickless support is a build-time option: adding `nohz_full=3` alone cannot enable it on this kernel. It requires building or installing a kernel with `CONFIG_NO_HZ_FULL=y`, then selecting CPU 3 with that boot parameter. It can suppress the periodic scheduler tick while a single task runs, but the host's sleep/wakeup and SPI operations still require kernel activity. See the [Linux NO_HZ documentation](https://www.kernel.org/doc/html/v6.12/timers/no_hz.html).

Both run **20,000 cycles by default** (`Trun = 20` seconds of nominal runtime) at a target **1 kHz**, request **30 MHz SPI mode 0**, and
send **0.08 A in current mode after the first zero-setpoint disabled cycle**. They stop automatically; **Ctrl+C** stops early. There are no command-line run options;
edit Python's variables or C++'s defines and rebuild C++ after changes.
The previous command-line options no longer apply.

| Setting                | Python: `host/python/main.py` | C++: `host/src/main.cpp` |
| ---------------------- | ----------------------------- | ------------------------ |
| SPI clock              | `SPI_SPEED_HZ = 30_000_000`   | `SPI_SPEED_HZ 30000000`  |
| Target period          | `PERIOD_US = 1000`            | `PERIOD_US 1000`         |
| Setpoint (A or V)      | `SETPOINT = 0.08`             | `SETPOINT 0.08f`         |
| Mode                   | `MODE = 0`                    | `MODE 0`                 |
| Current-mode limit (A) | `CURRENT_LIMIT_A = 1.0`       | `CURRENT_LIMIT_A 1.0f`   |
| Voltage-mode limit (V) | `VOLTAGE_LIMIT_V = 24.0`      | `VOLTAGE_LIMIT_V 24.0f`  |
| Runtime (seconds)      | `Trun = 20.0`                 | `Trun 20.0`              |
| Reporting interval     | `PRINT_EVERY = 1`             | `PRINT_EVERY 1`          |

Both compute `Nrun = floor(Trun * 1_000_000 / PERIOD_US + 0.5)` (nearest integer,
half rounded up), then preallocate exactly that many samples. Settings must produce
at least one cycle; `PERIOD_US` and `PRINT_EVERY` are integers. Both complete the same cycle count unless interrupted or an
error occurs. Actual elapsed time can differ because of scheduling and overruns.

Both preallocate timing buffers before the run. During execution they only store
samples: no formatting, terminal output, logging thread or queue. After stopping,
they attempt a disabled command, close SPI, and write CSV rows for windows of
`PRINT_EVERY` samples (default 1: every cycle). Set `PRINT_EVERY = 0` for one overall window. The buffers
store timing, motor telemetry and the sent command. Storage is 80 bytes per
configured cycle (about 1.6 MB at the default settings), plus runtime overhead.

Analyze either or both logs from the repository root:

```sh
python3 host/python/analyze_timing.py
```

The script checks `host/spi_timing_python.txt` and `host/spi_timing_cpp.txt`.
These are comma-separated CSV tables despite their `.txt` extension. Errors go to
stderr, separately from the CSV; do not merge stderr into the captured file.
The header is:

```text
cycle,dt_n,dt_min_ms,dt_mean_ms,dt_max_ms,spi_n,spi_min_ms,spi_mean_ms,spi_max_ms,cpu_s,wall_s,time_s,voltage_V,current_A,motor_position_rad,motor_velocity_rad_s,sent_setpoint,sent_enable,sent_mode
```

`cycle` is the final cycle index of the row (starting at 1). With N=1,
min/mean/max are identical for that sample. The first cycle has `dt_n=0` and
`0.0000` interval placeholders because no preceding sample exists. These are
not measurements and are excluded using `dt_n=0`. `cpu_s` and `wall_s`
are whole-run totals repeated on each row; use them once, do not sum them.
Larger N preserves timing-window statistics; data columns then contain the **last
cycle** of that window, not averages. Keep N=1 for identification/controller logs.
`time_s` is host READ-completion time relative to loop start, not an MCU sample
timestamp. The motor data were received **before** the command in the same row was
sent. `sent_setpoint` is in A for `sent_mode=0`, V for `sent_mode=1`; in current mode
it already includes the host's ±1 A clamp (voltage mode: ±24 V). `voltage_V` is the MCU's limited applied
voltage command **before compensation**, not an ADC measurement of terminal voltage.
Motor data and timestamps use six decimals; timing statistics use four. The timing
analyzer accepts both the extended CSV and older timing-only CSVs.

Python (standard library):

```python
import csv
with open('host/spi_timing_python.txt', newline='') as f:
    data = list(csv.DictReader(f))  # Convert numeric strings as needed.
```

MATLAB:

```matlab
data = readtable('host/spi_timing_cpp.txt', 'Delimiter', ',');
```

Old text logs must be regenerated for the CSV analyzer.

It prints final min/mean/p99/max across all recorded samples, weighting window means
by their sample counts. The first window has one fewer interval (`dt_n`) than
SPI samples (`spi_n`); the final partial window is included. Times use four decimal
places, so combined statistics reflect rounding. P99 uses linear interpolation
at sorted index `0.99 * (sample_count - 1)` and requires N=1 logs; it is reported
as unavailable for grouped windows. The first missing interval is excluded.

`dt` measures command-reply completion intervals, including pacing and scheduling.
SPI time sums READ and COMMAND calls, including software overhead and NSS waits;
it is not wire-only timing. CPU usage is process CPU time divided by elapsed run
time (100% means one core), excluding buffer allocation, SPI setup, final disable
and reporting. Timestamping and storing samples still add overhead.

Both use the same single-threaded READ → compute → COMMAND → record → sleep flow.
`chrt` sets FIFO priority 50. Pacing is relative to each cycle's start, without
catch-up bursts; this is not a hard real-time guarantee. `Trun` determines the cycle count, not a wall-clock deadline.

Both reject non-finite controller results before applying command limits. They
attempt a final disabled command after completion, Ctrl+C or execution errors, and
report partial results. C++ also handles SIGTERM; use Ctrl+C for Python.
A failed final-disable exchange returns nonzero. Replies are prepared before command
receipt and do not prove application. MCU command expiry remains independent.

## Motor modes, limits and filters

`MODE=0` requests current in A; `MODE=1` requests voltage in V. Enable remains
independent. The MCU validates mode, enable, finite setpoint and reserved zero.
Both hosts clamp current commands to `CURRENT_LIMIT_A=1.0` and voltage commands
to `VOLTAGE_LIMIT_V=24.0`. Keep the host settings aligned with each other; set the
host voltage limit to the MCU supply minus compensation (currently 24 − 0 = 24 V).
The MCU follows the current reference without a current-setpoint clamp.
This is a current-setpoint limit, **not an instantaneous overcurrent trip**. Voltage
mode bypasses the current controller and current limit, with no setpoint smoothing.

Both modes limit pre-compensation voltage to
`±(MPC_POWERSUPPLY_VOLTAGE - MPC_OFFSET_VOLTAGE)`. PWM is
`(abs(voltage) + MPC_OFFSET_VOLTAGE) / MPC_POWERSUPPLY_VOLTAGE` for nonzero voltage;
zero voltage produces zero PWM. `MPC_OFFSET_VOLTAGE` is configurable, currently
**0 V**, and was previously **2 V**. Its addition is excluded from telemetry.

Switching modes while enabled is supported. On entry to current control, the
setpoint filter is initialized to the actual measured current,
and PID integrator/derivative/output-filter history is initialized from the previous
voltage and current. The first handover tick retains that voltage; subsequent ticks
move toward the requested current. Voltage saturation or a changed setpoint can then
change voltage: this does not promise a perfectly flat physical response.
Current → voltage applies the requested voltage directly; use the existing voltage
as the new setpoint if continuity is desired. Disable/expiry always sets PWM zero,
disables the bridge and resets controller state; re-enabling current mode starts
from zero voltage. Sensor filters and encoder position never reset on mode changes.

The following filters run in the MCU motor task at **20 kHz**, not on the Pi:

- Position: unwrapped encoder counts → radians → optional **680 Hz notch**, damping
  **0.6**. `MPC_POSITION_NOTCH_ENABLED=true` initially; false reports raw position.
- Velocity: wrapped count increment / 50 µs → rad/s → **100 Hz first-order low-pass**
  → separate **680 Hz notch**, damping **0.6**. This follows Mapping_Robot's count-
  increment velocity approach, with radians instead of rotations.
- Current reference: held host command → **500 Hz second-order low-pass**, damping
  **0.9**, using backward Euler. It smooths setpoints; it does not interpolate future
  host commands. At 100 Hz the configured filter adds about 20.5° phase lag.

The measurement filters remain active while disabled and in voltage mode. Include
their dynamics in the MPC model; a notch alone is not a general anti-alias filter
for telemetry sampled by the host at 1 kHz. No raw/filtered sensor history is
reconstructed by the host. Encoder unwrapping assumes fewer than 32,768 counts
between MCU updates; velocity avoids differentiating accumulated float position.
GPA remains an optional current-mode experiment; its MCU-generated excitation
is not constrained by the host current-command limit.

## Code and ownership

| Location                                           | Responsibility                                                                   |
| -------------------------------------------------- | -------------------------------------------------------------------------------- |
| `mcu/include/config.h`                             | MCU pins, periods, priorities and controller settings                            |
| `mcu/src/main.cpp`                                 | Construct hardware/tasks and start them                                          |
| `mcu/lib/MotorControlThread/`                    | 50 µs High2 task: motor sensors, mode handover, control, expiry                  |
| `mcu/lib/IO_handler/`                              | Motor encoder, current ADC, PWM, direction and enable                                 |
| `mcu/lib/SPISlaveDMA/`                             | NSS handlers, immutable DMA frame, latest-command mailbox, High1 recovery worker |
| `host/python/main.py`, `host/src/main.cpp`         | Single-threaded READ → compute → COMMAND loop and timing output                  |
| `host/python/spi_nss.py`, `host/lib/Spi/spi_nss.h` | NSS transport                                                                    |
| `host/lib/Spi/protocol.h`                          | Native SPI wire encoding and reply validation                                    |
| `mcu/tests/`                                       | MCU command-policy regression test                                               |
| `host/tests/`                                      | Python transport tests and disabled-command hardware checks                      |

Libraries inherited from the last commit remain in `mcu/lib/`, including currently
unused utilities. They are retained for future coursework.

The native host keeps `src/` and `lib/Spi/`; Python keeps its client, transport, and timing analyzer
under `host/python/`. Add controller computation at the marked location in `run`,
after reading measurements and before sending the command. The current code is a
constant-current baseline; no estimator or MPC solver is integrated.

The MCU motor task continues to own controller/filter state and outputs. Short
critical sections protect coherent snapshots and the single latest command;
there is no command queue or current-loop mutex.

NSS falling copies the latest published telemetry and arms normal-mode DMA.
The Pi submits one `SPI_IOC_MESSAGE(2)`: a zero-byte **30 µs delay** segment, then
an 18-byte full-duplex burst, with NSS low throughout. NSS rising validates and
publishes the received command. The motor task never writes the active DMA
buffer. Before another exchange the client enforces at least **30 µs NSS high**;
controller computation can cover that interval. NSS settings are in `host/lib/Spi/spi_nss.h`
and `host/python/spi_nss.py`; keep them aligned.

There are exactly two useful bursts per cycle: **READ → compute → COMMAND**.
No ARM transfer or extra signaling is used. Timing margins are minimum waits;
Linux may extend them. The MCU rejects selections held beyond 20 ms. Normal frame
handling and bounded peripheral reset run in the NSS interrupt; the worker polls
for timeout and performs fallback recovery. No HAL polling loop runs in the IRQ.

## Wire protocol and command expiry

Every frame is **18 bytes**: one header, four little-endian float32 values, CRC-8
(poly 0x07, initial value zero, over the first 17 bytes). Firmware and both hosts
must be updated together: the previous 14-byte protocol is incompatible.

| Frame   | Header | Four payload values                                                                    |
| ------- | ------ | -------------------------------------------------------------------------------------- |
| READ    | `0x57` | All sixteen payload bytes zero                                                         |
| COMMAND | `0x55` | Setpoint, enable (0/1), mode (0=current / 1=voltage), reserved zero                    |
| REPLY   | `0x45` | Applied voltage before compensation (V), current (A), position (rad), velocity (rad/s) |

Both exchanges return telemetry selected before receiving that request. COMMAND's
reply is not an application ACK. There is no sequence number or sample timestamp,
so 50 µs sampling is not a guarantee of 50 µs data age at the Pi.

Only CRC-valid commands with finite setpoint, valid mode/enable and reserved zero refresh
the **300 ms command expiry**, measured from MCU receipt. Reads and corrupt frames
do not refresh it. Invalid command values disable output when consumed; a newer
command can overwrite an unconsumed one. Repeated valid commands refresh expiry.
Disable/expiry clears enable and PWM and resets the controller/filter. A non-finite
computed control voltage takes the same reset path; output stays disabled until
a new enabled command arrives. This catches arithmetic overflow without adding
a current limit to the MCU. Direction
and PWM are prepared before asserting enable. GPA mode also requires valid enabled current-mode commands. Expiry depends on the motor task running, and cannot protect against
an MCU stall. The hosts limit current setpoints; the MCU does not. Voltage mode has no current protection. The
reserved fault input is not checked. Physical limits must match the attached motor.

## Validation and diagnostics

Run regression checks from the repository root, without hardware:

```sh
python3 -m unittest discover -s mcu/tests -v
python3 -m unittest discover -s host/tests -v
```

Run the existing disabled-command hardware recovery checker from the repository root:

```sh
sudo chrt -f 50 python3 host/tests/check_spi.py --count 1000 --period-us 500 --faults
```

The checker injects short/extra frames, corrupt CRC, unknown header, cancellation
and held-NSS timeout, then checks 100 normal cycles after each fault at the selected
period. These test framing/recovery,
not physical control or every command's application. Clients stop on bad replies;
restart after a fault. Arbitrary resets are not transparently recoverable.

For student testing, build and flash matching firmware first, run the regression
checks and disabled-command recovery checker above, then run each host separately
and analyze its CSV. A default completed run contains 20,000 data rows with finite
motor values; the first row is disabled and has `dt_n=0`. Default clients enable
the output from the second cycle. For an initial powered-motor check, set
`MODE=1` and `SETPOINT=0` (zero voltage) in the selected host before running,
then verify encoder/current signs and scaling before increasing the command. Mode switching and controller tuning
must be tested on the actual motor; the sensorless bench cannot establish them.

SPI error/recovery counters are available through `SpiSlaveDMA::getDiagnostics()`
for debugging. PB5 marks motor-task execution. Historical timing experiments and comparison scripts are documented in the
development record; the current clients retain the buffered CSV timing recorder.
`MPC_PERFORM_GPA_MEAS` retains the existing optional frequency-response mode; it
has been compile-checked, not validated with a motor.

The full [development record](docs/DEVELOPMENT.md) preserves all available test
reports, unresolved cases, research and local rollback locations. Further testing
should include real controller computation, electrical NSS/SCK timing and physical
sample-to-actuation latency before claiming a deadline or selecting motor limits.
