# Communication development and validation record

This is the consolidated record of the SPI experiments on the Raspberry Pi 5
and NUCLEO-F446RE, through 2026-09-28. It includes unsuccessful experiments and
unresolved cases. The board has no motor or sensor hardware attached; output
register checks are not physical actuator tests. See [README](../README.md) for
current build/run instructions; historical commands below require their matching
firmware and clients.

## CPU-core isolation — 2026-09-28

README documents the optional `isolcpus=3 irqaffinity=0-2` boot settings and
running either host on CPU 3 with `taskset -c 3 chrt -f 50`. The active system
reported isolated CPU `3` and default interrupt affinity mask `7` (CPUs 0–2).
This excludes CPU 3 from normal scheduler load balancing, but some interrupts
and kernel activity remain; an NVMe queue is still assigned to CPU 3.

The checked kernel, `6.12.109+rpt-rpi-2712`, enables `CONFIG_PREEMPT` and
`CONFIG_NO_HZ_IDLE`, but not `CONFIG_PREEMPT_RT` or `CONFIG_NO_HZ_FULL`.
Full tickless support requires a different kernel build. The timing benefit
of isolation has not been established by a controlled comparison.

The subsequent fixed-schedule timing experiment was reverted at the user's
request. Both clients retain pacing relative to each actual cycle start and
the original CSV format and analyzer; only the core-isolation documentation
is retained. No kernel or boot settings were changed during this revert.

## Full review against the original SPI port — 2026-09-26

Baseline: `72ce22c` ("Ported to spi communication from Mapping_Robot project").
Reviewed its final-state difference against `127ce2b` and the fixes below, rather
than treating intermediate experiments as current design. 365 files are unchanged
moves; inherited utility/Eigen code was preserved. The main changes are:

| Area | Original SPI port | Current handover version |
| --- | --- | --- |
| Layout/build | MCU at root, Python under `python/` | Separate `mcu/`, `host/`, `docs/`; independent builds; VS Code targets MCU |
| Host exchange | Python ARM → delay → COMMAND, 5 MHz, 500 Hz | Matching Python/C++ READ → compute → COMMAND, 30 MHz, 1 kHz; kernel NSS setup and enforced inactive interval |
| Firmware ownership | 200 µs intermediary task plus 50 µs current task, mutex-shared state | 50 µs `MotorControlThread` owns sensors/control/enable; atomic latest-command mailbox; SPI recovery worker |
| SPI recovery | ARM/worker-driven preparation | NSS edges frame immutable DMA buffers; length/CRC/header/DMA checks, bounded IRQ reset and worker fallback/timeout |
| Commands | Current/enable/reserved, 14 bytes | Current or voltage/enable/mode/reserved, 18 bytes; receipt-based 300 ms expiry |
| Motor signals | Motor/pendulum positions and current | Motor-only voltage/current/position/velocity; optional position notch; count-derived velocity with 100 Hz LP and notch |
| Control | Current PID, 500 Hz reference LP, fixed 2 V compensation | Original gains/LP retained; initialized live handover; direct voltage mode; configurable compensation defaults to zero; limits ±1 A/±24 V in hosts, voltage saturation in MCU |
| Logging | Unbounded run, live terminal windows | Rounded finite cycle count; preallocated data/timing buffers; CSV after disable/close; MATLAB/Python import and weighted statistics/p99 |

Two numeric failure cases were reproduced and fixed during this review:

- Python's clamp converted a NaN produced after configuration validation into a
  positive-limit command. Both hosts now reject non-finite controller results
  before clamping and use their existing failure/final-disable paths. Normal
  constant settings were already validated; this also protects controller code
  added at the documented computation point.
- Extreme finite current commands could overflow MCU filter/PID arithmetic and
  pass NaN to PWM. The MCU now shares one disable/reset path for expiry, explicit
  disable and non-finite computed voltage. This does not reinstate a current
  limit. Output remains disabled until another enabled command arrives.

The existing CMake and Mbed application configuration are retained unchanged.
Their SD/schema and dependency warnings remain non-blocking with the installed
toolchain. README records that the local Mbed dependency/venv are not tracked by
Git and identifies the tested Mbed revision. Initial powered-motor testing starts
in zero-voltage mode, because zero current is still active current regulation.

Final verification for this review (the runs below used the temporarily simplified
build configuration, before restoring the original build files). Rebuilding with
the restored files passed and produced a byte-for-byte identical firmware binary
to the flashed/tested image:

- Fresh Release host and Develop MCU builds passed. Also compiled GPA enabled,
  position notch disabled and 2 V compensation, then restored/rebuilt defaults.
- All four repository tests, host-limit and N=0/1/2 CSV-parity checks passed.
  Injected NaN/+Inf/−Inf after host configuration validation: both hosts failed
  the run, emitted no enabled command and attempted the final disable. The MCU
  extreme-finite-command test produced 0 non-finite PWM samples out of 200 after
  the fix; ASan/UBSan passed with leak detection disabled. Normal ±2 A handover,
  voltage saturation and host-only current limits still passed. A 22,000-tick
  before/after simulation produced identical valid-input output across mode
  switches, signed references, enable/disable, voltage saturation and expiry.
- Flashed and verified final default firmware. Hardware recovery passed 1,000
  exchanges and three fault rounds (seven malformed-frame cases plus cancellation
  and timeout, 100 good exchanges after each). Both hosts passed normal completion
  and Ctrl+C/partial logging. Positive/negative/zero voltage, 100 mode-switch pairs,
  invalid-mode shutdown and expiry while reading passed.
- On the sensorless MCU bench, ±float32-maximum current requests returned finite
  zero-voltage telemetry at 20 ms, before command expiry. A later valid voltage
  command restored output; the test ended with a disabled command. This validates
  the numeric failure path, not physical motor protection.

Final timings: core 3, FIFO 50, 30 MHz; 20,000 current-mode cycles and 5,000
voltage-mode cycles per host. Values are ms; CPU is percent of one core.

| Client / mode | dt min | dt mean | dt p99 | dt max | SPI min | SPI mean | SPI p99 | SPI max | CPU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C++ current | 0.8707 | 1.0015 | 1.0076 | 1.1982 | 0.1515 | 0.1594 | 0.1659 | 0.3357 | 5.51% |
| C++ voltage | 0.9869 | 1.0015 | 1.0072 | 1.0384 | 0.1513 | 0.1593 | 0.1644 | 0.1949 | 5.50% |
| Python current | 0.2263 | 1.0024 | 1.0145 | 1.8750 | 0.1613 | 0.1672 | 0.1780 | 0.9903 | 6.59% |
| Python voltage | 0.9376 | 1.0022 | 1.0092 | 1.0571 | 0.1616 | 0.1669 | 0.1727 | 0.2362 | 6.55% |

Artifacts: `/tmp/mpc-baseline-review-EaFauI/`. No test framework or generated logs
were added to the repository. These results do not establish hard deadlines; the
older 57 µs MCU startup observation was not remeasured. GPA code and its option
remain unchanged; GPA was compile-checked, not validated on a physical plant.
Known GPA reset-state and duplicate-frequency-grid edge cases remain outside
these fixes. Motor/sensor calibration,
closed-loop stability and physical moving-mode transients remain unverified.

## Student handover review — 2026-09-26

Reviewed the staged motor-mode, telemetry, filter, host logging and
`MotorControlThread` rename changes together. Working tree matched the index
before this review. No further functional change was needed. Review edits clarify
the PID preload comment, correct the Nucleo mount path and motor-task wording,
and add a student test sequence to README.

Both projects were configured in empty build directories under
`/tmp/mpc-handover-ffDAr2/` (C++ Release; MCU NUCLEO_F446RE/Develop). All 310 MCU
build steps completed; the freshly built firmware was flashed via OpenOCD and
verified. This supersedes the earlier note that the host-only-current-limit
revision had not been flashed. Both defaults remain 20 seconds and current mode.

All four repository tests passed. Temporary checks passed Python/C++ CSV parity
for N=0/1/2, host command clamping and recorded/transmitted values, unclipped ±2 A
MCU handover, voltage saturation, filter reset/frequency response and 400,000
bidirectional encoder updates. MCU simulation/filter/encoder checks used ASan and
UBSan with leak detection disabled. On hardware, 1,000 exchanges at 500 µs and
three rounds of seven malformed-frame cases plus cancellation/timeout passed,
including 100 valid exchanges after each fault. Both hosts completed 20,000
current-mode cycles and 5,000 voltage-mode cycles, with valid CSV and no reported
SPI errors. Positive/negative/zero voltage, 100 enabled mode-switch pairs,
invalid-mode disable and expiry while READ continued passed. Ctrl+C checks passed
for both clients: exit zero, partial CSV and zero-voltage telemetry afterward.

Host timing used core 3, FIFO 50, 30 MHz SPI, without concurrent builds. Times are
ms, CPU is percentage of one core; these are observations, not deadline guarantees.

| Client / mode | dt min | dt mean | dt p99 | dt max | SPI min | SPI mean | SPI p99 | SPI max | CPU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C++ current | 0.9088 | 1.0015 | 1.0095 | 1.1272 | 0.1531 | 0.1596 | 0.1675 | 0.2758 | 5.52% |
| C++ voltage | 0.9610 | 1.0014 | 1.0076 | 1.0562 | 0.1505 | 0.1593 | 0.1636 | 0.2041 | 5.49% |
| Python current | 0.8467 | 1.0023 | 1.0149 | 1.1911 | 0.1609 | 0.1674 | 0.1809 | 0.3433 | 6.60% |
| Python voltage | 0.9369 | 1.0022 | 1.0076 | 1.0861 | 0.1594 | 0.1670 | 0.1727 | 0.2483 | 6.55% |

Remaining findings: the clean configure emits existing schema warnings about the
STORAGE/SD list overrides and ignores the undefined `sd.init-frequency` setting.
Dependency compilation reports two unused-variable warnings in STM32 system clock
code and a littlefs string-initializer warning. These are outside the staged
functional changes and did not prevent the build; dependencies/configuration were
left unchanged. The earlier 57 µs first-disabled-iteration timing result remains
a known limitation, not remeasured here. No motor/sensors are attached: physical
current regulation, polarity, filter response on the plant and moving-mode
transients remain student bench validation tasks. This review establishes readiness
for those tests, not 100% reliability or hard real-time operation.

Fresh build logs, test scripts and four timing CSVs are in
`/tmp/mpc-handover-ffDAr2/`; no test framework or generated logs were added to Git.

## Motor modes and telemetry — 2026-09-26

This section supersedes the earlier current-only, 14-byte protocol below.
The current protocol uses 18-byte frames: COMMAND carries setpoint, enable,
mode (0=current, 1=voltage), reserved zero; REPLY carries pre-compensation voltage,
current, motor position and motor velocity. Both hosts and MCU must be updated
together. There is no pendulum signal in this version.

Current setpoints are clamped to ±1 A in the hosts only. Both hosts also clamp
voltage commands to ±24 V, matching MCU supply minus compensation. The MCU retains
voltage saturation but no current-reference clamp, including for GPA excitation.
Voltage mode bypasses current regulation and current limiting. Compensation is
configurable, defaults to 0 V (previously 2 V), applies in both modes and is excluded
from voltage telemetry. Telemetry reports the limited voltage command, not an ADC
measurement of motor-terminal voltage.

On entry to current mode, the setpoint filter starts from actual measured current; PID history tracks the previous voltage. The first handover tick
holds that voltage. Later ticks respond to the new reference and limits. Entry to
voltage mode applies the requested voltage directly. Disable/expiry takes priority
and clears output/controller state. Encoder and measurement filters remain
continuous through mode changes. This supports moving-state handover in software;
physical stability and transients still require validation with the real motor.

Motor velocity uses wrapped encoder increments and a 100 Hz first-order low-pass,
following Mapping_Robot's approach, then a separate 680 Hz notch. Position has an
optional 680 Hz notch, enabled by default. The existing 500 Hz second-order
backward-Euler current-reference filter is retained (about 20.5° lag at 100 Hz).
The IIR default constructor now initializes a valid identity filter: its previously
uninitialized order could cause invalid indexing before an explicit filter init.
No other filter coefficient changes were needed. README documents the full chain.

Both hosts retain the same single-threaded READ → compute → COMMAND → wait flow,
20,000-cycle default and deferred output. Matching CSV columns now include host
read time, four motor signals and the command subsequently sent. N=1 retains every
sample; grouped windows retain the final cycle's data. The analyzer accepts both
old timing-only and extended CSVs. Memory is about 1.6 MB for the default buffers.

Follow-up validation after moving current limiting to the hosts: both builds and
all four existing command/transport tests passed. Temporary tests ran the actual
Python and C++ loops with mocked SPI for commands −100, 0 and +100 in both modes:
transmitted and logged commands matched ±1 A / ±24 V limits, with the initial
cycle disabled. The actual MCU loop with simulated IO passed handover at measured
currents ±2 A, followed references above 1 A without clipping, and retained ±24 V
saturation and disable behavior. ASan/UBSan passed with leak detection disabled.
At that stage this follow-up was built and software-tested, not flashed or
hardware-timed. The student handover review above subsequently flashed and tested it.

Validation below preceded the follow-up move of current limiting to the hosts
and addition of host voltage limits; its hardware timing figures describe that
earlier revision.

Validation performed:

- Native host and MCU builds passed, including an optional GPA-enabled build.
  Default GPA-disabled firmware was restored afterward. Existing MCU command-policy test and three
  host transport tests passed; command tests now cover both modes and invalid mode.
- Temporary tests exercised the actual MCU loop with simulated moving encoder and
  current inputs: both handover directions, disable/re-enable, ±1 A limiting,
  voltage saturation and expiry. Actual encoder code passed 400,000 forward/reverse
  updates across 16-bit counter wraps. ASan/UBSan reported no errors (leak detection
  disabled because the sandbox does not support it).
- Filter checks passed steady-state reset, notch rejection, differentiator reset,
  default identity and PID preload checks. Measured LP1 gain at 100 Hz was 0.707102.
  Config variants passed with position notch bypassed and compensation set to 2 V,
  including zero PWM and compensation-free voltage telemetry.
- Python/C++ CSV output matched byte-for-byte for N=0, 1 and 2 with identical input.
  Mocked Python execution checked 18-byte frames, current clamping, voltage bypass
  and recorded commands; extended CSV analysis passed.
- On normal firmware, 1,000 disabled exchanges at 500 µs passed. Three recovery
  rounds each injected seven frame faults, cancellation and timeout, with 100 valid
  exchanges after each. Direct positive/negative/zero voltage, 100 enabled mode
  switch pairs, invalid-mode disable and 300 ms expiry while READ continued passed.

Final normal-firmware runs used FIFO 50, core 3, 30 MHz SPI and 5,000 cycles each.
These are functional timing checks, not a controlled performance comparison;
software checks ran concurrently during part of the session. Times below are ms;
CPU is percentage of one core. Each run had 4,999 dt and 5,000 SPI samples.

| Client / mode  | dt min | dt mean | dt p99 | dt max | SPI min | SPI mean | SPI p99 | SPI max |   CPU |
| -------------- | -----: | ------: | -----: | -----: | ------: | -------: | ------: | ------: | ----: |
| C++ current    | 0.9118 |  1.0022 | 1.0287 | 1.1185 |  0.1550 |   0.1614 |  0.1885 |  0.2604 | 5.75% |
| C++ voltage    | 0.9454 |  1.0019 | 1.0238 | 1.0752 |  0.1515 |   0.1605 |  0.1830 |  0.2316 | 5.66% |
| Python current | 0.9135 |  1.0024 | 1.0174 | 1.1233 |  0.1605 |   0.1675 |  0.1831 |  0.2688 | 6.65% |
| Python voltage | 0.9331 |  1.0023 | 1.0141 | 1.0623 |  0.1613 |   0.1678 |  0.1819 |  0.2533 | 6.60% |

Temporary MCU instrumentation measured work after thread wakeup through output
update, including intervening interrupts, at 1 µs resolution. Current mode:
202,490 iterations, mean 16.5883 µs, max 25 µs; voltage: 208,901 iterations,
mean 15.5440 µs, max 24 µs; current entry: 102 iterations, mean 16.6176 µs,
max 24 µs. None of these enabled paths reached 50 µs. Disabled operation had
one 57 µs iteration. A second startup test reproduced it on the **first disabled
iteration**, with no further ≥50 µs event in 44,585 subsequent iterations. This
identifies when it occurs, not its internal cause, and is not a universal deadline
guarantee. Instrumentation was removed and normal firmware was rebuilt, flashed
and verified before the final host checks. The board has no motor/sensors attached,
so bench telemetry checks cannot validate physical moving-motor behavior.

Temporary validation artifacts are in `/tmp/mpc-mode-checks-a27egc0i/`,
`/tmp/mpc-mode-hardware-na6rerkb/`, `/tmp/mpc-mode-profile-check-rg0b3pn4/` and
`/tmp/mpc-mode-final-n76pqu06/`; they are not repository dependencies.

## Repository layout note

Historical paths below refer to the layout used during each experiment. Firmware
now lives under `mcu/`, Python clients under `host/python/`, and host checks under
`host/tests/`. Local checkpoints moved from `build/checkpoints/` to
`mcu/build/checkpoints/`. Use the root [README](../README.md) for current commands.

## Host baseline evolution — 2026-09-26 (historical steps)

Initially, Python `main.py` was restored byte-for-byte from commit `adeb639`. The native host
kept `src/`, `include/`, and `lib/Spi/`, and followed Python's simple single-threaded
READ → compute → COMMAND → print → sleep loop. At that stage both ran continuously, first sent
a disabled command, then enabled the 0.08 A placeholder. Settings are Python
variables/C++ defines; the threaded clients' run options and reporting queues are
removed. Scheduling is selected externally with `chrt`. The MCU is unchanged.

A temporary, instrumented comparison used three 10,000-cycle runs per variant,
1 kHz, FIFO 50, reporting off, and disabled commands throughout. Mean cycle
intervals were 1002.59–1002.90 µs for the original loop, 1005.37–1005.77 µs for the
threaded Event.wait loop, and 1002.23–1002.40 µs when only that wait was replaced
with time.sleep. Mean work stayed around 163–165 µs. Occasional long intervals
also occurred in the original loop; these runs do not establish their cause.
Temporary scripts and CSVs are in `/tmp/mpc-python-compare-ql6gs97q/` on this Pi.
This comparison used disabled commands and differs from the live-output checks.

Final hardware checks ran each client for 12 seconds with FIFO 50 and default
live printing, then sent one SIGINT directly to the client. Both exited zero
without reported SPI or final-disable errors. From the 23 printed windows per
client (11,499 intervals; the final partial window is excluded):

| Client | Mean interval (ms) | Maximum interval (ms) |
| ------ | -----------------: | --------------------: |
| python |           1.001988 |              1.106390 |
| cpp    |           1.001495 |              1.045112 |

Those logs were subsequently replaced by newer runs; the table records the
results of that historical check.
An earlier repeat using `timeout --signal=INT` reported an interrupted Python
final-disable attempt despite exit zero; its cause was not established. The
original Python shutdown behavior was retained at that stage. Native build/protocol test and
all six Python tests passed; MCU sources were unchanged.

The subsequent minimal reporting update moves C++ application settings into
`host/src/main.cpp` and NSS constants into the transport header, removing the
separate config header. Both clients add per-window SPI-call statistics and
process CPU/wall seconds. `host/python/analyze_timing.py` summarizes sample-weighted
interval/SPI statistics and elapsed-time-weighted CPU usage. Current logs are
`host/spi_timing_python.txt` and `host/spi_timing_cpp.txt`; earlier results above
predate this instrumentation.

The current host now defaults to `Trun = 20` seconds. Both clients preallocate
timing buffers and report only after stopping and closing SPI, including partial
windows. CPU timing covers execution rather than reporting. Earlier measurements
above used live printing and do not characterize this buffered implementation.

The latest host conversion uses a rounded integer cycle count from `Trun` and
`PERIOD_US` (20,000 cycles by default), rather than a wall-clock cutoff. Default
`PRINT_EVERY = 1` exports every cycle after shutdown as CSV with unit-bearing
headers. Run CPU/wall totals are repeated metadata, counted once by the analyzer.
Historical text logs and measurements above predate this format.

## Current host and commit validation

The current Python/C++ baseline has the same settings and sequence: READ → compute
→ COMMAND → record → sleep. Application settings live in each main file; NSS
settings live in the transport files. No separate host config header, logging
thread, queue or controller framework is needed. MCU files are unchanged.

Both round `Trun * 1e6 / PERIOD_US` to the nearest integer (halves up), allocate two
arrays before opening SPI, and complete that many cycles unless interrupted or an
error occurs. The first command is disabled at zero current; subsequent commands
use the 0.08 A placeholder. Final disable and SPI close precede CSV formatting.
Default settings are 20 seconds nominal, 1000 µs, 30 MHz, and one cycle per CSV row.
Actual elapsed time is not fixed. A controller belongs between READ and COMMAND.

CSV field units, Python/MATLAB import examples, root-directory run commands and
analyzer usage are in the [README](../README.md). P99 is linearly interpolated
from individual samples; grouped logs cannot provide it. CPU/wall values are
whole-run metadata, counted once. Times are rounded to four decimals. The first missing interval uses zero placeholders with `dt_n=0`, excluded from
statistics. Empty runs produce only a header. Logs do not contain a target-count or success marker:
check the process exit status and final cycle number, particularly after errors.
`tee -i` preserves buffered output on Ctrl+C; Bash `pipefail` preserves failures.

The native `protocol_test` and Python `test_protocol.py` were removed at the user's
request; older test counts below are historical. The default build now contains
only `mpc_spi`. Transport unit tests and the disabled-command recovery checker
remain. Final review checks use temporary files, without adding test infrastructure.

Before the final review fixes, sequential 60,000-cycle FIFO-50 hardware runs exited
cleanly and produced these results (intervals include pacing; SPI sums both calls):

| Client | Wall (s) | Interval mean/p99/max (ms) | SPI mean/p99/max (ms)    | CPU, one core |
| ------ | -------: | -------------------------- | ------------------------ | ------------: |
| C++    |  60.0927 | 1.0015 / 1.0071 / 1.2574   | 0.1552 / 0.1604 / 0.3654 |         5.52% |
| Python |  60.1471 | 1.0024 / 1.0153 / 1.2196   | 0.1638 / 0.1800 / 0.4834 |         6.56% |

These are observations, not deadline guarantees or physical actuator tests.

Final review fixed only concrete issues: malformed/missing/duplicated CSV rows
are rejected; loop errors are reported after the disable attempt; invalid settings
are rejected before running; Python rejects unsupported CLI options; C++ detects
CSV stream failures. Documentation now uses `tee -i` and Bash `pipefail`.

Validation: clean Release build with warnings treated as errors; three host
transport tests and one MCU command-policy test passed. Temporary checks confirmed
byte-identical Python/C++ CSV for N=0/1/2, partial windows, rounded cycle counts,
p99 interpolation, CPU metadata counted once, invalid CSV/settings rejection,
and no output during the loop. Injected Python loop failure attempted disable
and closed SPI before error/report output. No removed tests were reintroduced.

Both final default hardware runs completed 20,000 cycles and exited zero:

| Client | Wall (s) | Interval mean/p99/max (ms) | SPI mean/p99/max (ms)    | CPU, one core |
| ------ | -------: | -------------------------- | ------------------------ | ------------: |
| C++    |  20.0323 | 1.0016 / 1.0091 / 1.1389   | 0.1554 / 0.1631 / 0.2567 |         5.54% |
| Python |  20.0508 | 1.0025 / 1.0166 / 1.5828   | 0.1637 / 0.1774 / 0.7439 |         6.59% |

Process-group SIGINT checks through `tee -i` also exited zero and preserved valid
partial CSVs (1,995 C++ and 1,974 Python samples), without reported SPI or shutdown
errors. Default logs contain the final 20,000-cycle runs; the preceding 60,000-cycle
logs were copied to `/tmp/mpc-final-review-60s-{cpp,python}.txt` on this Pi.
Temporary validation files are not committed. MCU sources and firmware were not
changed or flashed. MATLAB import is documented but was not executed here.

## CPU affinity comparison — 2026-09-26

Six balanced rounds compared C++/Python, unpinned/core 3: 24 sequential runs of
10,000 cycles each at 1 kHz, FIFO 50, default 0.08 A placeholder, buffered N=1 CSV.
Three shuffled orders (seed 5087) were each followed by their reverse. Scheduler,
priority and affinity were verified for every process: unpinned CPUs 0–3 versus
pinned CPU 3. No deliberate background load or CPU isolation was applied. The
10-second nominal runtime used temporary C++ source/binary and a Python runtime
override; repository defaults and existing logs were not changed. All runs exited
zero with exactly 10,000 samples; no interval exceeded 2 ms.

The table reports medians of six per-run statistics, except the worst column,
which is the largest interval across all six runs. It is not a pooled p99.

| Client | Affinity | Median interval mean (ms) | Median interval p99 (ms) | Worst interval (ms) | Median SPI p99 (ms) | Median CPU |
| ------ | -------- | ------------------------: | -----------------------: | ------------------: | ------------------: | ---------: |
| C++    | Unpinned |                    1.0015 |                   1.0061 |              1.0895 |              0.1586 |      5.52% |
| C++    | Core 3   |                    1.0014 |                   1.0045 |              1.1104 |              0.1576 |      5.48% |
| Python | Unpinned |                    1.0024 |                   1.0124 |              1.2120 |              0.1736 |      6.53% |
| Python | Core 3   |                    1.0022 |                   1.0076 |              1.1234 |              0.1689 |      6.46% |

Pinned p99 was lower in 4/6 C++ and 5/6 Python pairs. Pinned maximum was lower in
only 2/6 C++ and 4/6 Python pairs. Intervals above 1.1 ms totaled 0 versus 1 for
C++, and 7 versus 1 for Python (unpinned versus pinned; 59,994 intervals per
condition). This supports optional pinning for a modest typical-tail improvement,
especially Python, not a claim of reliable worst-case improvement or core
exclusivity. Six short repeats on this Pi do not establish behavior under a future
controller workload or explain the earlier 4 ms outlier. Default commands remain
unpinned; README includes optional pinned commands.

Raw CSVs, run order, temporary native source/binary and per-run results are in
`/tmp/mpc-affinity-study-rkoh3vno/` on this Pi; these temporary artifacts are not
committed. No source/runtime changes were required by the comparison.

## Current decision

Keep NSS-framed normal DMA with immutable telemetry, two useful exchanges per
cycle (READ → compute → COMMAND), and 30 µs setup / minimum NSS-high margins.
Keep the 50 µs High2 current task and High1 SPI recovery worker. There is a single
latest command, no FIFO. CRC-8 and 14-byte payloads remain unchanged. Normal
communication needs only MOSI, MISO, SCK, NSS and ground.

At a 1,000 µs target cycle, measured C++ communication work averaged 155 µs,
leaving about 845 µs for computation and scheduling margin. This is an average
budget, not a guaranteed available compute time. Computation can also cover the
READ-to-COMMAND minimum high interval. Linux stalls can exceed the remaining
budget. No MPC computation or physical sample-to-actuation measurement is included.

## Lessons and rejected shortcuts

- **Thread abstraction:** the generic RealTimeThread experiment coincided with
  the previously working 100 µs ARM gap needing about 200 µs. Removing its wake-up
  mutex alone did not restore operation. Reverting inheritance restored the
  user's 100 µs test. The precise causal contribution was not isolated; mutex
  overhead alone was never established as the explanation. Preserve the direct
  current-task/Ticker implementation instead of repeating that refactor.
- **Two periodic tasks:** DMA does not eliminate computation deadlines. The
  separate 200 µs forwarding task did add delay and shared-state overhead, so it
  was removed. The current task now owns sensor sampling, controller state,
  output ordering and expiry; SPI publishes commands directly to its mailbox.
- **COBS:** byte-stream resynchronization did not address the observed SPI
  arming/recovery problem. NSS and fixed frame length already delimit exchanges.
  COBS was discussed and rejected, not implemented or benchmarked. CRC-8 was
  retained; it does not detect all corruption or establish freshness.
- **One full-duplex exchange:** it can carry telemetry and a prior command
  simultaneously, but cannot carry a command computed from that same incoming
  telemetry. Two useful exchanges avoid that intentional sample-period delay.
- **ARM gaps:** they cover software preparation/scheduling as well as wire time;
  they are not a simple function of message length. Four frames needed guards
  after payloads too. Clean reply CRCs hid MCU recovery events and a missed command.
- **Language choice:** C++ reduced host CPU substantially but initially reduced
  mean work only about 2%. It did not eliminate the approximately 50 µs variation.
  Shortening the MCU timing path then cut mean work from about 385 to 238 µs;
  NSS interrupt handling brought it to about 155 µs and reduced ordinary jitter.
- **FIFO/ring/double DMA buffers:** buffer capacity does not establish ownership.
  NDTR completion, NSS edges and current-loop publication are different events.
  DMA can consume memory while the CPU is changing it, including prefetched data.
  The final solution copies a coherent latest snapshot into a separate DMA buffer.
- **Real-time Linux:** FIFO scheduling is not PREEMPT_RT. PREEMPT_RT, memory
  locking, CPU/IRQ affinity and direct RP1 drivers were researched, not adopted
  or benchmarked here. PCIe ASPM and GPIO polling experiments did not remove the
  ordinary jitter. The original ASPM policy was restored.
- **Measure the right quantity:** wire duration, snapshot age, host work,
  completion interval, command receipt and physical actuation are different.
  Neither a valid reply nor a low average establishes a command ACK or deadline.
  Instrumentation can introduce pauses; confounded samples were excluded openly.
- **Recovery matters:** normal-traffic success did not imply malformed-frame
  recovery. Stopping DMA alone left stale SPI state. Bounded peripheral reset and
  worker fallback are retained; expensive HAL waits stay outside the NSS handler.

## Evidence index

1. [Current NSS implementation and hardware results](#nss-results)
2. [Legacy ARM, four-frame, Python profiling and task cleanup experiments](#arm-results)
3. [Python versus native C++ comparison](#host-results)
4. [MCU microsecond-ticker optimization](#ticker-results)
5. [External project and vendor/kernel research](#research)
6. [Architecture checkpoints and earlier acceptance checks](#checkpoints)
7. [Student cleanup and final review](#student-cleanup)

Sections 2–6 are historical records, preserved with their original figures and
limitations. Words such as “current”, “next” or “remains flashed” in those sections
refer to that checkpoint, not today's build. In particular REQUEST/READY, ARM,
libgpiod and spidev Python-package instructions are superseded by NSS. Do not mix
clients and firmware across those checkpoints. At that historical checkpoint Python PRINT_EVERY was 500;
historical tests that mention 100 used that earlier setting.

The original working-tree baseline is local `origin/main` / HEAD `72ce22c`
(“Ported to spi communication from Mapping_Robot project”). No commit, staging
change, kernel installation or boot change was made by the student cleanup.
All available `/tmp/mpc-*` experiment directories/files were copied to
`build/checkpoints/pre-student-cleanup/experiments/` before cleanup. These are
local, Git-ignored recovery artifacts; the results in this document are portable.
Older source and firmware snapshots remain in `build/checkpoints/`.

<a id="nss-results"></a>

## Current NSS implementation and hardware results

### NSS transport validation — 2026-09-25

Implemented and flashed on the Pi 5 / NUCLEO-F446RE SPI2 setup, without motor
or sensor hardware. The C++ and Python clients now use READ → compute → COMMAND,
with two useful 14-byte full-duplex bursts per cycle. CRC-8 and payloads are
unchanged. No ARM message or REQUEST/READY GPIO is used.

#### Implementation and timing contract

Each exchange uses one `SPI_IOC_MESSAGE(2)`: a zero-byte segment requesting a
30 µs delay with NSS asserted, then the payload, without a chip-select change
between segments. The client also enforces at least 30 µs NSS high after ioctl
completion before starting another exchange. This worked with the existing Pi
kernel; no device-tree or boot changes were made.

NSS falling copies the latest coherent published telemetry into a dedicated TX
buffer and arms normal-mode DMA. That buffer remains immutable during transfer.
NSS rising checks completion, SPI/DMA errors, framing and CRC before publishing
a command. There is no circular buffer, command FIFO, or intentional control-period
pipeline delay. The 50 µs High2 current task is retained. Normal SPI handling and
bounded peripheral resets run in the NSS handler; the High1 worker handles the
20 ms selected-transaction timeout and fallback recovery.

An early prototype deferred every malformed-frame reset to the worker and failed
immediate recovery after a short frame. The accepted version resets the selected
SPI peripheral in the handler when DMA has stopped; otherwise it defers recovery.
There is no HAL polling loop in the handler. Early 20 µs setup / 10 µs high tests
passed ordinary traffic, but the measured completion handler could exceed 10 µs.
The defaults are therefore 30/30 µs, not those smaller experimental settings.

#### Matched C++ comparison

Three alternating repetitions per firmware and rate, 10,000 cycles per run:
120,000 total cycles, all passed. Both used `chrt -f 50`, requested 30 MHz,
enabled 0.08 A test commands after the first disabled cycle, no periodic output,
and no controller computation. No builds ran during the comparison.
The baseline is the previously working, ticker-optimized REQUEST/READY firmware.

Times below are µs. Percentiles pool samples across the three runs; CPU is the
mean reported percentage of one core. `dt` is command-reply completion interval;
`work` spans the READ start through COMMAND completion.

| Transport     | Target period | Mean work | p99 work | Max work |   p99 dt | p99.9 dt |   Max dt |    CPU |
| ------------- | ------------: | --------: | -------: | -------: | -------: | -------: | -------: | -----: |
| REQUEST/READY |          1000 |   238.280 |  269.037 |  408.074 | 1050.536 | 1089.944 | 1171.666 |  4.22% |
| NSS           |          1000 |   155.269 |  160.352 |  280.630 | 1007.555 | 1022.111 | 1123.426 |  5.56% |
| REQUEST/READY |           500 |   236.268 |  296.814 |  440.185 |  550.593 |  593.833 |  719.963 |  8.41% |
| NSS           |           500 |   155.290 |  158.167 |  209.481 |  504.759 |  512.889 |  555.685 | 11.02% |

Mean READ duration fell from about 107 µs to 60 µs. Mean work fell about 35%.
No measured work exceeded its target period. Mean completion intervals remained
about 1001.6 / 501.5 µs because scheduling is relative to each cycle's start.
The NSS-high busy-wait increases CPU use; future controller computation between
READ and COMMAND can cover some or all of that interval.

#### Final hardware acceptance checks

- Production C++: 10,000 disabled cycles at 2 kHz, then 90 injected faults and
  9,000 valid recovery cycles. Python: 1,000 disabled cycles at 2 kHz, then the
  same 90 faults / 9,000 recovery cycles. Both passed. Faults cover 1/7/13/15/28
  bytes, bad CRC, unknown header, cancellation without clocks, and held selection
  beyond timeout. Recovery exchanges use normal margins, without extra sleeps.
- A temporary instrumented build of the final MCU sources passed 10,000 enabled
  cycles at 2 kHz, then 1,000 disabled cycles plus that fault matrix. MCU command
  counters reached exactly 20,002, including both final disables; 90 rejected
  transactions and ten timeouts were counted, with no arm failures or ignored
  NSS assertions. This checks receipt separately from the telemetry reply.
- Across 308,642 additional instrumented current-task iterations, skipped releases
  stayed zero and the late-finish counter stayed at its single startup event.
  Maximum execution remained 57 µs from startup; maximum release-to-task delay
  remained 20 µs. Thus the test added no observed overruns; it does not claim that
  every iteration, including startup, meets 50 µs.
- Maximum measured NSS preparation was 12 µs and completion handling 13 µs.
  These exclude edge-to-ISR latency. Maximum age of the last published telemetry
  at the preparation handler was 58 µs. Publication age excludes sensor acquisition,
  subsequent setup/clocks and Pi delay; it is not total sensor-to-host age.
- Non-halting ST-LINK register reads verified enable low and PWM zero following
  silence, READ-only traffic and bad-CRC traffic past command expiry. Fresh valid
  commands maintained enable; disable and invalid current/enable/reserved payloads
  cleared outputs. No motor was attached.
- Reset during NSS-low/no-clocks recovered on fresh transactions. Normal client
  completion, SIGINT and SIGTERM cleared enable/PWM and allowed a client reopen.
  This is not transparent recovery of an interrupted command or a debugger halt.
- Normal firmware was restored after instrumentation and passed final acceptance.
  It remains flashed, with test clients closed.

With three additional bounded CPU-busy processes on the Pi, one 10,000-cycle run
at each rate also passed:

| Target period | Mean work | p99 work | Max work |   p99 dt |   Max dt |    CPU |
| ------------- | --------: | -------: | -------: | -------: | -------: | -----: |
| 1000          |   156.209 |  169.482 |  312.425 | 1014.130 | 1160.573 |  5.64% |
| 500           |   155.501 |  163.556 |  184.981 |  509.241 |  527.871 | 11.10% |

Both reported zero work-over-budget events. This load is CPU-only, not a complete
stress test of storage, networking, IRQ interference or thermal behavior.

MCU and native builds passed, as did the nine Python regression tests and native
protocol test. Top-level MCU CMake was not changed. Standalone host CMake no longer
requires libgpiod; the old handshake source files were subsequently removed during student cleanup.

#### Limits and next check

These results support using this transport for the next controller experiment.
They do not establish hard real-time guarantees on the current Linux system.
Rare scheduling stalls remain possible. The requested SPI clock and NSS/SCK
timing have not been checked with a logic analyzer; simultaneous electrical timing
and MCU timing measurements are still needed to establish preparation margin.

There is no command-application ACK, sequence number or sample timestamp. A valid
COMMAND reply contains telemetry selected before command receipt. CRC-8 detects
many errors but does not prove freshness. Arbitrary partial-bit/electrical faults
and all interrupt-latency extremes are not covered. Only the wired SPI2 pin/DMA
configuration was hardware-tested. The 300 ms expiry depends on the current task
running and cannot protect against a stalled MCU.

The next useful measurement includes representative controller computation and,
when attached, physical sample-to-actuation timing. Further shaving of setup/high
intervals should wait for a measured worst-case preparation/completion margin.

#### Reproduce and recover

```sh
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 1000 --enable --csv /tmp/jitter-nss.csv
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 500 --faults
sudo chrt -f 50 python3 python/check_spi.py --count 1000 --period-us 500 --faults
```

Run only one client at a time. Full raw comparison CSVs, logs, diagnostic sources
and test scripts are saved locally under `build/checkpoints/nss-f3aeffbf/validation/`;
the original scratch location is `/tmp/mpc-nss/`. Scripts there document this
session's hardware addresses and paths and are not a portable test framework.

Production firmware SHA-256:
`f3aeffbff2a423b644f1fd0ea53e352b311a34e2c70db0cc4728afee75d15169`.

Previous REQUEST/READY firmware SHA-256:
`cdf8d67b2a0073c09adc5c57e6eea8989f9c03090e3442b7c12bdc1e837d3619`.
Its binary, client and source checkpoint remain in
`build/checkpoints/request-ready-cdf8d67b/`. Stop clients, flash that image and use
its matching saved client to roll back; REQUEST/READY wires are required again.


<a id="arm-results"></a>

## Historical ARM and Python profiling experiments

### SPI transport comparison — 2026-09-25

#### Decision summary

The proposed four-transfer cycle with only two ARM gaps is not reliable in this
experiment. It also needs time between READ completion and the next ARM, and
between COMMAND completion and the next ARM. Otherwise DMA may not be rearmed
before that ARM arrives. At 300 us ARM gaps it can return valid replies despite
thousands of MCU recovery events and a missed command. Reply CRC is not an ACK.

With a post-payload guard, the approach has tighter successful-cycle timing at
1 kHz, at the cost of substantially more Pi CPU time and reduced cycle budget.
100/100 us was clean in normal traffic but failed recovery tests. 150/150 us is
the smallest tested setting to pass the independent fault matrix. This is not a
worst-case margin guarantee. Current REQUEST/READY firmware remains the default;
all experimental firmware was isolated and the current image restored afterward.

#### What was held constant

The new alternative retains the current two-task architecture, current-control
code, priorities, 300 ms command expiry, CRC-8, READ/COMMAND payloads and peripheral
reset recovery. Only the SPI worker's arming trigger and host transport changed.
The firmware does not configure or read REQUEST/READY GPIOs. Existing wires were
left connected; do not disconnect them while using the restored handshake firmware.

The experimental sequence is:

`ARM → ARM gap → READ → compute/guard → ARM → ARM gap → COMMAND`

A minimum guard is enforced from the end of every payload until the next ARM.
At 1 kHz the inter-cycle sleep normally covers the guard after COMMAND; at 2 kHz
it may not. Thus the sustainable period must allow all four inter-frame gaps,
not just the two explicitly named ARM gaps. Controller computation can overlap
the post-READ guard, but cannot be assumed to provide it.

Tests use 30 MHz SPI unless stated, FIFO priority 50, 0.08 A enabled commands for
timing, no motor or sensors attached, and output through tee to a file. The new
matched clients retain 100-cycle printing and collect interval/work samples in
memory. CPU is Python process time as a percentage of one core, not total system
CPU. Commands in malformed-frame tests are disabled and zero current.

A test-only pointer permits ST-LINK reads of MCU diagnostic counters after runs,
without halting or logging during timing. Clean passes require matching command
publication counts and no unexpected recovery events. Publication is not proof
that every individual command was applied by the current task; latest-value
semantics remain. No command acknowledgement was added to the wire format.

#### 1 kHz timing comparison

All values below are milliseconds, except CPU %. The 99th percentile describes
successful-cycle completion intervals, not a guaranteed deadline. Maxima depend
on exposure and host conditions; trials were sequential, not randomized.

| Setup                                    | Completed cycles / intervals* | Mean interval | p99 interval |  Maximum | CPU % | Mean cycle work | Outcome                                              |
| ---------------------------------------- | ----------------------------: | ------------: | -----------: | -------: | ----: | --------------: | ---------------------------------------------------- |
| Old ARM/COMMAND, three tasks             |                         7,799 |      1.002121 |     1.013723 | 1.076946 |     — |               — | 10/10 trials failed                                  |
| Handshake before thread cleanup          |                        29,880 |      1.002189 |     1.085668 | 1.935003 |     — |               — | 2 × 15 s passed                                      |
| Current handshake, earlier repeated test |                        89,714 |      1.002296 |     1.051113 | 1.350206 |     — |               — | 3 × 30 s passed                                      |
| Current handshake, matched recorder      |                        30,000 |      1.002400 |     1.050797 | 1.259186 |   7.7 |           0.393 | 30,000 cycles passed                                 |
| Four frames, 100 us ARM + 100 us guard   |                        30,000 |      1.002363 |     1.012407 | 1.125130 |  34.9 |           0.426 | 3 × 10,000 normal cycles clean                       |
| Four frames, 150 us ARM + 150 us guard   |                        30,000 |      1.002235 |     1.008518 | 1.098833 |  49.8 |           0.575 | 3 × 10,000 normal cycles clean                       |
| Four frames, 200 us ARM + 200 us guard   |                        30,000 |      1.002290 |     1.013093 | 1.130797 |  64.8 |           0.726 | 3 × 10,000 normal cycles clean                       |
| Four frames, 100/100 us extended run     |                        60,000 |      1.002278 |     1.010778 | 1.098945 |  34.9 |           0.425 | 60,000 cycles; all commands counted; zero recoveries |

*Earlier repeated tests stored completed intervals rather than a separate cycle
count; each run omits the initial interval. The legacy runs lasted only
0.064–2.318 s including startup and all stopped on invalid replies. The equal-size
7,799-interval legacy/current comparison still gave p99 1.014/1.050 ms, so the
ordinary-jitter difference is not solely an effect of exposure length.

#### Four frames with ARM gaps only: failed experiment

| ARM gap |            Trials | Result                                                                                                                |
| ------- | ----------------: | --------------------------------------------------------------------------------------------------------------------- |
| 100 us  |                 3 | All failed before the first complete read/command cycle                                                               |
| 150 us  |                 3 | All failed after 1–183 complete cycles; frequent recoveries                                                           |
| 200 us  |                 3 | All failed after 7–154 complete cycles; frequent recoveries                                                           |
| 300 us  | 3 × 10,000 cycles | Replies passed, but 10,082 / 10,117 / 10,084 MCU failures; one run published only 10,000 of 10,001 attempted commands |

These results are not successful reliability tests even where reply timing looks
good. The added guard is a necessary distinction from the original proposal.

#### 2 kHz comparison

| Setup                   | Cycles | Mean interval ms |   p99 ms | Mean work ms | Work >500 us | CPU % | Outcome                                              |
| ----------------------- | -----: | ---------------: | -------: | -----------: | -----------: | ----: | ---------------------------------------------------- |
| Current handshake       | 10,000 |         0.502310 | 0.550833 |     0.392276 |            0 |  15.5 | Transport passed; no measured cycle work over budget |
| Four frames, 100/100 us | 10,000 |         0.523172 | 0.546908 |     0.521579 |        9,958 |  84.7 | Transport/counts passed; target period NOT sustained |

The guarded implementation needs another guard at the cycle boundary when sleep
no longer covers it. Its mean interval was about 523 us, approximately 1.91 kHz.
Neither test includes MPC computation. Zero work overruns does not mean zero
completion-time jitter or a proven real-time deadline.

#### Fault, expiry and reset tests

Each matrix case begins with an MCU reset and two valid disabled cycles, then one
injection followed immediately by 100 valid cycles using only the normal configured
gaps/guards. No extra recovery sleep is inserted. MCU command counts are checked.
Cases: 1/7/13/15/28 bytes, corrupt CRC, unknown header, ARM without payload, and ARM
followed by idle. The deliberately idle case pauses 35 ms. Each case repeats three
times per setting.

| ARM gap / guard | Recovery cases passed |
| --------------- | --------------------: |
| 100/100 us      |                 16/27 |
| 150/150 us      |                 27/27 |
| 200/200 us      |                 27/27 |

At 100/100 us failures included truncated/oversized frames, bad CRC and unknown
headers. An earlier continuous attempt at that setting also stopped before its
first complete recovery round. Normal-traffic success did not establish recovery.

At 200/200 us, separate register checks verified expiry under silence, READ,
corrupt-CRC traffic and ARM-only traffic; invalid payloads disabled PB9 enable and
TIM1 CCR3 PWM. Reset after ARM and client reopen both recovered.

Continuous 150/150 us test results:

```text
PASS round 1/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 2/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 3/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 4/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 5/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 6/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 7/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 8/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 9/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS round 10/10: 7 malformed frames + 2 abandoned-ARM cases; 100 confirmed commands after each
PASS expiry with silence
PASS expiry with READ
PASS expiry with bad CRC
PASS expiry with ARM
PASS invalid payloads disable enable/PWM
PASS reset after ARM and client reopen
```

These fault cases are comparable to the handshake's seven malformed-frame cases,
but abandoned ARM and held REQUEST are different mechanisms. The handshake's
previous 90-case run required no fixed inter-transfer/recovery gaps.

#### Earlier validation and timing experiments

- Legacy conservative baseline (user-reported checkpoint): 5 MHz, 500 Hz, 500 us
  ARM gap; 10,000 baseline exchanges plus 70 injected faults and 7,000 recovery
  exchanges passed. This was the old two-frame cycle, not four frames per cycle.
- Initial high-speed legacy probe at 1 kHz failed after about 0.41 s; subsequent
  ten-trial results above provide the better sample.
- Handshake before consolidation: 20,000 cycles at requested 30 MHz / 2 kHz,
  mean 0.502 ms, plus 90 fault cases / 9,000 recovery cycles passed.
- SPI recovery-only cleanup passed 2,000 cycles plus 27 fault cases / 2,700 recovery
  cycles before the forwarding task was removed.
- MCU current-loop instrumentation at requested 2 kHz SPI traffic: before/after
  consolidation, maximum measured execution 58/57 us; maximum ISR-to-task delay
  20/20 us; late finishes 7/1; coalesced releases 0/0. Startup was included. These
  counters exclude hardware-edge-to-ISR delay and some instrumentation overhead.
- A further 10,000 enabled-controller cycles with zero-current commands on the
  unpopulated board averaged 0.502 ms without additional late finishes beyond
  the startup count. This was not a physical control test.
- Final normal handshake build: 20,000 cycles at 2 kHz plus 90 fault cases / 9,000
  recovery cycles passed. Expiry, invalid-command output disabling, reset with
  REQUEST high and client reopen were checked separately.
- Six host regression tests passed, including command validation, delayed
  consumption, exact expiry boundaries and microsecond-counter wrap. Optional
  diagnostics and GPA paths compiled; GPA operation was not validated.
- User terminal log: 223 reporting windows, mean window average 1.003214 ms at
  a 1 ms target; two windows peaked above 2 ms (6.074 and 7.698 ms). Those specific
  rare spikes remain unexplained; file-output bench tests do not reproduce the
  full VS Code terminal path.

#### Phase profiling controls (separate instrumented clients)

135,000 profiled cycles passed across six runs. In the GC-disabled event-wait
control, four READY waits averaged 314 us total; both SPI calls averaged 56 us.
Cycle-start p99 was 1.0065 ms while completion p99 was 1.0521 ms. Normal jitter
tracked handshake duration more strongly than sleep overshoot.

| Temporary profiling variant                  | Cycles | Completion p99 ms | Mean work ms | Python CPU % |
| -------------------------------------------- | -----: | ----------------: | -----------: | -----------: |
| Event waits, GC disabled                     | 15,000 |          1.052075 |     0.395299 |         8.52 |
| GPIO polling, GC disabled                    | 15,000 |          1.052908 |     0.332722 |        29.61 |
| Event waits, PCIe ASPM disabled, GC disabled | 15,000 |          1.050352 |     0.384102 |         7.96 |

Two 30,000-cycle printing/quiet profiler runs and a 30,000-cycle GC-monitored run
were also performed. The recorder itself introduced garbage-collection pauses;
confirmed pauses reached 2.276 ms. Initial unmonitored recordings had 23–25 ms
peaks and are excluded from production timing conclusions. The user's earlier
6–8 ms spikes cannot be attributed to GC from these experiments.

Polling reduced latency but not p99 jitter. Disabling PCIe ASPM did not eliminate
the variation; the original powersave policy was restored. GPIO event timestamps
do not isolate electrical-edge-to-kernel delay, so MCU versus Pi interrupt/wake-up
contributions remain unresolved. Do not disable production GC based on the recorder.

#### Limits and artifacts

All outcomes concern this bench, wiring, firmware and workload. Fixed-gap margin
under other load, physical motor control, measurement age and command application
acknowledgement remain unproven. The experimental ARM worker's current-loop timing
was not instrumented; earlier MCU timing figures belong to the handshake versions.
No source changes were applied to the project. The current handshake firmware was
restored, and the REQUEST/READY wires remain necessary for it.

New firmware source, build script, binaries, raw intervals, diagnostic counters
and test scripts: `/tmp/mpc-arm-pairs/`. Earlier repeated-run data:
`/tmp/mpc-repeat-comparison/`. Phase data: `/tmp/mpc-phase-profile/`.
These temporary artifacts may not survive reboot.


<a id="host-results"></a>

## Historical Python / C++ comparison

### Pi C++ / Python timing comparison — 2026-09-25

Historical checkpoint; current transport results are in [NSS timing results](#nss-results).

These are the earlier host-only results. See [MCU timing improvement](#ticker-results)
for the subsequent firmware change and current measurements.

Unchanged two-task REQUEST/READY firmware, Pi 5 and Nucleo F446RE, no motor or
sensors attached. Requested SPI clock 30 MHz, FIFO priority 50, no CPU pinning.
Each row aggregates three runs of 10,000 cycles; client order alternated between
runs. Both clients sent 0.08 A enabled after the first disabled cycle, collected
timestamps into preallocated storage, and printed/wrote CSV only after the loop.
Python uses the existing protocol and transport via `host/benchmark_python.py`.
No garbage-collector settings, kernel settings, or firmware settings were changed.

`dt` is the interval between command-reply completions. `work` covers READ,
decode, placeholder computation, COMMAND, and reply validation, excluding sleep.
CPU is the arithmetic mean of the three runs, as a percentage of one core.

| Target  | Client | Mean dt (µs) | p99 dt (µs) | Max dt (µs) | Mean work (µs) | Work > period |    CPU |
| ------- | ------ | -----------: | ----------: | ----------: | -------------: | ------------: | -----: |
| 1000 Hz | python |     1002.339 |    1051.241 |    1270.260 |        392.531 |       0/30000 |  7.69% |
| 1000 Hz | cpp    |     1001.530 |    1049.945 |    1153.408 |        384.755 |       0/30000 |  4.20% |
| 2000 Hz | python |      502.139 |     550.593 |     657.167 |        391.919 |       1/30000 | 15.21% |
| 2000 Hz | cpp    |      501.424 |     550.000 |     623.611 |        384.485 |       3/30000 |  8.38% |

All 120,000 matched cycles passed reply validation. This does not prove every
command was applied: the current protocol has no applied-command acknowledgement.

C++ reduced measured CPU use by about 45%, but mean communication work decreased
by only 7–8 µs (about 2%). The approximately 50 µs p99 completion-interval excess
remains. At 1 kHz, p99 cycle-start intervals were 1007.167 µs for Python and
1003.778 µs for C++; stable starts alone do not make completions equally stable.
At 2 kHz both clients had occasional work above the 500 µs budget. C++ is a useful
basis for a native controller, but this measurement does not justify claiming
that switching languages solves transport jitter.

An earlier, separate 1,000-cycle **disabled-command** C++ smoke run had a
5.243 ms maximum completion interval and 4.682 ms maximum work. Its cause was not
established. It must not be hidden by the cleaner matched runs or treated as a
worst-case bound. No physical control deadlines have been validated.

Next timing investigation: measure MCU REQUEST-to-READY and transaction-retirement
latencies alongside the 50 µs current-task pulse. The repeated roughly 50 µs
variation suggests interaction with task scheduling, but does not prove its cause.
Use a logic analyzer or temporary phase instrumentation before changing priorities,
DMA handling, the handshake, or kernel settings. Keep the current firmware as the
comparison baseline. Add a representative controller workload before selecting a
production loop rate.

Reproduce one matched pair (run sequentially):

```sh
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 1000 --enable --csv /tmp/cpp.csv
sudo chrt -f 50 python3 host/benchmark_python.py --count 10000 --period-us 1000 --enable --csv /tmp/python.csv
```

Repeat three times, alternating order; use `--period-us 500` for 2 kHz.
Raw CSV files, individual logs, and aggregate statistics from this session are
in `/tmp/mpc-cpp-comparison/` (temporary, not a durable checkpoint).
The earlier double-ARM and Python phase experiments are in
`/tmp/mpc-arm-pairs/COMPARISON.md`. The guarded double-ARM experiment improved
ordinary completion jitter at the cost of substantial busy-wait CPU use and
fixed timing assumptions; it did not sustain 2 kHz with the necessary guards.

The firmware image is unchanged:
`f4aede65219d7432ea626e66fa65b24163ddf0d2e9059775ee9ab9cd877f6cee`.

Validation beyond the matched timing runs:

- Native golden-frame tests match Python bytes; every single-bit corruption of a
  valid reply and non-finite measurement in any field is rejected. Six existing
  Python/current-command tests passed; nine invalid CLI combinations were rejected.
- C++ at 2 kHz: 10,000 disabled baseline cycles (mean interval 501.452 µs), then
  90 malformed/cancelled/timed-out requests and 9,000 valid recovery cycles passed.
  This baseline had one work duration above 500 µs; framing success is not a
  deadline guarantee.
- ST-LINK register checks verified enable/PWM off after normal exit, SIGINT, and
  SIGTERM, with outputs enabled during each run. Each close/reopen passed another
  100 disabled cycles. A deliberately occupied READY line caused constructor
  failure and correctly released the previously claimed REQUEST line.
- A deliberate debugger-stall test caused absent READY. The client exited with
  an error after 202 ms (initial wait plus final-disable attempt), reported final
  disable unconfirmed, and left REQUEST low. The MCU did **not** resume normal
  communication after the debugger resume attempt; a reset was required. The
  cause of that stall-recovery failure was not established. This is an unresolved
  MCU/debugger-stall case, not a passed recovery test. After reset, 1,000 disabled
  cycles passed (mean interval 1001.614 µs). No firmware was changed.


<a id="ticker-results"></a>

## Historical MCU ticker optimization

### MCU timing improvement — 2026-09-25

Historical checkpoint; current transport results are in [NSS timing results](#nss-results).

The SPI worker now uses the F446's 32-bit, 1 MHz `us_ticker_read()` instead
of repeated `Timer::elapsed_time()` calls. Unsigned elapsed-time comparisons
preserve the 20 ms handshake timeout across ticker wrap. The 50 µs current
Ticker already prevents deep sleep, so removing this SPI Timer does not remove
that protection during operation.

The two tasks, their priorities, DMA setup, CRC-8, frame format, and Pi clients
are retained. No SPI processing was moved into an interrupt. REQUEST interrupts
also count high observations, allowing recovery to recognize a fresh request
when STM32's single pending bit for both edges coalesces a brief low pulse.
Boot synchronization still requires REQUEST low or a recorded release.

#### Measurement

Same Pi 5, Nucleo F446RE, 30 MHz SPI, native C++ client, `SCHED_FIFO` priority 50,
0.08 A test command, no attached actuator/sensors, and no MPC computation.
Work spans the READ handshake through completion of the COMMAND handshake;
it is not sensor age or command-to-actuator latency. Completion interval is
measured separately from work.

Temporary DWT instrumentation found substantial MCU preparation and completion
cost. On the original firmware, REQUEST ISR entry to READY averaged about
70 µs, and release ISR entry through completion about 63 µs. These elapsed
measurements include current-task preemption and instrumentation overhead;
they exclude electrical-edge-to-ISR latency. They are not pure SPI CPU costs.
An attempted subtraction of current-task time had non-atomic sampling errors
and was discarded. The acceptance comparison uses uninstrumented firmware.

Three 10,000-cycle runs per firmware/rate, 120,000 cycles total, all valid.
One additional pair overlapped compilation; its raw results were retained but
excluded and the entire pair repeated without compilation.

| Firmware / rate  | Mean work | p99 work | Max work | p99 completion interval | Max completion interval |
| ---------------- | --------: | -------: | -------: | ----------------------: | ----------------------: |
| Original / 1 kHz |  384.9 µs | 409.2 µs | 630.4 µs |               1049.9 µs |               1133.0 µs |
| Original / 2 kHz |  384.7 µs | 409.2 µs | 521.9 µs |                550.0 µs |                610.8 µs |
| Updated / 1 kHz  |  237.7 µs | 292.7 µs | 476.5 µs |               1050.5 µs |               1231.7 µs |
| Updated / 2 kHz  |  238.1 µs | 297.2 µs | 377.6 µs |                550.8 µs |                683.5 µs |

Mean work fell about 38%; mean READ completion fell from about 183 µs to
108 µs. No updated-firmware work duration exceeded its requested period in
these matched runs; the original exceeded 500 µs five times at 2 kHz.
Completion-interval maxima were higher on the updated firmware in these runs:
this is a latency improvement, not demonstrated jitter improvement. These are
finite observations, not worst-case bounds.


#### Validation and limits

- Corrected isolated candidate: C++ 10,000 disabled baseline cycles at 2 kHz,
  then 90 injected faults and 9,000 recovery cycles passed. Python: 1,000 baseline
  cycles, 90 faults and 9,000 recovery cycles passed.
- Current-loop diagnostics: 10,000 enabled cycles at 2 kHz added 100,449 current
  iterations with no additional skipped releases or late finishes. The single
  startup late finish and 57 µs historical execution maximum did not increase.
  Diagnostic instrumentation was excluded from the timing comparison.
- Normal project build, flashed and verified: another 10,000 enabled cycles each
  at 1/2 kHz averaged 238.0/237.5 µs work, with maxima 320.9/320.1 µs. Another
  1,000 disabled cycles plus 90 faults / 9,000 recovery cycles passed.
- Twenty held-REQUEST timeouts stayed unarmed for another 40 ms while REQUEST
  remained high, then passed 100 valid cycles each. Reset with REQUEST high,
  fresh requests after release, and client close/reopen passed.
- ST-LINK register reads without halting verified command expiry during silence,
  READ-only and bad-CRC traffic; fresh commands maintained enable; disable and
  invalid current/enable/reserved payloads cleared enable and PWM.
- MCU build, native protocol test, six Python/current-command regression tests,
  and whitespace checks passed. No CMake, priority, or host-code changes.


The first ticker-only candidate failed recovery immediately after a held-REQUEST
timeout. The additional request counter and synchronization guard correct that
case; the failing candidate was not retained. A continuously high REQUEST must
remain unarmed after timeout, and REQUEST high across reset must not start DMA.

Ordinary completion jitter remains roughly one 50 µs current-loop period.
Lower average work does not establish a hard deadline on this Linux host.
The previously documented debugger-stall recovery failure remains unresolved.
A representative controller workload and longer tests under defined Pi load
are still needed before selecting a production rate. Further interrupt/DMA
changes should first demonstrate a benefit without delaying the current loop.

#### Reproduction and checkpoint

Build/flash as in the README, then run sequentially:

```sh
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 1000 --enable
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 500 --enable
sudo chrt -f 50 build/host/mpc_spi --count 10000 --period-us 500 --faults
sudo chrt -f 50 python3 python/check_spi.py --count 1000 --period-us 500 --faults
```

Repeat timing runs three times per firmware/rate, alternating firmware order.
Use `--csv /tmp/name.csv` for raw C++ samples. Avoid compiling during timing runs.
Temporary isolated builds, rollback image, phase measurements, raw CSVs, and
validation scripts/logs are in `/tmp/mpc-mcu-latency/`; these are not durable
artifacts. The aggregate findings are preserved here.

Original firmware SHA-256:
`f4aede65219d7432ea626e66fa65b24163ddf0d2e9059775ee9ab9cd877f6cee`.
The matched candidate was built in isolation. The production build differs only
in comments and a private timeout member name; final on-board checks use that
normal project build, SHA-256:
`cdf8d67b2a0073c09adc5c57e6eea8989f9c03090e3442b7c12bdc1e837d3619`.


The verified production firmware remains flashed; all test clients are closed.


<a id="research"></a>

## Historical external research

### SPI control communication research — 2026-09-25

#### Conclusion

The measured 385 µs is a property of our current software path, not an SPI or
STM32F446 lower bound. The useful alternatives are to prepare transactions ahead
of demand, reduce MCU task scheduling on the transaction path, and reduce Pi
GPIO/SPI driver overhead. Keep the tested firmware as the comparison baseline.
Do not equate a faster SPI clock, a fast MCU current loop, or buffered telemetry
throughput with fresh-sample-to-command latency through a Linux controller.

This is a targeted review of public projects, their implementation files, and
vendor/kernel documentation, not an exhaustive survey of every available project.
No firmware, host implementation, wiring, driver binding, or kernel configuration
was changed during this research. External source files were read, not executed.

#### Closest comparable systems

##### Remora / LinuxCNC: Pi 5 plus STM32F4

The [supported boards](https://remora-docs.readthedocs.io/en/latest/hardware/SPI/rpspi.html)
include Pi 5 and STM32F446. The reviewed
[STM32F4 communication source](https://github.com/scottalford75/Remora/blob/09d7e2416da40a46681cb9ddb21cca04242d0fe4/Firmware/FirmwareSource/Remora-OS6/TARGET_STM32F4/drivers/comms/RemoraComms.cpp)
configures circular RX/TX DMA and starts it in advance. NSS completion marks
received data for processing. There is no equivalent per-frame REQUEST/READY
exchange. That file's incomplete-transfer reset path is a placeholder and its
packet processing checks headers; this is not evidence of our CRC/fault coverage.

The [host component](https://github.com/scottalford75/Remora/blob/09d7e2416da40a46681cb9ddb21cca04242d0fe4/LinuxCNC/Components/Remora-spi/remora-spi.c)
provides separate read and write functions and selects an RP1 implementation for
Pi 5. Its [SPI engine](https://github.com/scottalford75/Remora/blob/09d7e2416da40a46681cb9ddb21cca04242d0fe4/LinuxCNC/Components/Remora-spi/spi-dw.c)
services FIFOs by polling, rather than calling spidev for each exchange.

Relevance: a closely matched architecture without our repeated preparation
handshakes. No trustworthy, directly comparable Pi 5 + F446 READ–compute–COMMAND
latency distribution was found. Circular DMA is worth understanding, not copying
without resolving immutable snapshots, transaction boundaries, and recovery.

##### LinuxCNC / Mesa HostMot2: Pi 5 plus FPGA

[hm2_spix documentation](https://linuxcnc.org/docs/devel/html/man/man9/hm2_spix.9.html)
describes dedicated Pi drivers, a spidev fallback, and bulk queuing to reduce
per-operation overhead. Its documented Pi 5 servo-thread example is 1 kHz, not a
claim of 20 kHz host feedback control. Direct drivers need exclusive ownership;
the documentation warns against concurrent kernel SPI access.

The [Pi 5 source](https://github.com/LinuxCNC/linuxcnc/blob/41568b415c0aa97b29470092cbce4ac73ff2b0a2/src/hal/drivers/mesa-hostmot2/spix_rpi5.c)
maps RP1 registers and fills/drains the SPI FIFOs directly. The endpoint is an
FPGA, so its register response does not wait for an MCU RTOS worker.

Relevance: separate the Linux driver costs from the slave's response costs.
An FPGA is not required to investigate those costs on our existing hardware.

##### mjbots pi3hat: Pi 3/4 plus STM32 SPI-to-CAN bridges

The [protocol reference](https://github.com/mjbots/pi3hat/blob/12ab9afffaabdacba5a99b106552db962a08a502/docs/reference.md)
defines 10 MHz SPI and minimum 3 µs setup/phase/turnaround holds. Its native host
library bypasses Linux peripheral drivers and sometimes busy-waits. That product
explicitly does not support Pi 5; its host code cannot simply be reused here.

The [slave implementation](https://github.com/mjbots/pi3hat/blob/12ab9afffaabdacba5a99b106552db962a08a502/fw/register_spi_slave.h)
handles NSS/address events in interrupts and programs DMA registers directly. Its
buffer-selection callback has a stated under-1-µs execution requirement.
This is evidence that short slave preparation paths are possible, not a timing
bound for F446 under our current-task load.

The author's [2024 command-rate benchmark](https://blog.mjbots.com/2024/05/16/optimizing-moteus-command-rate/)
reports roughly 2.2 kHz for a single moteus with default messages through pi3hat.
That includes CAN-FD and drive response and excludes application computation.
It must not be presented as either pure SPI latency or a 20 kHz host loop.

Relevance: very small protocol gaps can work when the firmware and host paths
are designed for them. Our tested 150 µs gaps are not fundamental SPI requirements.

##### Open Dynamic Robot Initiative / Solo µDriver

Its [SPI interface specification](https://github.com/open-dynamic-robot-initiative/master-board/blob/574fe988270aa3da948484ad3fa1f15dbf8162bf/documentation/BLMC_%C2%B5Driver_SPI_interface.md)
reports a simultaneous 272-bit command/status exchange taking less than 35 µs at
8 MHz. It also requires at least 0.7 ms between transactions and describes sensor
updates around 1.5 kHz, with data potentially about 0.66 ms old.

Relevance: a short exchange can coexist with a much slower effective feedback
path. Simultaneous command/status transfer also cannot compute the outgoing
command from the status that arrives during that same transfer.

##### MIT Mini Cheetah

The reviewed [SPI host source](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/robot/src/rt/rt_spi.cpp)
uses spidev full-duplex 132-byte exchanges and requests 6 MHz. The
[hardware bridge](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/robot/src/HardwareBridge.cpp)
schedules the SPI task with a 0.002-second period in this version.

Relevance: a well-known robot is not automatically evidence for a faster,
otherwise identical host communication loop. Code versions and scheduling matter;
secondary descriptions claiming different rates should not override the source.

##### Pi 5 / STM32F411 servo-drive experiment

[jnzim/stm32-servo-drive](https://github.com/jnzim/stm32-servo-drive) is unusually
close hardware and describes circular-DMA telemetry and MCU-local control loops.
Its README contains both a 20 kHz telemetry claim and later measurement-path
corrections, including torn frames from modifying the active DMA buffer. Other
sections retain the older description. Pi-streamed control is described as future
work. It is useful evidence of the buffer-coherency trap, not a validated 20 kHz
Pi READ–compute–COMMAND benchmark.

#### Vendor and kernel findings

- [Espressif's slave guidance](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_slave.html)
  recommends preparing/queuing slave transactions and using a GPIO readiness
  signal where needed. Preparation ahead of demand and flow control are compatible.
  A DMA buffer or transaction descriptor is not necessarily a FIFO of stale motor
  commands; latest-command semantics can be preserved in our application.
- [Espressif's master measurements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/spi_master.html#transaction-duration)
  give typical one-byte transaction durations of 28 µs with interrupt/DMA versus
  8 µs with polling/non-DMA on ESP32. These are not Pi estimates; they demonstrate
  why DMA can reduce CPU work without minimizing short-transaction latency.
- [Linux spidev](https://docs.kernel.org/spi/spidev.html) is synchronous and permits
  composite transfers. Batching protocol segments can reduce syscall overhead,
  but cannot run our userspace controller between READ and COMMAND inside one
  already-submitted ioctl. Two host submissions remain appropriate for that order.
- The [RP1 manual, section 3.3.1](https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008370-DS-1-rp1-peripherals.pdf?disposition=inline)
  documents PCIe access latency and extra round trips for reads. Pi 3/4 direct-GPIO
  timing results therefore cannot be assumed on Pi 5. PIO also is not a free
  shortcut: [Raspberry Pi's PIOLib explanation](https://www.raspberrypi.com/news/piolib-a-userspace-library-for-pio-control/)
  describes additional host-to-firmware latency for its operations.
- [STM32F446 errata ES0298, section 2.14.6](https://www.st.com/resource/en/errata_sheet/es0298-stm32f446xcxe-device-errata-stmicroelectronics.pdf)
  warns that BSY can remain asserted at the end of a slave transaction. Keep
  NSS/receive-completion-aware, bounded recovery; do not replace our handling with
  an unbounded BSY wait. Generic STM32 examples may target different SPI hardware.

#### What is true on this Pi

Read-only inspection on this date found:

```text
Kernel: 6.12.109+rpt-rpi-2712
CONFIG_PREEMPT=y
### CONFIG_PREEMPT_RT is not set
### CONFIG_PREEMPT_DYNAMIC is not set
CONFIG_HZ=250
SPI device driver: spidev
SPI controller driver: dw_spi_mmio
```

FIFO priority 50 is not equivalent to PREEMPT_RT. The latter changes kernel
preemptibility and interrupt handling; see the
[kernel explanation](https://docs.kernel.org/core-api/real-time/theory.html).
Neither CONFIG_HZ=250 nor the absence of PREEMPT_RT implies a 4 ms minimum period.
Our measured sub-millisecond cycles already disprove such an interpretation.
PREEMPT_RT is a separate tail-latency experiment, not a promised cure for mean delay.

The Raspberry Pi 6.12 branch's
[DesignWare DMA eligibility code](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/spi/spi-dw-dma.c)
rejects transfers no larger than the FIFO. The
[core transfer path](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/spi/spi-dw-core.c)
selects DMA, polling when no IRQ is connected, or interrupt handling otherwise.
Do not assume that using DMA on the F446 means our 14-byte Pi transfers also use
DMA. This was source inspection of the matching branch, not runtime tracing of
our exact kernel build.

#### Recommended experiments, one change at a time

1. **Measure the baseline's electrical phases and MCU execution.** Record REQUEST,
   READY, NSS/SCK, and the current-task pulse; add bounded MCU cycle timestamps if
   needed. Separate wire completion, host return, command acceptance, and current-
   task application. Current 385 µs is host cycle work, not measured sensor-to-PWM
   latency. Include sensor age and controller computation in the eventual budget.
2. **Shorten the MCU normal transaction path first.** Prototype bounded interrupt
   handling for the few hardware preparation/completion steps that currently wait
   for the lower-priority SPI worker. Keep expensive recovery/logging outside the
   interrupt. Measure added current-task interference; do not blindly raise the
   entire SPI worker above the current loop. Retain the current handshake initially
   so its contribution can be measured independently.
3. **Evaluate preparing DMA/snapshots ahead of demand.** This is the stronger
   architectural opportunity highlighted by Remora. Define buffer ownership,
   snapshot freshness, peripheral prefetch, and short/extra-frame recovery first.
   Double buffering protects consistency only with correct ownership; it does not
   automatically make an already-armed reply fresh. Use a sample counter/timestamp
   when evaluating this. Keep readiness signaling until a replacement boundary
   protocol is proven; simply deleting READY waits introduces races.
4. **Benchmark a Pi 5-specific host path if remaining overhead warrants it.**
   Compare direct RP1 SPI/GPIO access or a suitable kernel driver against spidev/
   libgpiod. Use exclusive peripheral ownership, bounded polling and cleanup. Do
   not combine a direct driver with the still-bound kernel controller. This is
   more maintenance than the current client; retain it only for a measured gain.
5. **Evaluate PREEMPT_RT, affinity/IRQ placement and locked/prefaulted memory for
   tail latency**, after identifying the critical path. Test under realistic load.
   Do not introduce all tuning at once or assume a higher FIFO priority is always
   better than the interrupt threads needed to complete I/O.

Keep the user's read → compute → command ordering. A single full-duplex frame
cannot send a command derived from data not yet received. A future pipelined
controller could change this tradeoff, but that would be a control-architecture
change, not a free transport optimization.

Do not adopt the previous double-ARM variant as the endpoint: its 150 µs margins
were measured for a particular implementation. The earlier roughly 550 µs C++
estimate applied only to retaining those gaps, was never benchmarked, and says
nothing about an optimized prearmed slave or interrupt-driven protocol.

#### Acceptance criteria

Compare mean, p99/p99.9 and maximum latency, CPU use, sample age, command application
latency, and current-task deadline interference. Run the existing malformed-frame,
CRC, cancellation, timeout, reset/reopen and expiry checks. Use command counters or
acknowledgement instrumentation: valid reply CRC alone does not prove application.
The debugger-stall recovery limitation remains unresolved.

No external result reviewed establishes a guaranteed sub-100-µs fresh-sensor-to-
new-command loop for our exact Pi 5/F446/Mbed workload. The sources justify pursuing
lower overhead; they do not justify promising a particular rate before measurement.

#### Follow-up: fresh snapshots without REQUEST/READY — 2026-09-25

This review is read-only with respect to firmware, wiring, kernel configuration,
GPIO ownership and SPI traffic. The tested REQUEST/READY firmware remains installed.

##### Verified mechanisms and corrections

- ST [AN4031 §1.1.11](https://www.st.com.cn/resource/en/application_note/an4031-using-the-stm32f2-stm32f4-and-stm32f7-series-dma-controller-stmicroelectronics.pdf)
  says double-buffer DMA switches memory targets when NDTR reaches zero. It
  permits changing the *inactive* target address. This is a DMA block boundary,
  not an NSS latch or a 50 µs measurement timer. A software critical section
  cannot prevent DMA consuming a buffer that the CPU is modifying.
- Live device tree: `spi0.0` is under `rp1/spi@50000`, compatible
  `snps,dw-apb-ssi`; its controller's `cs-gpios` property selects GPIO8 and GPIO7,
  active low. Device zero therefore uses GPIO8-controlled NSS. No
  `spi-cs-setup-delay-ns` property is presently set on this device.
- The matching Raspberry Pi 6.12 branch's
  [SPI core](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/spi/spi.c)
  parses `spi-cs-setup-delay-ns`, `spi-cs-hold-delay-ns` and
  `spi-cs-inactive-delay-ns`. For GPIO CS it writes the GPIO and invokes
  `spi_delay_exec`. This supports a kernel-managed setup interval, not a
  hardware-generated fixed interval. The inspected branch invokes the setup
  delay in both the GPIO helper and the outer CS helper; do not assume a
  requested setting equals measured pin timing. These are source findings,
  not measurements of a configured delay on this board.
- The [DesignWare driver](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/spi/spi-dw-core.c)
  supplies `set_cs` and `transfer_one`, with no `set_cs_timing` callback in
  the reviewed source. Selecting an SPI transfer still involves kernel/driver
  work after NSS assertion. Scheduling/interrupts can lengthen the interval.
- Ordinary spidev `delay_usecs` is a delay *after* the transfer, as defined in
  its [UAPI header](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/include/uapi/linux/spi/spidev.h).
  It is not a direct CS-assertion-to-first-clock setting. A command segment plus
  a delay plus a payload segment can instead be submitted in one SPI message,
  preserving CS, but still introduces two clock segments and driver overhead.
- ST [ES0298 §2.14.6](https://www.st.com/resource/en/errata_sheet/es0298-stm32f446xcxe-device-errata-stmicroelectronics.pdf)
  rules out relying solely on BSY going low to detect slave completion. NSS,
  RX completion/count, and peripheral errors must participate in validation.

The earlier suggestion of a "controller-generated CS setup delay" was too
strong: this Pi's verified path is GPIO CS plus software delay in the kernel.
The earlier double-buffer sketches also omitted buffer lifetime: two publishing
buffers cannot alternate freely while DMA retains ownership of one of them.

##### What comparable projects actually establish

The pinned pi3hat reference and source already listed above specify minimum
3 µs setup, address-to-payload, hold and turnaround intervals. The slave uses
NSS/address interrupts, direct DMA register programming and a start callback
required to finish within 1 µs. Its reference says "8 byte address", while
its implementation documents and consumes an 8-bit address; do not copy that
reference typo. This validates the *pattern* of an explicitly timed protocol;
it does not validate those timing constants for our F446/Mbed/Pi 5 system.
Its MCU DMA implementation differs from F446, the host bypasses kernel drivers,
and the inspected error path contains a TODO.

The pinned Remora F4 source prearms circular DMA and checks headers. Its
incomplete-transfer reset branch prints a placeholder rather than performing
recovery. It demonstrates eliminating per-frame ARM requests, not a proof of
fresh, coherent CRC-protected snapshots under arbitrary transaction timing.

##### Concrete candidate and reliability contract

For two useful 14-byte exchanges per controller cycle, a reasonable candidate is
NSS-framed **normal-mode DMA** with a kernel-managed setup interval:

1. The current thread publishes a complete latest measurement snapshot every
   control iteration. Its publication must be coherent with the NSS handler.
2. NSS falling starts a bounded MCU preparation handler: copy that completed
   snapshot into a dedicated TX frame, finish its CRC if necessary, and arm
   RX/TX DMA from a known peripheral state. Keep the TX frame immutable until
   physical transfer completion. Buffer capacity is trivial here; a small copy
   is easier to review than a circular stream with changing ownership.
3. The Pi clocks only after the agreed minimum NSS setup interval. The bound
   must cover interrupt masking, higher-priority interrupts, preparation and
   DMA startup. It cannot be chosen just from an average interrupt latency.
4. On NSS rising, validate length/RX completion, SPI/DMA errors and CRC. Only a
   valid COMMAND publishes a new setpoint; READ does not refresh command expiry.
   TX DMA completion alone is not wire completion. Handle preload/drain/reset
   explicitly rather than just changing a DMA pointer.
5. Specify a minimum NSS-high interval for completion/normal rearming. Keep
   expensive fault recovery outside the short handler; reject transactions
   during recovery. Startup, partial/extra frames, delayed clocks, reset and
   immediate retries must have defined outcomes. No extra READY wire means the
   master learns about failures from protocol validation/timeouts, not from
   an acknowledgment that preparation succeeded.

This yields READ exchange, Pi computation, COMMAND exchange with no dedicated
control-period pipeline delay. It removes ARM messages and userspace REQUEST/
READY waits. It does **not** remove timing requirements, Linux jitter, or the
possibility of stale data if the Pi stalls after snapshot selection. Data age
at reception includes sample/publication age, setup interval and transfer time;
50 µs is not an established total bound. Timestamp/sequence instrumentation is
needed to measure it, and actual command acceptance must be checked separately
from reply CRC.

Before adopting it, verify NSS/SCK and MCU preparation timing together, under
load, then repeat current-loop interference and the existing fault matrix.
No numerical setup interval or jitter improvement is established by this review.
If a bounded setup interval cannot be supported without disturbing the current
loop, retain readiness acknowledgment or accept an older prearmed snapshot.

##### Implementation follow-up — 2026-09-25

The candidate is implemented using a zero-byte delayed segment followed by the
payload in one spidev ioctl. This provides kernel-managed NSS setup on the current
Pi without a device-tree change. Defaults are 30 µs setup and 30 µs minimum high
interval. Hardware tests and MCU instrumentation passed; simultaneous electrical
NSS/SCK measurement is still outstanding, so no worst-case margin is established.
See [NSS timing results](#nss-results) for the matched comparison,
fault recovery, current-loop checks and remaining limits.


<a id="checkpoints"></a>

## Historical architecture checkpoints

### SPI reliability and control architecture

#### Resume checkpoint — 2026-09-25, NSS transport

- Two full-duplex exchanges: READ → compute → COMMAND. NSS falling selects an
  immutable telemetry frame; NSS rising validates and publishes commands.
- C++ and Python use a zero-byte delay segment plus payload in one ioctl;
  default setup and minimum NSS-high intervals are both 30 µs. No REQUEST/READY
  GPIO access, ARM transfer, device-tree or kernel change.
- Normal handling and bounded peripheral reset run in the NSS IRQ. A High1 worker
  handles timeout/fallback recovery; current task remains 50 µs / High2.
- Current measurements and acceptance checks: [host/NSS_TIMING_RESULTS.md](#nss-results).
- Final normal firmware is flashed; clients are closed. Matched comparison,
  both clients' fault matrices, expiry/output checks, held-NSS reset, signal
  shutdown, current-loop diagnostics and bounded Pi CPU-load tests passed.
- Previous firmware/client/source checkpoint: `build/checkpoints/request-ready-cdf8d67b/`.
  Top-level MCU CMake is unchanged. Standalone host CMake no longer requires libgpiod.
- Next: test representative controller computation and physical sensor/actuator
  behavior. No hard real-time or 50 µs end-to-end freshness bound is established.


#### Previous checkpoint — MCU ticker optimization

- SPI timing uses the 32-bit microsecond ticker with unsigned elapsed comparisons.
  A fresh high REQUEST interrupt can also end recovery after initial synchronization,
  covering a short release pulse missed by STM32's shared edge-pending bit.
- Tasks, priorities, protocol, wiring, Pi clients, and CMake are retained.
  No DMA/CRC work was moved into an interrupt.
- Measured mean C++ read→command work is about 238 µs, previously about 385 µs.
  Ordinary completion jitter remains roughly 50 µs; this is not a hard deadline.
- Built, flashed, and verified; no test client remains running. Fault recovery,
  expiry, held-REQUEST/reset checks, and current-loop diagnostics passed.
  Evidence: [host/MCU_TIMING_RESULTS.md](#ticker-results).
- Next: measure with a representative controller workload and defined Pi load;
  preserve current-loop timing when considering further handshake optimizations.
  The earlier debugger-stall recovery issue remains open.

#### Previous checkpoint — C++ host client validated

- Added standalone C++ Pi client in `host/`; MCU sources, priorities, wiring,
  firmware image, and top-level CMake remain unchanged by this addition.
  Native build: `cmake -S host -B build/host -DCMAKE_BUILD_TYPE=Release`, then
  `cmake --build build/host -j2`. Installed `libgpiod-dev` 1.6.3 on this Pi.
- C++ defaults to 10,000 disabled cycles at 1 kHz / 30 MHz. Explicit `--enable`
  sends the test setpoint after the first disabled cycle. Samples are preallocated;
  no periodic printing or busy-waiting in the timed loop. See README for commands.
- Matched Python/C++ comparisons: three 10,000-cycle runs each at 1 kHz and 2 kHz,
  all 120,000 cycles passed. CPU fell about 45%, mean work only about 2%, and
  completion jitter remained similar. An earlier C++ smoke run had a 5.243 ms
  completion outlier; language choice does not eliminate rare stalls.
  Evidence and reproduction: [host/TIMING_RESULTS.md](#host-results).
- C++ 2 kHz disabled baseline: 10,000 cycles plus 90 faults / 9,000 recovery cycles
  passed. Normal exit, SIGINT, and SIGTERM cleared enable/PWM; reopening passed.
  Partial GPIO-constructor failure released REQUEST. Native protocol tests and
  the six existing host regression tests passed.
- Missing-READY test: C++ timed out and released REQUEST. A deliberately debugger-
  stalled MCU did not recover on resume; reset was required. Stall recovery remains
  unresolved and is recorded in the timing report. After reset, 1,000 disabled
  cycles passed. Firmware was not modified; no client or test remains running.
- Next: measure MCU handshake phases alongside the current-task pulse, then test
  a representative controller workload. Do not change priorities or adopt fixed
  ARM gaps on the assumption that Python caused the remaining jitter.

#### Retained firmware checkpoint — two-task cleanup validated

- Current firmware is built, flashed, and running. No test remains running.
- Two application tasks: the 50 µs current task (High2) and event-driven SPI worker
  (High1). The 200 µs forwarding task, its ticker/debug output, and current-loop
  mutex are removed. Do not reintroduce generic RealTimeThread inheritance.
- The current task owns sensors, controller/filter state, PWM, direction, motor
  enable, and the 300 ms command timeout. Commands expire from the SPI worker's
  MCU reception timestamp, not consumption time. Expiry disables enable/PWM and
  resets controller/filter state. Protection still depends on this task running.
- Single latest-command storage; no FIFO. Commands and diagnostics are separate.
  Measurements publish directly from the current task. Short critical sections
  protect coherent copies; the SPI worker alone owns DMA buffers and recovery.
- Redundant DMA aborts, NSS busy-wait, reset threshold, unused reply overloads,
  and obsolete forwarding interfaces are removed. CRC-8 and wire format unchanged.
- REQUEST wiring: Pi pin 16 / GPIO23 → PC4 / CN10-34.
  READY wiring: PA8 / CN10-23 → Pi pin 18 / GPIO24.
  Pi GPIO discovery uses `pinctrl-rp1`; installed libgpiod Python bindings are 1.6.3.
- Current Python settings are requested 30 MHz SPI, 1 kHz, 0.08 A, print every 100 cycles.
  `PRINT_EVERY = 0` now disables periodic output and interval collection.
- Final build passed 20,000 disabled cycles (mean 0.502 ms), 90 injected faults,
  and 9,000 post-fault cycles without fixed recovery sleeps:
  `sudo chrt -f 50 python3 python/check_spi.py --count 20000 --speed 30000000 --period-us 500 --faults`.
- Bench checks with no motor attached verified actual enable/PWM registers:
  silence, reads, and bad CRCs do not keep an enabled command alive; fresh commands
  maintain enable; disabled or invalid payloads clear enable/PWM.
- Reset with REQUEST high, subsequent release/recovery, and client reopen passed.
  These checks do not establish recovery across arbitrary reset timing.
- Six host tests passed, including command expiry boundaries and ticker wrap.
  Optional diagnostics and GPA paths compile; GPA operation was not validated.
- Preserve staged and unstaged work. No commit was made. CMakeLists.txt unchanged;
  rerun CMake configuration after removing the forwarding source files because
  the existing source glob is evaluated during configuration.

#### Timing evidence and limits

Optional `MPC_CONTROL_DIAGNOSTICS` records current-loop timing and release counts.
Both it and `MPC_SPI_DIAGNOSTICS` are disabled in the final flashed build.
In instrumented 20,000-cycle disabled runs, old/new maximum execution was 58/57 µs,
maximum ISR-to-task start was 20/20 µs, and late finishes were 7/1. No coalesced
releases were observed. The new late count was present at the first report and
stayed constant. A further 10,000 enabled-controller cycles, with zero-current
commands and no motor attached, did not add late finishes beyond startup.

These counters add overhead, exclude timer-edge-to-ISR latency and final counter
bookkeeping, and cannot count hardware releases whose interrupts never ran.
The average Pi cycle time is not a deadline guarantee. README describes the
instrumentation and tested behavior. PB5 remains the current-task execution pulse.

#### Transport rules retained

Each 14-byte SPI frame has a header, three little-endian float32 values, and CRC-8.
Pi executes `read measurements → validate → compute → send command`.
READ is 0x57, COMMAND 0x55, REPLY 0x45. There is no ARM-only frame or fixed ARM gap.
Two useful SPI transactions are necessary for a command computed from the sample
just received; a single transaction would require using an earlier sample.

For each transaction: Pi raises REQUEST, MCU prepares an immutable snapshot and
arms DMA before raising READY, Pi transfers under NSS and releases REQUEST,
then MCU processes/rejects the frame and lowers READY. READY low is retirement,
not command acknowledgement. The MCU timeout is 20 ms; Pi waits are 100 ms.
A falling-edge counter preserves REQUEST release across worker scheduling delays.
Startup and timeout recovery require release before a new request is accepted.

READ frames and rejected framing/CRC do not refresh command expiry. CRC-valid
invalid command values disable output when consumed. Repeated valid commands do
refresh expiry: no sequence or session metadata exists. Later commands may
replace an unconsumed command, including an invalid one; latest-command semantics
are intentional. The command-transfer reply is prepared before command receipt.

#### Remaining work, in priority order

1. **Validate timing under the intended workload.** Longer runs, enabled control
   with real sensors, future controller computation, and external measurement of
   release/output timing. Investigate startup latency before claiming 50 µs
   deadlines. Do not infer physical control quality from communication tests.
2. **Finish fault coverage.** Held-low NSS, interrupted clocks without NSS release,
   and resets at different transaction phases. Application expiry cannot protect
   against a stalled current task/CPU; assess that separately before motor trials.
3. **Add metadata only when needed.** Measurement age/sample ID and applied-command
   acknowledgement are still absent. Define duplicate and restart semantics
   before adding sequence/session fields; avoid expanding the protocol piecemeal.
4. **Add MPC after transport/control timing is understood.** Compute from a valid
   measurement, bound execution time and command limits, and reject obsolete
   results. The existing 300 ms expiry is not an acceptable latency budget by default.


<a id="student-cleanup"></a>

## Student cleanup and final review — 2026-09-25

The existing NSS behavior is the reference for this cleanup, not the old ARM
behavior at `origin/main` (`72ce22c`). Preserving today's behavior necessarily
retains the intervening transport, expiry and two-task changes. No additional
controller or transport redesign was made.

Removed unused REQUEST/READY clients and their GPIO-only tests and the abandoned
RealTimeThread class added during development. At the user's request, libraries
already present in the last commit are retained: SerialPipe, SerialStream,
DebounceIn, Chirp and Eigen are restored exactly from HEAD, even though they are
currently unused. Top-level CMake is also restored exactly; it has no change
relative to the last commit. The earlier removal of the 200 µs forwarding task
remains part of the accepted two-task architecture, not this library cleanup.
Kept the encoder, PWM, filter, PI controller, optional GPA, thread flag and SPI
libraries, native/Python clients, quiet Python benchmark, relevant regression
tests and fault checker. The seven Python tests retain command expiry, protocol
ordering/validation and NSS ioctl/cleanup coverage; only two obsolete GPIO-handshake
tests were removed from the previous nine.

The six overlapping reports/plans were consolidated here, including the previously
scratch-only ARM comparison. README now describes only the active workflow.
Comments explain snapshot/DMA ownership, state transitions and reception-time
expiry. The test helper was renamed from `cancel_request` to `cancel_selection`
to match NSS; there is no behavioral change. The user's Python print interval of
500 was preserved. Existing staged entries were not rewritten, so review/stage
the final working tree together rather than committing the old index alone.

After reconfiguration, the normal MCU and C++ builds produced binaries identical
to the accepted NSS versions:

- MCU SHA-256: `f3aeffbff2a423b644f1fd0ea53e352b311a34e2c70db0cc4728afee75d15169`.
- Native client SHA-256: `a698896515134e0a7f456e1e6c9d6c6b933047131c36b895c205b08c40caa65e`.

Seven Python regression tests and the native protocol test passed. Binary identity
is stronger evidence of unchanged MCU/native execution than a similar timing
average. Historical acceptance measurements therefore still describe the same
normal executable. Current limitations above remain; this cleanup does not turn
finite bench tests into a hard real-time guarantee.

The diagnostics and GPA branches were also compiled from the cleaned sources in
an isolated directory, leaving production flags off. Python syntax-tree comparison
against the pre-cleanup snapshot confirmed no logic change after accounting for
the cancellation helper rename and help text. Local documentation links, unused
dependency references, whitespace, configuration preservation and the staged diff
were checked; no staging or commit operation was performed.

Post-cleanup hardware acceptance also passed with the normal firmware flashed
and verified:

- C++: 10,000 disabled cycles at 2 kHz, then 90 faults / 9,000 recovery cycles.
  Mean work 155.338 µs, p99 work 159.685 µs, maximum work 223.315 µs; zero work
  overruns. Completion interval mean 501.492 µs, p99 505.222 µs, max 578.444 µs.
- Python: 1,000 disabled cycles at 2 kHz, then 90 faults / 9,000 recovery cycles;
  mean baseline interval 0.502 ms.
- Normal `python/main.py`: two 500-cycle reporting windows produced valid replies;
  Ctrl+C exited successfully. This preserves the user's normal entry point.
- Non-halting register checks again verified expiry during silence, READ-only
  traffic and bad CRCs; fresh commands maintained enable; disable and invalid
  current/enable/reserved payloads cleared enable/PWM.

No timed comparison was run concurrently with compilation. Current-loop
instrumentation, reset/signal and CPU-load tests from the preceding NSS acceptance
were not repeated wholesale: the firmware/native binaries are identical and those
checks already passed. This avoids presenting repeated averages as new guarantees.
The cleanup did not install any experimental firmware; normal NSS remains flashed.

The pre-cleanup source snapshot, staged/working diffs and all available experiment
artifacts are in `build/checkpoints/pre-student-cleanup/`. Its `final-validation/`
directory contains cleanup acceptance logs. The earlier NSS and REQUEST/READY
checkpoints remain available locally; generated build/checkpoint files are ignored
by Git and are not required by students. They are deliberately retained for recovery.

Restoring the inherited libraries and original CMake include path was followed by
a successful reconfigure/rebuild. The normal firmware is still byte-for-byte
identical to the accepted NSS image (SHA-256 above); no reflash was needed.

## Final minimization against the last pushed revision

The user requested a narrower student version after the first cleanup. Before
removing further development-only code, its role and evidence are recorded here:

- `host/benchmark_python.py` was a quiet, bounded Python recorder used to compare
  language overhead with the native client. Its methodology, tables, outliers
  and raw artifacts are preserved in the historical sections above. Students
  use `python/main.py`; native timing and CSV remain in `host/main.cpp`.
- `MPC_CONTROL_DIAGNOSTICS` added optional current-release counters, timestamps,
  late/coalesced-release detection and `getTiming()`. It was disabled in the
  working firmware. Its exact measurement exclusions and results are preserved
  above, including the startup overrun. The temporary diagnostic source snapshots
  remain in the ignored checkpoints; this profiling mode is removed from the
  student current loop.
- `MPC_SPI_DIAGNOSTICS` enabled optional serial printing from main. That output
  was also disabled in the working firmware and is removed. SPI diagnostic
  counters and `getDiagnostics()` remain available for fault debugging, retaining
  the diagnostic concept already present in the last commit.

Snapshots of these files before removal are in
`build/checkpoints/pre-final-minimization/`. Historical references to their CLI
commands or diagnostic flags describe the old snapshots, not the student version.
The current README is the authoritative guide to the remaining interfaces.
The hardware-only checker has since moved from `python/check_spi.py` to
`tests/check_spi.py`; historical commands below/above retain their original paths.
Protocol/current-command regression tests and the disabled-command hardware fault
checker are retained for student maintenance. No inherited utility library or
controller/GPA feature is removed. No wire format, delays, priorities, timeout,
controller equation or output ordering is changed by this step.

Final minimization verification: MCU build, seven Python regression tests, native
protocol test and whitespace checks passed. MCU firmware and native client remain
byte-for-byte identical to the accepted NSS versions (hashes above), so no further
hardware test or flash was needed. Inherited unused libraries and top-level CMake
have zero diff against HEAD. The remaining tracked implementation changes are the
configuration centralization, current-task/command-expiry integration, removal of
the forwarding task, NSS transport and Pi read-before-command loop. New native
client/transport and maintenance tests are retained deliberately; no new control
feature or protocol extension was added in this minimization.

Final source/comment/documentation review after moving the hardware checker to
`tests/check_spi.py`: MCU/native builds, seven Python tests, native protocol test,
checker CLI import, local documentation links and whitespace checks all passed.
Both normal binaries still match the hardware-validated NSS checkpoint exactly;
no hardware tests were repeated in this final review. No further runtime changes
were found necessary. The Git index still contains the older checker path until
the final move and documentation edits are staged; the working tree is authoritative.

## Periodic Linux scheduling comparison — 2026-09-29
The RealtimeThread example prompted an isolated test of periodic scheduling,
without adding a thread framework or changing the production host/MCU code.
The example header alone does not establish its wait/locking implementation.

Three C++ variants used identical added measurement code and the same current
18-byte NSS protocol, 30 MHz request, 30 µs setup/high margins, 0.08 A current
setpoint after the first disabled cycle, and final-disable handling. All ran on
isolated CPU 3 with SCHED_FIFO priority 50. Sources were copied from commit
`9f244f3`; production settings remain 1 kHz / 20 nominal seconds.

- Baseline: next release is the previous actual iteration start plus 250 µs.
- Absolute sleep: scheduled release advances by 250 µs on CLOCK_MONOTONIC using
  clock_nanosleep with TIMER_ABSTIME, independent of actual wake-up time.
- Timerfd: a periodic CLOCK_MONOTONIC timer with blocking expiration-count reads.

For the latter two, releases crossed during work are discarded before waiting
for the next future release. A wait delayed by whole periods executes only the
latest release and counts the missed ones. Small wake-up lateness does not skip
an iteration. The first iteration is immediate, as in the original client.
No signals, mutexes, additional control threads or per-cycle heap allocations
were introduced by the scheduling helpers.

Each variant ran three times for 40,000 cycles (10 nominal seconds), rotating
order: baseline/absolute/timerfd, timerfd/baseline/absolute,
absolute/timerfd/baseline. All 360,000 cycles completed with exit status zero,
valid replies and no final-disable errors. No compilation ran during timing.
Both fixed variants recorded zero skipped releases in ordinary traffic. All
variants completed every measured READ-to-COMMAND work interval before its next
scheduled release in these runs. Reply validation is not an applied-command ACK.

Times below are µs. Percentiles pool samples across repetitions. Rate is measured
between first and last starts, averaged over runs; CPU is mean percentage of one
core. Startup is excluded from release-lateness and interval distributions.

| Variant | Effective Hz | Mean SPI | p99 SPI | p99 completion interval | Max completion interval | p99 release lateness | CPU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline | 3979.647 | 158.964 | 160.593 | 252.741 | 309.981 | 1.722 | 21.48% |
| absolute | 4000.000 | 158.968 | 160.186 | 251.056 | 272.871 | 1.732 | 21.61% |
| timerfd | 4000.002 | 159.028 | 160.814 | 251.315 | 281.018 | 2.001 | 21.63% |

The baseline mean completion interval was 251.278 µs; both fixed variants were
approximately 250.000 µs. Baseline release lateness is relative to its continually
shifted target, not the original fixed time grid. Therefore similar lateness
numbers do not imply equivalent long-term phase stability. Absolute sleep and
timerfd maximum start intervals were 255.333 / 257.592 µs, versus 260.056 µs for
baseline. Completion intervals also contain variation in communication duration.

Mean READ-start through COMMAND-completion work, including the placeholder
computation between calls, was 158.995 / 159.000 / 159.059 µs respectively.
It excludes post-command logging/bookkeeping and sleep. Maximum work was
223.500 / 183.000 / 205.686 µs. No SPI-call sum exceeded 250 µs.
The recorded run wall time excludes the last inter-cycle sleep in all prototypes;
rate comparison therefore uses start-to-start timing rather than count/wall time.

A separate 1,000-cycle run per fixed variant injected a 1 ms sleep after cycle
100, outside the measured communication work. Both counted four skipped releases,
advanced the target by 1,250 µs, and completed the remaining cycles successfully.
There were no duplicate scheduled releases or catch-up iterations for those four
missed periods. This checks overrun bookkeeping separately from normal jitter.

**Conclusion:** fixed absolute scheduling is the preferred candidate. It removed
accumulated drift with nearly unchanged CPU use and communication time. Timerfd
provided no demonstrated timing benefit here and requires more setup/cleanup.
Neither variant establishes a hard deadline. The slightly better finite-run
maxima do not prove a better worst-case bound. These runs did not include a real
controller workload, intentional background stress or physical actuation checks.

Production code was deliberately left unchanged for review of these results.
The next proposed change is only the fixed release schedule with explicit skipped-
release handling, not the full RealtimeThread abstraction. SPI should remain as is.

Implementation, source-generation script, binaries, nine raw comparison CSVs,
forced-stall CSVs, exit statuses and aggregate JSON are stored locally in
`host/build/periodic-comparison-20260929/` (Git-ignored). `prepare.py` captures how
variants were derived, `periodic.h` holds their scheduling logic, `run.py` records
run order, and `analyze.py` recomputes the summary. The normal clients, their saved
logs, firmware, kernel settings and README launch commands were not modified.

API references: [clock_nanosleep](https://man7.org/linux/man-pages/man2/clock_nanosleep.2.html)
and [timerfd](https://man7.org/linux/man-pages/man2/timerfd_create.2.html).

## Fixed scheduling adopted in both clients — 2026-09-29

The user requested direct, minimal repository changes after the prototype
comparison. Both host loops now maintain a fixed monotonic release grid. C++
retains CLOCK_MONOTONIC/TIMER_ABSTIME sleeps; Python uses monotonic_ns to compute
the remaining time passed to time.sleep. No timerfd, extra thread, dependency,
new scheduling class, SPI change or MCU change was introduced.

Releases crossed during work are skipped before sleeping; a wake delayed by whole
periods executes only the latest release. Each client counts skipped releases and
prints the total to stderr after final-disable handling and SPI close. The CSV
schema is unchanged. Small lateness does not shift subsequent scheduled releases.
There is no final inter-cycle sleep. Trun still specifies a number of executed
cycles, so missed releases extend elapsed time. Skipped releases do not count
all forms of jitter: completion intervals can exceed a period without a skipped
release when communication duration varies between cycles.

Production defaults remain 1 kHz, 20 nominal seconds, 30 MHz, current mode and
0.08 A. SPI setup/high margins and command validation/limits are unchanged.
Both normal clients ran directly from the repo for 20,000 cycles each. Then
isolated copies with only period/runtime changed ran three 40,000-cycle trials
per language at 4 kHz (10 nominal seconds), alternating language order. All used
CPU 3 and FIFO priority 50. Nothing compiled during the hardware timing runs.
All 280,000 normal cycles completed without communication or final-disable errors.

Percentiles pool per-cycle CSV samples across repetitions. CPU is average process
CPU percentage of one core; rate is estimated from rounded completion-interval
samples, so tiny deviations above exactly 4,000 Hz are not significant.
All times below are µs.

| Client | Target Hz | Cycles | Effective Hz | Mean SPI | p99 SPI | Max SPI | p99 dt | Max dt | CPU | Skipped releases |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| python | 4000 | 120000 | 3999.9 | 166.3 | 168.8 | 330.5 | 252.0 | 418.4 | 26.11% | 5 |
| cpp | 4000 | 120000 | 4000.0 | 159.0 | 160.8 | 242.6 | 251.3 | 332.0 | 21.60% | 0 |
| python-default | 1000 | 20000 | 1000.0 | 166.6 | 170.7 | 267.6 | 1005.2 | 1082.7 | 6.58% | 0 |
| cpp-default | 1000 | 20000 | 1000.0 | 159.1 | 161.3 | 191.8 | 1002.2 | 1037.0 | 5.41% | 0 |

At 4 kHz Python had five SPI-call sums exceeding 250 µs; C++ had none. The reported
skip totals were Python 1/1/3 and C++ 0/0/0 across repetitions. The ordinary target
rate is now maintained without accumulated start-relative drift, but rare timing
excursions remain. Python's worst observed communication took 330.5 µs; C++'s took
242.6 µs. C++ remains the stronger candidate for 4 kHz control. Neither result
establishes a worst-case bound or includes real MPC computation.

A separate 1,000-cycle test per language injected a 1 ms pause after cycle 100.
Both completed, counted exactly four skipped releases, and produced the next
completion interval around 1.25 ms (C++ 1.2500, Python 1.2497), rather than queuing
four catch-up iterations. Python tests exercise the actual loop with a controlled
clock: a work overrun, a delayed wake-up, and small lateness that must not shift the
schedule. All three pass, as do three existing SPI transport tests and the MCU
motor-command validation/expiry/wrap test.

MCU and C++ builds and Python byte-compilation passed. The rebuilt MCU binary is
identical to its pre-change version, so no flash was necessary. Default firmware
and SPI transport behavior were retained; only host release scheduling changed.
README pacing/reporting instructions were updated. All clients closed normally.

Raw logs, stderr skip counts, pre-change sources, 4 kHz source copies/binary,
injected-stall sources/binary, and aggregate statistics are saved locally in
`host/build/fixed-production-20260929/`. `analyze.py` reconstructs the aggregate
report; `run.py` records the normal run sequence. The earlier prototype results
above remain historical evidence; their statement that production was unchanged
was superseded by this adoption.

## Repeated 2 kHz and 1 kHz tests — 2026-09-29

Following concerns about 4 kHz completion-interval maxima, both updated clients
were retested at 2 kHz and 1 kHz. Each client/rate ran three times for 10 nominal
seconds: 60,000 cycles per client at 2 kHz and 30,000 per client at 1 kHz.
Rate and client order alternated between repeats. All 180,000 cycles completed
with valid replies, exit code zero and no final-disable errors. Tests used CPU 3,
FIFO priority 50, requested 30 MHz, the existing 0.08 A current-mode setpoint,
18-byte frames and unchanged NSS margins. Test copies changed only period/runtime;
repository defaults, normal binaries and earlier logs were not overwritten.
No compilation ran concurrently with timing. These are ordinary bench runs,
without injected stalls or a real MPC workload.

Times are µs; percentiles pool individual samples across three runs. CPU is the
mean percentage of one core. Skips are missed scheduled releases, not a count of
all completion intervals longer than their target.

| Rate | Client | Mean SPI | p99 SPI | Max SPI | Mean dt | p99 dt | Max dt | CPU | Skipped releases |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2000 Hz | python | 166.9 | 174.8 | 454.3 | 500.0 | 505.5 | 973.6 | 13.23% | 1 |
| 2000 Hz | cpp | 159.0 | 161.3 | 227.6 | 500.0 | 502.2 | 554.5 | 10.80% | 0 |
| 1000 Hz | python | 166.7 | 172.8 | 288.5 | 1000.0 | 1006.8 | 1143.9 | 6.59% | 0 |
| 1000 Hz | cpp | 159.1 | 161.1 | 178.8 | 1000.0 | 1002.2 | 1016.8 | 5.42% | 0 |

C++ had zero skipped releases at both rates. Its maximum completion interval
exceeded the target by 54.5 µs (10.9%) at 2 kHz and 16.8 µs (1.68%) at 1 kHz.
Python skipped one release in its second 2 kHz run, with a worst completion gap
of 973.6 µs. Its 1 kHz runs skipped no releases but reached 1,143.9 µs between
completions. No measured SPI-call sum exceeded its respective period, including
that Python run: wake-up delays and work outside the measured SPI calls also
consume schedule time. This data does not isolate the cause of the Python skip.

Completion intervals above 110% of target: Python/C++ 39/1 at 2 kHz and 1/0 at
1 kHz. Lower frequency provides more compute headroom; it does not eliminate
outliers or establish a bound. The worse Python maximum than in the earlier
4 kHz test should not be read as proof that reducing rate causes more jitter;
these are separate finite samples. C++ at 1 kHz had the tightest relative observed
completion timing of these runs. Required controller tolerance remains undefined.

Validated CSVs, per-run skipped counts and exit status, test source/binary copies,
run order and analysis scripts, and aggregate JSON are saved locally under
`host/build/rate-comparison-20260929/`. `RESULTS.md` duplicates this report.

## Commit review and retained operating rate — 2026-09-29

Decision: retain 1 kHz (`PERIOD_US = 1000`) in both production clients, with
20 nominal seconds / 20,000 cycles. Higher-rate tests remain experiments. The
README now states this explicitly and describes fixed scheduling, skipped-release
reporting, final-cycle behavior and the absence of hard timing guarantees.

Final review found no blocking defect in the changed scheduling paths. The source
matches the hardware-tested versions, with only rate/runtime differing in the
isolated higher-rate copies. Changes remain limited to the two host loop files,
three focused Python scheduling regression tests, and documentation. MCU sources,
SPI transport, command format/limits, priorities, NSS margins and build files are
unchanged. Comments describe the scheduling and overrun policy implemented.

Both builds, six host tests, the MCU command-policy test, Python byte-compilation
and whitespace checks passed again. The MCU binary remains identical to the
pre-change firmware. No further hardware runs were needed for documentation-only
edits. The accepted 1 kHz timing evidence is above; it is not a worst-case bound
or a guarantee of bug-free operation. No commit or staging operation was performed.
Include `host/tests/test_periodic.py` when staging the five changed/new files.
