# MPC Demonstrator — Host

Minimal C++17 UART host for the MPC Demonstrator MCU. Supports Linux and Windows
using native serial APIs, with no external libraries. The MCU firmware is maintained
on the `MCU` branch; this is the `Host` branch.

The initial application uses one communication thread and blocking serial I/O.
Its dummy controller always commands **0 A with the motor disabled**. TinyMPC is
not integrated yet.

## Layout

- `src/main.cpp`: application, packet encoding/decoding, communication loop, and dummy controller.
- `lib/SerialPort/`: Linux/Windows serial implementation and interface.
- `include/host_config.h`: sample-output settings.
- `docs/`: UART protocol and timing notes.
- `matlab/`: reserved for host-related scripts and analysis.
- `CMakeLists.txt`: host build configuration.

## Build and run

Requires CMake 3.16 or newer and a C++17 compiler.

Linux:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/uart_host /dev/ttyACM0
```

Windows with Visual Studio/MSVC:

```text
cmake -S . -B build
cmake --build build --config Release
build\Release\uart_host.exe COM5
```

Pass the actual serial port as the sole argument. On Linux, the user must have
permission to access that device. Stop the application with Ctrl+C.

## Behavior

At startup the host opens the port, clears pending RX/TX bytes once, attempts to
raise the communication thread priority, and sends a disabled, zero-current command.
It then receives measurements, calls `controller()`, and sends the next command.
There are no application-level queues or asynchronous I/O operations.

The intended communication rate is 500 Hz. The MCU independently checks for new
data on its faster ticker. The imported host has no explicit 500 Hz timer; its
frequency constant only supplies the displayed expectation. Actual exchange timing
must be measured on hardware. See [UART protocol and timing](docs/uart_protocol.md).

Sample output is configured in `include/host_config.h`:

- Comment out `#define UART_HOST_PRINT_SAMPLES` to disable sample output and timing statistics. Startup and error messages remain enabled.
- Set `UART_HOST_PRINT_EVERY_N` to a positive integer. The default is 10: nominally 50 printouts/s at 500 samples/s. Rebuild after changing either setting.

Each printout contains the latest motor angle, pendulum angle, and current, plus
`dt_avg`, `dt_min`, and `dt_max` in milliseconds for the intervals since the previous
printout. The first sample establishes the clock baseline: the first 10-sample
window has 9 intervals; subsequent windows have 10. With N=1, the first printout
has measurements only. An incomplete final window is not printed.

Printing runs in the communication thread before the next command is sent, so
slow console output can delay communication. The print rate follows the actual
sample rate; it is not controlled by a separate 50 Hz timer.

Thread priority is best effort: `SCHED_FIFO` priority 50 on Linux and
`THREAD_PRIORITY_HIGHEST` on Windows. If setting it fails, execution continues
with a warning.

The host stops its communication loop on serial errors or read timeout, with no
automatic reconnection or packet resynchronization. The imported program currently
returns exit status zero after communication errors; missing arguments return one.
The dummy controller remains in `src/main.cpp` for subsequent controller integration.

Linux serial writes have no explicit timeout. Ctrl+C terminates the process; it
does not send a final disable command. See the protocol notes for the existing
MCU watchdog and packet-framing limitations.

## Validation

The host builds on Linux with GCC and cross-compiles for Windows with MinGW-w64.
Linux pseudo-terminal checks cover packet exchange, fragmented responses, timeout,
disconnect, binary data, and sample-output settings. These checks do not establish
physical UART timing. Native Windows/MSVC and MCU hardware testing remain required.
