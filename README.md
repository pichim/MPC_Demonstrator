# MPC Demonstrator — Host

Minimal C++17 UART host for the MPC Demonstrator MCU. Supports Linux and Windows
using native serial APIs, with no external libraries. The MCU firmware is maintained
on the `MCU` branch; this is the `Host` branch.

The initial application uses one communication thread and blocking serial I/O.
Startup commands **0 A with the motor disabled**. After the first response, the
constant-current test commands **0.05 A with the motor enabled**. TinyMPC is not
integrated yet.

## Layout

- `src/main.cpp`: application, packet encoding/decoding, communication loop, and dummy controller.
- `lib/SerialPort/`: Linux/Windows serial implementation and interface.
- `include/host_config.h`: sample-output settings.
- `docs/`: UART protocol and timing notes.
- `matlab/`: reserved for host-related scripts and analysis.
- `CMakeLists.txt`: host build configuration.
- `.clang-format`: shared C++ formatting settings.
- `.gitignore`: generated build directories and the timing log.
- `LICENSE`: existing GNU GPL version 3 license text.

## Build and run

Requires CMake 3.16 or newer and a C++17 compiler. Run the following commands
from the project root. Replace the example serial port with the Nucleo's actual
port. Stop an existing host before starting another run.

### Linux (Bash)

Build:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Run with console output:

```sh
sudo ./build/uart_host /dev/ttyACM0
```

Run with output and errors written to a text file:

```sh
sudo ./build/uart_host /dev/ttyACM0 > uart_timing.txt 2>&1
```

`sudo` allows the worker to request `SCHED_FIFO` priority 50. Verify from another
terminal while the host is running:

```sh
ps -T -C uart_host -o pid,tid,cls,rtprio,comm
```

The communication thread should show `FF` and `50`. The main thread normally
shows `TS` because it only waits for the worker.

### Windows (Command Prompt / cmd.exe)

Use a Visual Studio Developer Command Prompt with the C++ tools and CMake
installed. The Visual Studio 2022 example requires CMake 3.21 or newer
([generator documentation](https://cmake.org/cmake/help/latest/generator/Visual%20Studio%2017%202022.html)).
These run commands use the `cmd.exe` built-in `start`, not PowerShell's
`start` alias.

Build with the Visual Studio generator (example for Visual Studio 2022):

```bat
cmake -S . -B build-msvc -G "Visual Studio 17 2022" -A x64
cmake --build build-msvc --config Release
```

Run with console output:

```bat
start "" /high /wait build-msvc\Release\uart_host.exe COM5
```

Run with output and errors written to a text file:

```bat
start "" /high /wait cmd /c "build-msvc\Release\uart_host.exe COM5 > uart_timing.txt 2>&1"
```

`/high` selects the High process priority class at launch. The application then
requests `THREAD_PRIORITY_HIGHEST` for its communication thread. Administrator
mode is not normally required for this priority class. In Task Manager's Details
tab, right-click `uart_host.exe` and check that Set priority shows High; this
checks the process class, not the worker's individual priority.

The command opens a separate console and waits for the host to exit. Press Ctrl+C
in that console to stop the host, including when its output is redirected.

### Cross-build Windows from Linux (MinGW-w64)

Requires `x86_64-w64-mingw32-g++` (already installed on this development machine):

```sh
cmake -S . -B build-windows \
  -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++ \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXE_LINKER_FLAGS=-static
cmake --build build-windows
```

Copy `build-windows/uart_host.exe` to Windows. From Command Prompt in its directory,
use the same priority and output options:

```bat
start "" /high /wait uart_host.exe COM5
```

Or, for a text file:

```bat
start "" /high /wait cmd /c "uart_host.exe COM5 > uart_timing.txt 2>&1"
```

The static link bundles the compiler runtime libraries. This build was checked to
import only Windows system DLLs (`KERNEL32.dll` and `msvcrt.dll`), with no separate
MinGW runtime DLLs. It is a Windows binary, not a Linux executable. Separate build
directories prevent mixing compilers and are ignored by Git.

### Common behavior and platform differences

Both platforms flush each completed output line, including when redirected to a
file. `stdbuf` is no longer needed. `>` overwrites `uart_timing.txt` each run;
`2>&1` includes error messages. File I/O still runs in the communication thread.
Ctrl+C does not print an incomplete statistics window or send a final disable
command.

If setting the worker priority fails, the host prints a warning and continues.
Linux `SCHED_FIFO` and Windows High process / Highest thread priority are different
scheduler policies, not equivalent real-time guarantees. The UART protocol,
200 Hz pacing, current command, statistics, and output format are the same.
Actual wake-up timing depends on the OS and hardware.
In particular, raising priority does not guarantee precise 5 ms wake-ups on
Windows; timer resolution and scheduling still affect waits. The application
does not change the Windows timer resolution.

Windows priority and launch behavior follow Microsoft's
[Scheduling Priorities](https://learn.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities)
and [start command](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/start)
documentation. Windows execution and hardware timing still need native testing;
a cross-build verifies compilation and linking only.

## Behavior

Configuration is deliberately kept in the source; rebuild after changing it:

| Setting | Location | Current value |
|---|---|---|
| UART baud | `BAUD` in `src/main.cpp` | 115200 |
| Host command rate | `COMMUNICATION_FREQUENCY_HZ` in `src/main.cpp` | 200 Hz |
| Running current command | `controller()` in `src/main.cpp` | 0.05 A, enabled |
| Startup command | `Command` defaults in `src/main.cpp` | 0 A, disabled |
| Sample output | `include/host_config.h` | Enabled, every 10 samples |
| Serial timeouts | Platform-specific constants in `lib/SerialPort/serial_port.cpp` | 20 ms per read wait/call; also per Windows write call |

The serial port is the only command-line argument. There is no automatic port
selection or runtime controller configuration. Both platforms target little-endian
systems with 32-bit IEEE-754 floats.

At startup the host opens the port, clears pending RX/TX bytes once, attempts to
raise the communication thread priority, and sends a disabled, zero-current command.
It then receives measurements, calls `controller()`, and sends the next command.
There are no application-level queues or asynchronous I/O operations.

The host targets 200 Hz by waiting until at least 5 ms after the previous command
start before sending the next command. It waits for each response first; late
cycles do not cause catch-up bursts. OS scheduling and transport delays can lower
the actual rate and cause response timing jitter. The MCU polling interval is
unchanged. See [UART protocol and timing](docs/uart_protocol.md).

Sample output is configured in `include/host_config.h`:

- Comment out `#define UART_HOST_PRINT_SAMPLES` to disable sample output and timing statistics. Startup and error messages remain enabled.
- Set `UART_HOST_PRINT_EVERY_N` to a positive integer. The default is 10: nominally 20 printouts/s at 200 samples/s. Rebuild after changing either setting.

Each printout contains the latest motor angle, pendulum angle, measured current,
and the preceding command (`current_cmd` in amperes and `enable`), plus
`dt_avg`, `dt_min`, and `dt_max` in milliseconds for the intervals since the previous
printout. The first sample establishes the clock baseline: the first 10-sample
window has 9 intervals; subsequent windows have 10. With N=1, the first printout
has measurements only. An incomplete final window is not printed.

Printing runs in the communication thread before the next command is sent, so
slow console output can delay communication. The print rate follows the actual
sample rate; it is not controlled by a separate print timer.

Thread priority is best effort: `SCHED_FIFO` priority 50 on Linux and
`THREAD_PRIORITY_HIGHEST` on Windows. If setting it fails, execution continues
with a warning.

The host stops its communication loop on serial errors or read timeout, with no
automatic reconnection or packet resynchronization. The imported program currently
returns exit status zero after communication errors; an incorrect argument count returns one.
The dummy controller remains in `src/main.cpp` for subsequent controller integration.

Linux serial writes have no explicit timeout. Ctrl+C terminates the process; it
does not send a final disable command. See the protocol notes for the existing
MCU watchdog and packet-framing limitations.

## Validation

The host builds on Linux with GCC and cross-compiles for Windows with MinGW-w64.
Linux pseudo-terminal checks cover packet exchange, fragmented responses, timeout,
disconnect, binary data, and sample-output settings. These checks do not establish
physical UART timing. Native Windows/MSVC and MCU hardware testing remain required.
The 200 Hz pacing must be measured on the real device on each OS; the earlier
Linux hardware captures at 500 Hz do not validate this new pacing.
