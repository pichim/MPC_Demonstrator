# MPC Demonstrator — Host

Linux C++17 UART host for the NUCLEO-F446RE: **230400 baud, 500 Hz**.
The startup command is 0 A disabled; subsequent commands are **0.05 A enabled**.
TinyMPC is not integrated yet.

## Build and run

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
sudo ./build/uart_host /dev/ttyAMA0
```

To record output, append `> uart_timing.txt 2>&1`. Ctrl+C stops the host without
sending a final disable command. `sudo` permits best-effort `SCHED_FIFO` priority
50. No CPU pinning or memory locking is used.

## GPIO UART

Wire with power off. Use 3.3 V logic and connect no power pins between boards.

| Pi 5 physical pin | Nucleo CN7 pin |
|---|---|
| 8 — GPIO14 TX | 2 — PC11 RX |
| 10 — GPIO15 RX | 1 — PC10 TX |
| 6 — GND | 8 — GND |

Enable `dtoverlay=uart0-pi5` under `[all]` in the active boot `config.txt`.
Remove serial-console arguments from `cmdline.txt`, then reboot. Check
`pinctrl get 14-15` shows TXD0/RXD0 and no serial getty uses `/dev/ttyAMA0`.
The MCU's `include/config.h` must select PC10/PC11 at 230400 baud; rebuild and
flash after changing it. ST-LINK USB can remain connected for power/flashing.

## Settings and behavior

Baud, target rate and controller are in `src/main.cpp`; sample output is configured
in `include/host_config.h`. Rebuild after changes. A separate reporting thread
prints every 10 samples (nominally 50 Hz); its 63-report queue drops new reports
when full. Arrival-time statistics include MCU, UART and scheduling delays.

Serial errors stop communication but currently return exit status zero. The
protocol has no framing/CRC; partial commands can stall the MCU's nominal 0.3 s
watchdog. See [protocol and timing](docs/uart_protocol.md) for details.
