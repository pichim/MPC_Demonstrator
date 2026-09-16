# MPC Demonstrator — MCU

Mbed CE firmware for **NUCLEO-F446RE**. The Pi host communicates over USART3
at **230400 baud**. ST-LINK USB remains available for power and flashing.

## Configuration and wiring

`include/config.h` defines `MPC_UART_TX_PIN` (PC10), `MPC_UART_RX_PIN` (PC11)
and `MPC_UART_BAUD` (230400). The host must use the same baud rate.

| Pi 5 physical pin | Nucleo CN7 pin |
|---|---|
| 8 — GPIO14 TX | 2 — PC11 RX |
| 10 — GPIO15 RX | 1 — PC10 TX |
| 6 — GND | 8 — GND |

Wire with power off, use 3.3 V logic, and connect no power pins between boards.
The host README describes Pi UART setup. To use ST-LINK serial instead, change
the pin macros to `USBTX`/`USBRX`, set `MPC_UART_BAUD` to 115200, and set the
Host `BAUD` to 115200 as well. Rebuild both programs and reflash the MCU.

## Build and flash

With the existing configured Arm/Mbed CE toolchain:

```sh
cmake --build build/NUCLEO_F446RE-Develop --target MPC_Demonstrator -j2
```

Flash `build/NUCLEO_F446RE-Develop/MPC_Demonstrator.bin` onto the Nucleo's
`NOD_F446RE` drive, or use the existing VS Code build/flash task. Stop the host
before flashing. Building alone does not flash the board.

## Behavior

The communication ticker is 200 µs; the current-control ticker is 50 µs.
The host targets 500 Hz, initially commanding 0 A disabled, then 0.05 A enabled.
UART uses 8N1, no flow control and little-endian 32-bit floats:

- Request: current command float + enable byte (5 bytes).
- Response: motor angle, pendulum angle and current floats (12 bytes).

Packets have no framing/CRC or command range validation. A partial command can
block reception and stall the nominal 0.3 s watchdog. Ctrl+C on the host sends
no final disable command. The motor fault input is currently unused.
