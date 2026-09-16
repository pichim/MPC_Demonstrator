# MPC Demonstrator — SPI MCU

NUCLEO-F446RE firmware with a Raspberry Pi SPI master. The existing 50 µs
current-control loop is unchanged; the communication loop checks new SPI commands
every 200 µs. Pins are in `include/config.h`.

## Wiring

Power off before rewiring. Use short 3.3 V signal wires and common ground;
connect no power pins between boards. Keep ST-LINK USB for power/flashing.
The old PC10/PC11 UART wires are no longer used for commands.

| Pi 5 physical pin | Signal | Nucleo-F446RE |
|---|---|---|
| 19 (GPIO10) | MOSI | PC3 — CN7 pin 37 |
| 21 (GPIO9) | MISO | PC2 — CN7 pin 35 |
| 23 (GPIO11) | SCK | PB10 — CN10 pin 25 |
| 24 (GPIO8 / CE0) | NSS | PB12 — CN10 pin 16 |
| 6 | GND | CN7 pin 8 |

Pin reference: [ST UM1724](https://www.st.com/resource/en/user_manual/dm00105823.pdf).

## Build and run

With the existing Arm/Mbed CE toolchain:

```sh
cmake -S . -B build/NUCLEO_F446RE-Develop
cmake --build build/NUCLEO_F446RE-Develop --target MPC_Demonstrator -j2
```

Flash `build/NUCLEO_F446RE-Develop/MPC_Demonstrator.bin` using the existing VS Code
flash task or copy it onto the `NOD_F446RE` drive. Stop clients before flashing.

Enable Pi SPI0 (`dtparam=spi=on` in the active boot `config.txt`, reboot if changed),
check `/dev/spidev0.0`, and install `python3-spidev` if needed. From this MCU repository:

```sh
cd ~/Mbed_CE_Programs/MPC_Demonstrator
sudo chrt -f 50 python3 -u python/main.py 2>&1 | tee spi_timing.txt
```

The client targets **500 Hz** (2 ms), SPI mode 0 at **5 MHz**. It first sends 0 A disabled,
then **0.08 A enabled** after a valid reply. Settings are at the top of
`python/main.py`. Ctrl+C or a bad reply attempts a final disable and closes SPI;
this is not an acknowledgement that the motor stopped. No CPU pinning is used.

## Protocol

Both transfers are 14 bytes: header + three little-endian float32 values + CRC-8
(poly 0x07, initial value 0, over header and payload). The Pi sends an ARM frame
(0x56, zero payload), waits at least 100 µs, then a command frame (0x55).

- Command floats: current in A, enable (exactly 0 or 1), reserved zero.
- Reply header 0x45; floats: motor angle in rad, pendulum angle in rad, current in A.

Replies contain previously prepared measurements, not command acknowledgements;
there is no sequence number or measurement timestamp. The fixed rearm gap needs
hardware validation. Only CRC-valid commands with finite current, enable 0 or 1, and reserved zero refresh the
elapsed-time 0.3 s watchdog. Invalid command payloads disable the motor; missing
or corrupt frames eventually expire the watchdog. A stalled MCU task can still
delay shutdown.

Every 10 exchanges the client prints telemetry and arrival interval statistics.
Printing and Linux scheduling can extend the period. The two transfers plus the 100 µs gap take at least
144.8 µs at the requested clock, before Python/driver overhead; 500 Hz is an
experimental target, not a guaranteed rate. Firmware builds and mocked
client checks do not validate electrical operation or physical timing. `realtime_thread` uses SPI; the UART helper libraries remain available.
The separate host repository retains the C++ UART client for use with UART firmware.
