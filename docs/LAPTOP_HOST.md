# Ubuntu laptop host with the Waveshare CH347T

The Waveshare USB to UART/I2C/SPI/JTAG converter (SKU 25411) can act as the
SPI master for the NUCLEO-F446RE. The laptop target uses libusb and the existing
C++ host control loop, wire protocol, and timing logs. It requests SPI mode 0,
MSB first, at 15 MHz and targets a 1 kHz host rate by default.

## Adapter and wiring

With USB unplugged, set the red mode switch to **M1** (switch 1 away from ON,
switch 2 toward ON) and the signal voltage to **3.3 V**. After reconnecting USB,
`lsusb -d 1a86:55db` should list the adapter.
USB ID `1a86:55da` indicates M0 (dual UART), and `1a86:55dc` indicates M2 (HID).
The [Waveshare manual](https://www.waveshare.com/wiki/USB_TO_UART/I2C/SPI/JTAG)
shows the switch positions and the 12-pin connector.

Disconnect the Pi's SPI wires before connecting the adapter. With both boards
unpowered, wire the labeled signals as follows. Power the Nucleo via ST-LINK
USB and leave the adapter's VCC pin disconnected.

| Waveshare SPI signal | NUCLEO-F446RE  |
| -------------------- | -------------- |
| SDO / MOSI           | PC3 / CN7-37   |
| SDI / MISO           | PC2 / CN7-35   |
| SCK                  | PB10 / CN10-25 |
| CS0                  | PB12 / CN10-16 |
| GND                  | GND / CN7-8    |

Use short leads. The bench setup had intermittent CRC errors with two ground
leads. Three passed the slower clock sweep, and four passed at 15 MHz. The
older REQUEST/READY leads on PC4 and PA8 are unused by the NSS firmware.

In the verified bench setup, the laptop connects to the CH347T by USB and the
Pi powers the Nucleo through its ST-LINK USB cable. The Pi's SPI leads stay
disconnected and the CH347T is the SPI master. When both USB cables went to the
laptop, including through a shared dongle, later tests saw invalid SPI replies.

## Build and check

From the repository root on Ubuntu 24.04, install the development packages
if needed and build the optional target.

```sh
sudo apt install g++ cmake ninja-build pkg-config libusb-1.0-0-dev
cmake -S host -B host/build/laptop -G Ninja -DCMAKE_BUILD_TYPE=Release -DMPC_BUILD_CH347=ON
cmake --build host/build/laptop --target mpc_spi_usb
host/build/laptop/mpc_spi_usb --probe
```

`--probe` configures the bridge and drives CS0 inactive. It sends zero SPI bytes
and can be run with only the USB adapter connected. If it reports
`LIBUSB_ERROR_ACCESS`, add a device-specific udev rule, then unplug and reconnect
the adapter.

```sh
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55db", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/70-mpc-ch347.rules
sudo udevadm control --reload-rules
```

With the Nucleo wired and running the matching firmware, run a disabled
communication check and analyze the CSV. Every command has zero setpoint and
enable off. The default run should finish successfully with 20,000 samples.

```sh
host/build/laptop/mpc_spi_usb --disabled > host/spi_timing_cpp.txt &&
python3 host/python/analyze_timing.py
```

Check stderr for `skipped_releases` and use the analyzer to check completion
intervals and SPI call time. A skipped release is a scheduled cycle start that
the host passed over. The host still aims to collect the configured number of
valid samples. SPI call time includes USB transfers and chip-select handling.
The host asserts CS0 and waits at least 30 µs before requesting the data
transfer. The bridge automatically releases CS0 after the 18-byte full-duplex
burst, and the host waits at least 30 µs after the USB reply before selecting
it again.

## Real-time FIFO test

From the repository root, use `sudo chrt -f 50` to run the disabled test
under the Linux `SCHED_FIFO` policy at priority 50. The command runs
without CPU pinning and works with the existing Ubuntu kernel. The laptop
host defaults to 1 kHz and 15 MHz SPI. To keep a normal-scheduling CSV for
comparison, first copy it to `host/spi_timing_cpp_normal.csv`.

```sh
sudo chrt -f 50 host/build/laptop/mpc_spi_usb --disabled > host/spi_timing_cpp.txt &&
python3 host/python/analyze_timing.py
```

For a five-minute comparison, build a separate copy with `Trun = 300.0`
in `host/src/main.cpp`. Use that same binary in the normal-scheduling and
FIFO commands above. Keep the two CSVs and record each run's
`skipped_releases` from stderr. A complete run at 1 kHz records 300,000
samples with motor enable off.

One paired 20,000-cycle test with the Pi powering the Nucleo had valid replies
throughout. The longest completion interval was 2.846 ms under normal scheduling
and 1.127 ms under FIFO 50. The FIFO run skipped zero releases. These are
bench measurements, and longer runs under the intended workload are needed
to characterize the timing tails. CPU pinning gave mixed results on this laptop.

The five-minute comparison used the same temporary build in both runs,
without CPU pinning, at 1 kHz and 15 MHz SPI. The Pi powered the Nucleo through
ST-LINK and the laptop exchanged frames through the CH347T. Both runs completed
300,000 valid exchanges with motor enable off.

| Metric | Normal scheduling | FIFO 50 |
| --- | ---: | ---: |
| Skipped releases | 421 | 9 |
| Completion interval p99.9 | 1.543 ms | 1.154 ms |
| Completion interval p99.99 | 4.006 ms | 1.268 ms |
| Completion intervals over 2 ms | 190 | 4 |
| Longest completion interval | 6.567 ms | 5.703 ms |
| Mean SPI call | 0.243 ms | 0.242 ms |

The longest FIFO interval included a 4.920 ms SPI call. The log measures
the time inside that call, and the cause of the delay remains unknown.

## Higher host rates

To compare other host rates, configure a separate build. For example, a
333 µs period targets about 3 kHz. The host skips releases it cannot service.
Compare the reported skipped releases and completion intervals across runs.

```sh
cmake -S host -B host/build/laptop-3khz -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DMPC_BUILD_CH347=ON -DMPC_CH347_PERIOD_US=333
cmake --build host/build/laptop-3khz --target mpc_spi_usb
host/build/laptop-3khz/mpc_spi_usb --disabled > host/spi_timing_cpp.txt
```

Set `MPC_CH347_SPI_SPEED_HZ=7500000` at configure time to return to 7.5 MHz.
In the initial four-ground bench runs, 15 MHz passed the read-only clock
sweep and 30 MHz failed frame validation. With 15 MHz SPI, the 1 kHz disabled
run completed 20,000 cycles with zero skipped releases. The 3 kHz disabled run
completed 60,060 cycles with valid replies and skipped 124 releases. These
measurements used normal Ubuntu scheduling.

A later repeat on the bench failed reply validation with both pinned and
unpinned runs, including with the earlier manual chip-select build. The
15 MHz read-only sweep passed in one repeat while disabled-command exchanges
still showed CRC errors. Verify the full disabled run after setting up the
wiring. A passing read-only sweep by itself leaves command-cycle reliability
unverified.

## Validation scope

The laptop bench results establish disabled-command communication on the tested
wiring and characterize timing for those runs. They leave powered motor control,
sensor behavior, operation with an MPC solver, and worst-case feedback latency
unverified. The tested arrangement also used the Pi to power the Nucleo.
A direct replacement for the Pi remains unverified.

`--enable` runs the configured control command from the second cycle onward.
Settings such as current/voltage mode, setpoint and duration are shared with
the native C++ host in `host/src/main.cpp`. The host attempts a final disabled
command after a run or a loop failure. The MCU command timeout still applies
if USB communication is lost. A telemetry reply cannot confirm command
application. Bench validation used disabled commands.
