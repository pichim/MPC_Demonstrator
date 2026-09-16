#include "realtime_thread.h"
#include "config.h"

#include <cmath>

realtime_thread::realtime_thread(IO_handler &io, float Ts, float Ts_fast)
    : thread(osPriorityHigh1, OS_STACK_SIZE)
    , Ts(Ts)
    , io_handler(io)
    , spi(MPC_SPI_MOSI_PIN, MPC_SPI_MISO_PIN, MPC_SPI_SCK_PIN, MPC_SPI_NSS_PIN,
          osPriorityHigh1, OS_STACK_SIZE)
    , fast_rt_thread(io, Ts_fast)
    // , m_SerialStream(PB_10, PC_5, 30, 2000000) // PB_10 is currently used by SPI SCK.
{
}

realtime_thread::~realtime_thread() {}

void realtime_thread::loop(void)
{
    Timer timer;
    timer.start();
    microseconds last_command{0};
    const microseconds timeout{static_cast<int64_t>(WATCHDOG_TIMEOUT_SEC * 1e6f)};
    bool enabled = false;
    float current_cmd = 0.0f;

    while (true) {
        ThisThread::flags_wait_any(threadFlag);
        io_handler.set_enable_rtt_do(true);
        const auto now = timer.elapsed_time();

        if (spi.hasNewData()) {
            const SpiData command = spi.getSPIData();
            // Only CRC-valid PUBLISH frames reach this point. The third float is reserved.
            const bool valid = std::isfinite(command.data[0]) &&
                               (command.data[1] == 0.0f || command.data[1] == 1.0f) &&
                               command.data[2] == 0.0f;
            enabled = valid && command.data[1] == 1.0f;
            current_cmd = enabled ? command.data[0] : 0.0f;
            if (valid)
                last_command = now;
        }

        if (now - last_command >= timeout) {
            enabled = false;
            current_cmd = 0.0f;
        }

        float current, motor_angle, pendulum_angle;
        fast_rt_thread.updateStateAndReturnMeasurements(
            enabled, current_cmd, current, motor_angle, pendulum_angle);
        io_handler.set_enable_motor(enabled);
        spi.setReplyData(motor_angle, pendulum_angle, current);
        io_handler.set_enable_rtt_do(false);
    }
}

void realtime_thread::start_loop(void)
{
    if (!spi.start()) {
        printf("SPI startup failed; motor remains disabled.\n");
        return;
    }
    fast_rt_thread.start_loop();
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal),
                  microseconds{static_cast<int64_t>(Ts * 1e6f)});
}
