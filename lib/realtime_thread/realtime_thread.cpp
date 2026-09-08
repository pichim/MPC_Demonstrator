#include "realtime_thread.h"

#include <cstdint>
#include <cstring>

realtime_thread::realtime_thread(IO_handler &io, float Ts, float Ts_fast)
    : thread(osPriorityHigh1, OS_STACK_SIZE)
    , Ts(Ts)
    , io_handler(io)
    , serialPipe(USBTX, USBRX, BAUD, 8, 5)
    , fast_rt_thread(io, Ts_fast)
    // , m_SerialStream(PB_10, PC_5, 30, 2000000)
    // , m_Chirp(F0_HZ, (1.0f / 2.0f) / Ts, T1_SEC, Ts)
{
}

realtime_thread::~realtime_thread() {}

void realtime_thread::loop(void)
{
    char rx_buf[4 + 1];     // 1 float (4 bytes) + 1 enable byte
    char tx_buf[4 + 4 + 4]; // 3 floats: motor_angle, pendulum_angle, current

    bool is_enabled = false;

    uint32_t watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT_SEC / Ts + 0.5f);

    // m_Timer.start();
    // m_time_previous_us = m_Timer.elapsed_time();

    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        // // Measure delta time since last cycle (unused, kept as reference)
        // const microseconds time_us = m_Timer.elapsed_time();
        // const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
        // m_time_previous_us = time_us;

        if (serialPipe.readable()) {

            io_handler.set_enable_rtt_do(true);

            // Read values (setpoint and enable) from UART
            const int nread = serialPipe.get(rx_buf, sizeof(rx_buf), true);
            if (nread != static_cast<int>(sizeof(rx_buf))) {
                // Error or incomplete frame: treat as "no valid communication" for watchdog
                if (watchdog_counter > 0) {
                    watchdog_counter--;
                    if (watchdog_counter == 0 && is_enabled) {
                        // Watchdog timeout: no valid communication for WATCHDOG_TIMEOUT_SEC seconds
                        is_enabled = false;
                        io_handler.set_enable_motor(false);
                        fast_rt_thread.updateState(false, 0.0f);
                    }
                }
                continue;
            }

            // We have a valid fresh packet -> reset watchdog
            watchdog_counter = (uint32_t)(WATCHDOG_TIMEOUT_SEC / Ts + 0.5f);

            // From host: 1 float value (4 bytes) + enable (1 byte) are sent
            float current_cmd = 0.0f;
            memcpy(&current_cmd, &rx_buf[0], sizeof(float)); // assumes little-endian
            const bool enable_cmd = (rx_buf[4] == 1);        // uint8 value -> bool

            // Enable is simply set by the command itself
            is_enabled = enable_cmd;
            io_handler.set_enable_motor(is_enabled);

            // Drive fast loop according to current enable state
            float current = 0.0f;
            float motor_angle = 0.0f;
            float pendulum_angle = 0.0f;
            if (is_enabled)
                fast_rt_thread.updateStateAndReturnMeasurements(true, current_cmd, current, motor_angle, pendulum_angle);
            else
                fast_rt_thread.updateStateAndReturnMeasurements(false, 0.0f, current, motor_angle, pendulum_angle);

            // Write encoder values + actual current back to host (3 floats)
            memcpy(&tx_buf[0], &motor_angle, sizeof(float));
            memcpy(&tx_buf[4], &pendulum_angle, sizeof(float));
            memcpy(&tx_buf[8], &current, sizeof(float));
            serialPipe.put(tx_buf, sizeof(tx_buf), true);

            io_handler.set_enable_rtt_do(false);

        } else {
            if (watchdog_counter > 0) {
                watchdog_counter--;
                if (watchdog_counter == 0 && is_enabled) {
                    // Watchdog timeout: no communication for WATCHDOG_TIMEOUT_SEC seconds
                    // -> force motor and current loop disabled
                    is_enabled = false;
                    io_handler.set_enable_motor(false);
                    fast_rt_thread.updateState(false, 0.0f);
                }
            }
        }
    }
}

void realtime_thread::start_loop(void)
{
    fast_rt_thread.start_loop();

    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}
