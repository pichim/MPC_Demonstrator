#include "fast_realtime_thread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

fast_realtime_thread::fast_realtime_thread(IO_handler &io, SpiSlaveDMA &spi, float Ts)
    : thread(MPC_FAST_RT_PRIORITY, MPC_FAST_RT_STACK_SIZE)
    , Ts(Ts)
    , io_handler(io)
    , spi(spi)
{
    notchEncoders[0].notchInit(MPC_F_CUT_HZ_NOTCH, MPC_D_NOTCH, Ts);
    notchEncoders[1].notchInit(MPC_F_CUT_HZ_NOTCH, MPC_D_NOTCH, Ts);

    lowPass2CurrentSetpoint.lowPass2Init(MPC_F_CUT_HZ, MPC_CURRENT_SETPOINT_DAMPING, Ts);

    pidCntrl.setup(MPC_KP_I, MPC_KI_I, 0.0f, 0.0f, MPC_TAU_RO_I, Ts, (-MPC_POWERSUPPLY_VOLTAGE + MPC_OFFSET_VOLTAGE), (MPC_POWERSUPPLY_VOLTAGE - MPC_OFFSET_VOLTAGE));

#if MPC_PERFORM_GPA_MEAS
    // closed-loop measurement
    const float fMin = 10.0f;
    const float fMax = 0.99f / (2.0f * Ts);
    const uint16_t NfexcDes = 120;
    const float Aexc0 = 0.4f;
    const float Aexc1 = 0.3f;
    const int NperMin = 3;
    const float TmeasMin = 1.0f;
    const int NmeasMin = (int)ceilf(TmeasMin / Ts);
    const float Tstart = 1.0f;
    const int Nstart = (int)ceilf(Tstart / Ts);
    const float Tsweep = 0.3f;
    const int Nsweep = (int)ceilf(Tsweep / Ts);
    m_GPA.init(fMin, fMax, NfexcDes, NperMin, NmeasMin, Ts, Aexc0, Aexc1, Nstart, Nsweep, true, true);
#endif
}

fast_realtime_thread::~fast_realtime_thread() {}

void fast_realtime_thread::loop(void)
{

#if MPC_PERFORM_GPA_MEAS
    float exc = 0.0f;
    // Print gpa info
    m_GPA.printGPAmeasPara();
#endif

    CurrentCommand command;
    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        io_handler.set_enable_frtt_do(true);

        // Read current and filtered encoder values
        const float current = io_handler.read_current();
        const float motor_angle = notchEncoders[0].apply(io_handler.read_encoder_motor());
        const float pendulum_angle = notchEncoders[1].apply(io_handler.read_encoder_pendulum());

        // Consume the newest command only. Expiry starts at SPI receipt, not
        // here; a delayed current iteration must not extend an old command.
        SpiCommand received;
        if (spi.takeCommand(received))
            command.accept(received.data, received.received_at_us);
        command.expire(us_ticker_read(), MPC_COMMAND_TIMEOUT_US);
        spi.setReplyData(motor_angle, pendulum_angle, current);

        // Disable the bridge before resetting PWM/controller state. On enable,
        // prepare direction and PWM first, then assert the bridge enable.
        if (!command.enabled) io_handler.set_enable_motor(false);

        // Current controller
        if (command.enabled) {

            // Error
            float current_error = lowPass2CurrentSetpoint.apply(command.current) - current;
#if MPC_PERFORM_GPA_MEAS
            current_error += exc + 0.6f;
#endif
            // Controller
            const float u = pidCntrl.update(current_error, current);

            // Calculate direction and PWM value
            if (u > 0.0f)
                io_handler.set_dir(0);
            else
                io_handler.set_dir(1);

            io_handler.write_pwm_motor(clamp((fabs(u) + MPC_OFFSET_VOLTAGE) / MPC_POWERSUPPLY_VOLTAGE, 0.0f, 1.0f));
            io_handler.set_enable_motor(true);

#if MPC_PERFORM_GPA_MEAS
            // Update GPA excitation
            exc = m_GPA.update(u, current);
#endif
        } else {
            io_handler.write_pwm_motor(0.0f);
            pidCntrl.reset();
            lowPass2CurrentSetpoint.reset(0.0f);
        }

        io_handler.set_enable_frtt_do(false);
    }
}

bool fast_realtime_thread::start_loop(void)
{
    if (thread.start(callback(this, &fast_realtime_thread::loop)) != osOK) return false;
    ticker.attach(callback(this, &fast_realtime_thread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
    return true;
}

float fast_realtime_thread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

void fast_realtime_thread::sendSignal()
{
    thread.flags_set(threadFlag);
}
