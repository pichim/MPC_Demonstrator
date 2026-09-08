#include "fast_realtime_thread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

using MutexLock = rtos::ScopedMutexLock;

fast_realtime_thread::fast_realtime_thread(IO_handler &io, float Ts)
    : thread(osPriorityHigh2, (4 * OS_STACK_SIZE))
    , Ts(Ts)
    , io_handler(io)
{
    notchEncoders[0].notchInit(F_CUT_HZ_NOTCH, D_NOTCH, Ts);
    notchEncoders[1].notchInit(F_CUT_HZ_NOTCH, D_NOTCH, Ts);

    lowPass2CurrentSetpoint.lowPass2Init(F_CUT_HZ, D, Ts);

    pidCntrl.setup(KP_I, KI_I, 0.0f, 0.0f, TAU_RO_I, Ts, (-POWERSUPPLY_VOLTAGE + OFFSET_VOLTAGE), (POWERSUPPLY_VOLTAGE - OFFSET_VOLTAGE));

#if PERFORM_GPA_MEAS
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

void fast_realtime_thread::updateState(bool enable, float current_setpoint)
{
    MutexLock lock(m_mutex);
    m_is_enabled = enable;
    m_current_setpoint = current_setpoint;
}

void fast_realtime_thread::updateStateAndReturnMeasurements(bool enable, float current_setpoint, float &current, float &motor_angle, float &pendulum_angle)
{
    MutexLock lock(m_mutex);
    m_is_enabled = enable;
    m_current_setpoint = current_setpoint;
    current = m_current;
    motor_angle = m_motor_angle;
    pendulum_angle = m_pendulum_angle;
}

void fast_realtime_thread::loop(void)
{

#if PERFORM_GPA_MEAS
    io_handler.set_enable_motor(true);
    updateState(true, 0.0f);
    float exc = 0.0f;
    // Print gpa info
    m_GPA.printGPAmeasPara();
#endif

    while (true) {
        ThisThread::flags_wait_any(threadFlag);

        io_handler.set_enable_frtt_do(true);

        // Read current and filtered encoder values
        const float current = io_handler.read_current();
        const float motor_angle = notchEncoders[0].apply(io_handler.read_encoder_motor());
        const float pendulum_angle = notchEncoders[1].apply(io_handler.read_encoder_pendulum());

        bool is_enabled;
        float current_setpoint;
        {
            MutexLock lock(m_mutex);
            is_enabled = m_is_enabled;
            current_setpoint = m_current_setpoint;
            m_current = current;
            m_motor_angle = motor_angle;
            m_pendulum_angle = pendulum_angle;
        }

        // Current controller
        if (is_enabled) {

            // Error
            float current_error = lowPass2CurrentSetpoint.apply(current_setpoint) - current;
#if PERFORM_GPA_MEAS
            current_error += exc + 0.6f;
#endif
            // Controller
            const float u = pidCntrl.update(current_error, current);

            // Caluclate direction and PWM value
            if (u > 0.0f)
                io_handler.set_dir(0);
            else
                io_handler.set_dir(1);

            io_handler.write_pwm_motor(clamp((fabs(u) + OFFSET_VOLTAGE) / POWERSUPPLY_VOLTAGE, 0.0f, 1.0f));

#if PERFORM_GPA_MEAS
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

void fast_realtime_thread::start_loop(void)
{
    thread.start(callback(this, &fast_realtime_thread::loop));
    ticker.attach(callback(this, &fast_realtime_thread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
}

float fast_realtime_thread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
