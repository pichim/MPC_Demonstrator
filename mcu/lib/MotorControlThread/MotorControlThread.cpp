#include "MotorControlThread.h"

#include <chrono>
#include <cmath>
#include <cstdint>

static_assert(MPC_POWERSUPPLY_VOLTAGE > 0.0f && MPC_OFFSET_VOLTAGE >= 0.0f &&
                  MPC_OFFSET_VOLTAGE < MPC_POWERSUPPLY_VOLTAGE,
              "Invalid voltage configuration");

MotorControlThread::MotorControlThread(IO_handler &io, SpiSlaveDMA &spi, float Ts)
    : thread(MPC_FAST_RT_PRIORITY, MPC_FAST_RT_STACK_SIZE)
    , Ts(Ts)
    , io_handler(io)
    , spi(spi)
{
    positionNotch.notchInit(MPC_F_CUT_HZ_NOTCH, MPC_D_NOTCH, Ts);
    velocityNotch.notchInit(MPC_F_CUT_HZ_NOTCH, MPC_D_NOTCH, Ts);

    lowPass2CurrentSetpoint.lowPass2Init(MPC_CURRENT_SETPOINT_F_CUT_HZ, MPC_CURRENT_SETPOINT_DAMPING, Ts);

    pidCntrl.setup(MPC_KP_I,
                   MPC_KI_I,
                   0.0f,
                   0.0f,
                   MPC_TAU_RO_I,
                   Ts,
                   (-MPC_POWERSUPPLY_VOLTAGE + MPC_OFFSET_VOLTAGE),
                   (MPC_POWERSUPPLY_VOLTAGE - MPC_OFFSET_VOLTAGE));

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

MotorControlThread::~MotorControlThread() {}

void MotorControlThread::loop(void)
{

#if MPC_PERFORM_GPA_MEAS
    float exc = 0.0f;
    // Print gpa info
    m_GPA.printGPAmeasPara();
#endif

    MotorCommand command;
    bool was_enabled = false;
    MotorCommand::Mode previous_mode = command.mode;
    float voltage = 0.0f; // Last applied voltage before optional compensation.
    const float voltage_limit = MPC_POWERSUPPLY_VOLTAGE - MPC_OFFSET_VOLTAGE;
    while (true) {
        ThisThread::flags_wait_any(threadFlag);
        io_handler.set_enable_frtt_do(true);

        const float current = io_handler.read_current();
        const auto encoder = io_handler.read_encoder_motor();
        const float position = MPC_POSITION_NOTCH_ENABLED ? positionNotch.apply(encoder.position) : encoder.position;
        const float velocity = velocityNotch.apply(encoder.velocity);
        // Sensors were sampled under the previously applied voltage. Publish a
        // coherent snapshot before computing the next output, not an ACK.
        spi.setReplyData(voltage, current, position, velocity);

        SpiCommand received;
        if (spi.takeCommand(received))
            command.accept(received.data, received.received_at_us);
        command.expire(us_ticker_read(), MPC_COMMAND_TIMEOUT_US);
        const bool entering_current = command.enabled && command.mode == MotorCommand::Mode::Current &&
                                      (!was_enabled || previous_mode != command.mode);
        if (entering_current) {
            lowPass2CurrentSetpoint.reset(current);
            pidCntrl.trackOutput(voltage, 0.0f, current);
#if MPC_PERFORM_GPA_MEAS
            exc = 0.0f;
#endif
        }
        if (command.enabled) {
            if (command.mode == MotorCommand::Mode::Current) {
                // Hold voltage for the handover tick; evolve control next tick.
                if (!entering_current) {
                    float current_setpoint = lowPass2CurrentSetpoint.apply(command.setpoint);
#if MPC_PERFORM_GPA_MEAS
                    current_setpoint += exc + 0.6f;
#endif
                    voltage = pidCntrl.update(current_setpoint - current, current);
#if MPC_PERFORM_GPA_MEAS
                    exc = m_GPA.update(voltage, current);
#endif
                }
            } else {
                // Direct voltage: no current regulation or setpoint smoothing.
                voltage = command.setpoint;
            }
        }
        // Disable/expiry and numeric failure share one output/reset path.
        // Finite commands can still overflow controller/filter arithmetic.
        if (!command.enabled || !std::isfinite(voltage)) {
            command.enabled = false;
            command.setpoint = 0.0f;
            io_handler.set_enable_motor(false);
            io_handler.write_pwm_motor(0.0f);
            voltage = 0.0f;
            pidCntrl.reset();
            lowPass2CurrentSetpoint.reset(0.0f);
#if MPC_PERFORM_GPA_MEAS
            exc = 0.0f;
#endif
        } else {
            voltage = clamp(voltage, -voltage_limit, voltage_limit);
            io_handler.set_dir(voltage < 0.0f ? 1 : 0);
            // Zero voltage must remain zero even when compensation is enabled.
            const float duty = voltage == 0.0f ? 0.0f : (fabsf(voltage) + MPC_OFFSET_VOLTAGE) / MPC_POWERSUPPLY_VOLTAGE;
            io_handler.write_pwm_motor(clamp(duty, 0.0f, 1.0f));
            io_handler.set_enable_motor(true);
        }
        was_enabled = command.enabled;
        previous_mode = command.mode;

        io_handler.set_enable_frtt_do(false);
    }
}

bool MotorControlThread::start_loop(void)
{
    if (thread.start(callback(this, &MotorControlThread::loop)) != osOK)
        return false;
    ticker.attach(callback(this, &MotorControlThread::sendSignal), microseconds{static_cast<int64_t>(Ts * 1e6f)});
    return true;
}

float MotorControlThread::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

void MotorControlThread::sendSignal() { thread.flags_set(threadFlag); }
