#pragma once

#include <cmath>
#include <cstdint>

// Owned by the motor task. Only a newly received command calls accept().
class MotorCommand
{
public:
    void accept(const float values[4], uint32_t received_at_us)
    {
        const bool valid = std::isfinite(values[0]) && (values[1] == 0.0f || values[1] == 1.0f) &&
                           (values[2] == 0.0f || values[2] == 1.0f) && values[3] == 0.0f;
        enabled = valid && values[1] == 1.0f;
        setpoint = enabled ? values[0] : 0.0f;
        if (valid)
            mode = values[2] == 0.0f ? Mode::Current : Mode::Voltage;
        if (valid)
            m_received_at_us = received_at_us;
    }

    void expire(uint32_t now_us, uint32_t timeout_us)
    {
        // Unsigned subtraction handles ticker wrap. Once expired, stay disabled
        // until a new command arrives; the motor task checks every 50 us.
        if (enabled && uint32_t(now_us - m_received_at_us) >= timeout_us) {
            enabled = false;
            setpoint = 0.0f;
        }
    }

    bool enabled{false};
    float setpoint{0.0f};
    enum class Mode {
        Current = 0,
        Voltage = 1
    };
    Mode mode{Mode::Current};

private:
    uint32_t m_received_at_us{0};
};
