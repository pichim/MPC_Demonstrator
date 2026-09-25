#pragma once

#include <cmath>
#include <cstdint>

// Owned by the current task. Only a newly received command calls accept().
class CurrentCommand {
public:
    void accept(const float values[3], uint32_t received_at_us)
    {
        const bool valid = std::isfinite(values[0]) &&
                           (values[1] == 0.0f || values[1] == 1.0f) &&
                           values[2] == 0.0f;
        enabled = valid && values[1] == 1.0f;
        current = enabled ? values[0] : 0.0f;
        if (valid) m_received_at_us = received_at_us;
    }

    void expire(uint32_t now_us, uint32_t timeout_us)
    {
        // Unsigned subtraction handles ticker wrap. Once expired, stay disabled
        // until a new command arrives; the current task checks every 50 us.
        if (enabled && uint32_t(now_us - m_received_at_us) >= timeout_us) {
            enabled = false;
            current = 0.0f;
        }
    }

    bool enabled{false};
    float current{0.0f};

private:
    uint32_t m_received_at_us{0};
};
