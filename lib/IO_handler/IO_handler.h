#pragma once

#include <cstdint>

#include "Encoder.h"
#include "FastPWM.h"
#include "config.h"
#include "mbed.h"

class IO_handler
{
public:
    IO_handler();
    virtual ~IO_handler();
    float read_encoder_motor() { return encoder_motor.getAngleRad(); }
    float read_encoder_pendulum() { return encoder_pendulum.getAngleRad(MPC_PENDULUM_ENCODER_SIGN); }
    void reset_encoders()
    {
        encoder_motor.reset();
        encoder_pendulum.reset();
    }
    void write_pwm_motor(float val)
    {
        pwm_val = val;
        pwm.write(pwm_val);
    }
    float get_set_value() const { return pwm_val; }
    void set_enable_motor(bool en) { enable = en ? 1 : 0; }
    float read_current()
    {
        return (current.read() * MPC_ADC_REFERENCE_V - MPC_CURRENT_ZERO_V) * MPC_CURRENT_SIGN /
               (MPC_CURRENT_AMPLIFIER_GAIN * MPC_CURRENT_SHUNT_OHM);
    }
    void set_dir(uint8_t d) { dir = d; }
    void set_enable_frtt_do(bool en) { frtt_do = en ? 1 : 0; }

private:
    Encoder encoder_motor;    // Motor encoder
    Encoder encoder_pendulum; // Pendulum encoder
    FastPWM pwm;              // PWM for motor
    float pwm_val;            // stored PWM value
    DigitalOut dir;           // Direction for motor
    DigitalOut enable;        // Enable power h-bridge
    AnalogIn current;         // Current sensing
    DigitalIn fault;          // Fault status flag
    DigitalOut frtt_do;       // Fast real-time thread debug output
};
