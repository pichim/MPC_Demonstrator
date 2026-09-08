#pragma once

#include <cstdint>

#include "Encoder.h"
#include "FastPWM.h"
#include "mbed.h"

//  1 Encoder Motor A            PA_6  - ok
//  2 Encoder Motor B            PC_7  - ok
//  3 Encoder Pendel A           PB_6  - ok
//  4 Encoder Pendel B           PB_7  - ok
//  5 PWM Motor                  PB_15 - ok
//  6 DIR Motor                  PB_14 - ok
//  7 Enable Motor               PB_9  - ok
//  8 Strommessung Motor         PA_7  - ok
//  9 Fehler Motor (Statusflag)  PB_8  - untested, TODO: Make use of this flag
// 10 Real-time thread DO        PB_4  - ok
// 10 Fast real-time thread DO   PB_5  - ok

#define ENCODER_MOTOR_COUNTS_PER_TURN (4 * 4096)    // 4096 PPR encoder with x4 decoding
#define ENCODER_PENDULUM_COUNTS_PER_TURN (4 * 1024) // 1024 PPR encoder with x4 decoding
#define MOTOR_PWM_PERIOD_US 50                      // 20 kHz PWM

class IO_handler
{
public:
    IO_handler();
    virtual ~IO_handler();
    float read_encoder_motor() { return encoder_motor.getAngleRad(); }
    float read_encoder_pendulum() { return encoder_pendulum.getAngleRad(-1.0f); }
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
    float read_current() { return (current.read() * 3.3f - 1.5f) * -1.0f / (50.0f * 7.0e-3f); }
    void set_dir(uint8_t d) { dir = d; }
    void set_enable_rtt_do(bool en) { rtt_do = en ? 1 : 0; }
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
    DigitalOut rtt_do;        // Real-time thread debug output
    DigitalOut frtt_do;       // Fast real-time thread debug output
};
