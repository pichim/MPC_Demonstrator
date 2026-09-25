#include "IO_handler.h"

// constructors
IO_handler::IO_handler(void)
    : encoder_motor(MPC_MOTOR_ENCODER_A_PIN, MPC_MOTOR_ENCODER_B_PIN, MPC_ENCODER_MOTOR_COUNTS_PER_TURN)
    , encoder_pendulum(MPC_PENDULUM_ENCODER_A_PIN, MPC_PENDULUM_ENCODER_B_PIN, MPC_ENCODER_PENDULUM_COUNTS_PER_TURN)
    , pwm(MPC_MOTOR_PWM_PIN)
    , pwm_val(0.0f)
    , dir(MPC_MOTOR_DIR_PIN)
    , enable(MPC_MOTOR_ENABLE_PIN)
    , current(MPC_CURRENT_PIN)
    , fault(MPC_MOTOR_FAULT_PIN)
    , frtt_do(MPC_FAST_RT_DEBUG_PIN)
{
    pwm.write(0.0f); // Ensure PWM starts at zero.
    pwm.period_us(MPC_MOTOR_PWM_PERIOD_US);
    dir = 0;
    enable = 0;
}

IO_handler::~IO_handler() {}
