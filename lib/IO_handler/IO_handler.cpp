#include "IO_handler.h"

// constructors
IO_handler::IO_handler(void)
    : encoder_motor(PA_6, PC_7, ENCODER_MOTOR_COUNTS_PER_TURN)
    , encoder_pendulum(PB_6, PB_7, ENCODER_PENDULUM_COUNTS_PER_TURN)
    , pwm(PB_15)
    , pwm_val(0.0f)
    , dir(PB_14)
    , enable(PB_9)
    , current(PA_7)
    , fault(PB_8)
    , rtt_do(PB_4)
    , frtt_do(PB_5)
{
    pwm.write(0.0f); // enusure motor is off
    pwm.period_us(MOTOR_PWM_PERIOD_US);
    dir = 0;
    enable = 0;
}

IO_handler::~IO_handler() {}
