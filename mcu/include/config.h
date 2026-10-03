#ifndef MPC_CONFIG_H_
#define MPC_CONFIG_H_

#include "mbed.h"

// SPI2 slave communication with Raspberry Pi SPI0 (mode 0)
#define MPC_SPI_MOSI_PIN PC_3
#define MPC_SPI_MISO_PIN PC_2
#define MPC_SPI_SCK_PIN PB_10
#define MPC_SPI_NSS_PIN PB_12
#define MPC_SPI_TRANSACTION_TIMEOUT_US 20000 // Abort NSS held low beyond 20 ms

// Periodic motor task and SPI recovery worker (Mbed RTOS priorities)
#define MPC_FAST_RT_PERIOD_US 50
#define MPC_FAST_RT_PRIORITY osPriorityHigh2
#define MPC_FAST_RT_STACK_SIZE (4 * OS_STACK_SIZE)
#define MPC_SPI_PRIORITY osPriorityHigh1
#define MPC_SPI_STACK_SIZE OS_STACK_SIZE
#define MPC_COMMAND_TIMEOUT_US 300000

// Motor, sensors, and timing outputs
#define MPC_MOTOR_ENCODER_A_PIN PA_6
#define MPC_MOTOR_ENCODER_B_PIN PC_7
#define MPC_MOTOR_PWM_PIN PB_15
#define MPC_MOTOR_DIR_PIN PB_14
#define MPC_MOTOR_ENABLE_PIN PB_9
#define MPC_CURRENT_PIN PA_7
#define MPC_MOTOR_FAULT_PIN PB_8 // Reserved input; not currently checked
#define MPC_FAST_RT_DEBUG_PIN PB_5

#define MPC_ADC_REFERENCE_V 3.3f
#define MPC_CURRENT_ZERO_V 1.5f
#define MPC_CURRENT_AMPLIFIER_GAIN 50.0f
#define MPC_CURRENT_SHUNT_OHM 7.0e-3f
#define MPC_CURRENT_SIGN -1.0f

#define MPC_PERFORM_GPA_MEAS false

#define MPC_ENCODER_MOTOR_COUNTS_PER_TURN (4 * 4096) // 4096 PPR encoder with x4 decoding
#define MPC_MOTOR_PWM_PERIOD_US 50                   // 20 kHz PWM

#define MPC_POWERSUPPLY_VOLTAGE 24.0f // Voltage of the power supply in Volts
#define MPC_OFFSET_VOLTAGE 2.0f       // Optional compensation in both modes; previously 2 V.

#define MPC_KP_I 2.5f                  // Proportional gain current controller
#define MPC_TN_I (0.0013f / 4.5320f)   // Integral time constant current controller (L / R)
#define MPC_KI_I (MPC_KP_I / MPC_TN_I) // Integral gain current controller
// Time constant of the current controller's first-order rolloff filter.
#define MPC_TAU_RO_I (1.f / (2.f * 3.14159265358979323846f * 3.0e3f))

#define MPC_POSITION_NOTCH_ENABLED true // Velocity notch remains enabled.
#define MPC_VELOCITY_F_CUT_HZ 100.0f    // First-order low-pass of encoder count velocity.

#define MPC_F_CUT_HZ_NOTCH 680.0f // Notch filter cutoff frequency in Hz
#define MPC_D_NOTCH 0.6f          // Notch filter damping

#define MPC_CURRENT_SETPOINT_F_CUT_HZ 500.0f // Second order low-pass filter cutoff frequency in Hz
#define MPC_CURRENT_SETPOINT_DAMPING 0.9f    // Second order low-pass filter damping ratio

#endif /* MPC_CONFIG_H_ */
