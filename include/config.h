#ifndef MPC_CONFIG_H_
#define MPC_CONFIG_H_

#include "mbed.h"

// UART communication with Raspberry Pi (NUCLEO-F446RE USART3).
#define MPC_UART_TX_PIN PC_10 // CN7 pin 1 -> Pi physical pin 10 (RX)
#define MPC_UART_RX_PIN PC_11 // CN7 pin 2 <- Pi physical pin 8 (TX)
#define MPC_UART_BAUD 230400   // Must match the host BAUD setting.

#endif /* MPC_CONFIG_H_ */
