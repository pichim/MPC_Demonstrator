#ifndef MPC_CONFIG_H_
#define MPC_CONFIG_H_

#include "mbed.h"

// SPI2 slave communication with Raspberry Pi SPI0 (mode 0)
#define MPC_SPI_MOSI_PIN PC_3
#define MPC_SPI_MISO_PIN PC_2
#define MPC_SPI_SCK_PIN PB_10
#define MPC_SPI_NSS_PIN PB_12

#endif /* MPC_CONFIG_H_ */
