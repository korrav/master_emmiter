#ifndef __I2CEXCHANGE
#define __I2CEXCHANGE
#include "stm32f4xx.h"
void I2C_Config(void);
void i2c_write(uint8_t* pBuffer, unsigned int size);
void interruptDmaI2c_Rx(void);
void interruptDmaI2c_Tx(void);
void i2c_hand(void);
#endif /* __I2CEXCHANGE */
