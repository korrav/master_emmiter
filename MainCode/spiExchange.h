#ifndef __SPIEXCHANGE
#define __SPIEXCHANGE
#include "stm32f4xx.h"
void SPI_Config(void);
void interruptDmaSpi(void);
void spi_write(uint8_t* pBuffer, unsigned int size);
#endif /* __SPIEXCHANGE */
