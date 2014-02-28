#include "emmiter.h"
#include "spiExchange.h"
#include "generate_message.h"
#include "allocator.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include <string.h>

#define SPI3_ADDR 0x40003C0C

static struct b_pool* pb_spi;

void SPI_Config(void) {
	GPIO_InitTypeDef structGPIO;
	SPI_InitTypeDef structSPI;
	DMA_InitTypeDef structDMA;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
	//configure GPIO
	structGPIO.GPIO_Pin = GPIO_Pin_4;
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_PP;
	structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &structGPIO);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	structGPIO.GPIO_Pin =GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &structGPIO);	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	//init spi 
	SPI_StructInit(&structSPI);
	structSPI.SPI_Direction = SPI_Direction_1Line_Tx;
	structSPI.SPI_Mode = SPI_Mode_Master;
	structSPI.SPI_DataSize = SPI_DataSize_16b;
	SPI_Init(SPI3, &structSPI);
	//init DMA
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	structDMA.DMA_Channel = DMA_Channel_0;
	structDMA.DMA_PeripheralBaseAddr = SPI3_ADDR;
	structDMA.DMA_Memory0BaseAddr = (uint32_t)pb_spi->pbuf;
	structDMA.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	structDMA.DMA_BufferSize = SIZE_BIG_BUFFER/2;
	structDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	structDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	structDMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	structDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	structDMA.DMA_Mode = DMA_Mode_Normal;
	structDMA.DMA_Priority = DMA_Priority_Medium;
	structDMA.DMA_FIFOMode = DMA_FIFOMode_Disable;
	structDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &structDMA);
	//init DMA interrupt
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//enable DMA
	DMA_Cmd(DMA1_Stream5, DISABLE);
	//enable SPI
	SPI_Cmd(SPI3, ENABLE);
}

void interruptDmaSpi(void) {
	free_buf(pb_spi);
	DMA_SetCurrDataCounter(DMA1_Stream5, SIZE_BIG_BUFFER);
	return;
}

void spi_write(uint8_t* pBuffer, unsigned int size) {
	if (size > SIZE_BIG_BUFFER)
		return;
	pb_spi = alloc_buf(SIZE_BIG_BUFFER);
	memcpy(pb_spi->pbuf, pBuffer, size);
	DMA_MemoryTargetConfig(DMA1_Stream5, (uint32_t)pb_spi->pbuf, 0);
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

// void SPI_Config(void) {
// 	GPIO_InitTypeDef structGPIO;
// 	SPI_InitTypeDef structSPI;
// 	DMA_InitTypeDef structDMA;
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
// 		//configure GPIO
// // 	GPIOA->AFR[1] |= GPIO_AF_SPI3 << 8;
// // 	GPIOA->AFR[1] |= GPIO_AF_SPI3 << 12;
// // 	GPIOA->AFR[1] |= GPIO_AF_SPI3 << 16;
// 	structGPIO.GPIO_Pin =GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //spi3
//   structGPIO.GPIO_Mode = GPIO_Mode_AF;
// 	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
// 	structGPIO.GPIO_OType = GPIO_OType_PP;
// 	structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_Init(GPIOC, &structGPIO);	
// 	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
//  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
//  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
// 	
// 	/*structGPIO.GPIO_Pin =GPIO_Pin_4;
// 	structGPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
//  	GPIO_Init(GPIOA, &structGPIO);	
// 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);*/
// 	//init spi 
// 	SPI_StructInit(&structSPI);
// 	structSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
// 	structSPI.SPI_Mode = SPI_Mode_Slave;
// 	structSPI.SPI_DataSize = SPI_DataSize_16b;
// 	SPI_Init(SPI3, &structSPI);
// 	//init DMA
// 	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
// 	structDMA.DMA_Channel = DMA_Channel_0;
// 	structDMA.DMA_PeripheralBaseAddr = SPI3_ADDR;
// 	structDMA.DMA_Memory0BaseAddr = (uint32_t)getAnalogBuf(0);
// 	structDMA.DMA_DIR = DMA_DIR_PeripheralToMemory;
// 	structDMA.DMA_BufferSize = SIZE_BUF * 2;
// 	structDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	structDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	structDMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
// 	structDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
// 	structDMA.DMA_Mode = DMA_Mode_Circular;
// 	structDMA.DMA_Priority = DMA_Priority_VeryHigh;
// 	structDMA.DMA_FIFOMode = DMA_FIFOMode_Enable;
// 	structDMA.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
// 	structDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
// 	DMA_Init(DMA1_Stream0, &structDMA);	
// 	//init DMA interrupt
// 	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
// 	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
// 	//enable DMA
// 	DMA_Cmd(DMA1_Stream0, ENABLE);
// 	//initialize the analog part
// 	DMA_SetCurrDataCounter(DMA1_Stream0, SIZE_BUF);
// 	//enable SPI
// 	SPI_Cmd(SPI3, ENABLE);
// }
