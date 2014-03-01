#include "i2cExchange.h"
#include "emmiter.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "allocator.h"
#include <string.h>

#define I2C_SLAVE 2
#define I2C_OWN 1
#define I2C I2C3
#define I2C3_ADDR 0x40005C10

static struct b_pool* pbr_i2c;
static struct b_pool* pbt_i2c;

void I2C_Config(void) {
	GPIO_InitTypeDef structGPIO;
	I2C_InitTypeDef structI2c;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef structDMA;
	pbr_i2c = alloc_buf(SIZE_LITTLE_BUFFER);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	structGPIO.GPIO_Pin = GPIO_Pin_8; //i2c3_scl
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_OD;
	structGPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &structGPIO);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
	structGPIO.GPIO_Pin = GPIO_Pin_9; //i2c3_sda
	GPIO_Init(GPIOC, &structGPIO);	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);
	
	structI2c.I2C_Mode = I2C_Mode_I2C;
  structI2c.I2C_DutyCycle = I2C_DutyCycle_2;
  structI2c.I2C_OwnAddress1 = I2C_OWN;
  structI2c.I2C_Ack = I2C_Ack_Enable;
  structI2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  structI2c.I2C_ClockSpeed = 400000;
	I2C_Init(I2C3, &structI2c);
	I2C_ITConfig(I2C3, I2C_IT_EVT, ENABLE);
	
	I2C_DMACmd(I2C3, ENABLE);
	//DMA for receive
	structDMA.DMA_Channel = DMA_Channel_3;
	structDMA.DMA_PeripheralBaseAddr = I2C3_ADDR;
	structDMA.DMA_Memory0BaseAddr = (uint32_t)pbr_i2c->pbuf;
	structDMA.DMA_DIR = DMA_DIR_PeripheralToMemory;
	structDMA.DMA_BufferSize = SIZE_LITTLE_BUFFER;
	structDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	structDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	structDMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	structDMA.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	structDMA.DMA_Mode = DMA_Mode_Normal;
	structDMA.DMA_Priority = DMA_Priority_Low;
	structDMA.DMA_FIFOMode = DMA_FIFOMode_Disable;
	structDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2, &structDMA);
	//DMA for transmit
	structDMA.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(DMA1_Stream4, &structDMA);
	//init I2C3 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = I2C3_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//init DMA interrupt for receive
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//init DMA interrupt for transmit
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//enable DMA
	DMA_Cmd(DMA1_Stream2, ENABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	//enable i2c3
	I2C_Cmd(I2C3, ENABLE);
	return;
}

void i2c_write(uint8_t* pBuffer, unsigned int size) {
	if( size > SIZE_LITTLE_BUFFER)
		return;
	pbt_i2c = alloc_buf(SIZE_LITTLE_BUFFER);
	memcpy(pbt_i2c->pbuf, pBuffer, size);
	DMA_MemoryTargetConfig(DMA1_Stream4, (uint32_t)pbt_i2c->pbuf, 0);
	while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY) != RESET);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	I2C_GenerateSTART(I2C, ENABLE);
}

void i2c_hand(void) {
	if (I2C_GetITStatus(I2C, I2C_IT_ADDR) == SET)
		I2C_GetFlagStatus(I2C, I2C_FLAG_TRA);
	if (I2C_GetITStatus(I2C, I2C_IT_STOPF) == SET) {
		I2C_GenerateSTOP(I2C, ENABLE);
	}
	return;
}

void interruptDmaI2c_Rx(void) {
	enqueue_buf(pbr_i2c);
	pbr_i2c = alloc_buf(SIZE_LITTLE_BUFFER);
	DMA_SetCurrDataCounter(DMA1_Stream2, SIZE_LITTLE_BUFFER);
	DMA_MemoryTargetConfig(DMA1_Stream2, (uint32_t)pbr_i2c->pbuf, 0);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	return;
}

void interruptDmaI2c_Tx(void) {
	free_buf(pbt_i2c);
	while(I2C_GetFlagStatus(I2C, I2C_FLAG_BTF) == RESET);
	I2C_GenerateSTOP(I2C, ENABLE);
	DMA_SetCurrDataCounter(DMA1_Stream4, SIZE_LITTLE_BUFFER);
	return;
}

/*void i2c_read(uint8_t command, uint8_t* pBuffer, uint32_t num) {
	uint32_t i = 0;
	I2C_ITConfig(I2C, I2C_IT_BUF | I2C_IT_EVT, DISABLE);
	while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY) != RESET);
	I2C_GenerateSTART(I2C, ENABLE);
	while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C, I2C_SLAVE, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C, command);
	while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_GenerateSTART(I2C, ENABLE);
	while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C, I2C_SLAVE, I2C_Direction_Receiver);
	if(num < 2) {
		while(I2C_GetFlagStatus(I2C, I2C_FLAG_ADDR) == RESET);
		I2C_AcknowledgeConfig(I2C, DISABLE);
		(void)I2C->SR2;
		I2C_GenerateSTOP(I2C, ENABLE);
		while(I2C_GetFlagStatus(I2C, I2C_FLAG_RXNE) == RESET);
		*pBuffer = I2C_ReceiveData(I2C);
		while(I2C->CR1 & I2C_CR1_STOP);
		I2C_AcknowledgeConfig(I2C, ENABLE);
	}	else {
		while(!I2C_CheckEvent(I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		for(i = num; i < num; i++) {
			if(i == (num - 2)) {
				while(I2C_GetFlagStatus(I2C, I2C_FLAG_RXNE) == RESET);
				I2C_AcknowledgeConfig(I2C, DISABLE);
				pBuffer[i] = I2C_ReceiveData(I2C);
				I2C_GenerateSTOP(I2C, ENABLE);
				while(I2C_GetFlagStatus(I2C, I2C_FLAG_RXNE) == RESET);
				pBuffer[i+1] = I2C_ReceiveData(I2C);
				while(I2C->CR1 & I2C_CR1_STOP);
				I2C_AcknowledgeConfig(I2C, ENABLE);
			} else {
				while(I2C_GetFlagStatus(I2C, I2C_FLAG_RXNE) == RESET);
				pBuffer[i] = I2C_ReceiveData(I2C);
			}
		}
	}
	I2C_ITConfig(I2C, I2C_IT_BUF | I2C_IT_EVT, ENABLE);
	return;
}*/
