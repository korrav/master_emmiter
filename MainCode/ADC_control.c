#include "emmiter.h"
#include "allocator.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "ADC_control.h"

/*NOTE: CK_INT TIM3 = 84 MHz*/
#define ADC_DR_ADDRESS ((uint32_t)0x4001204C)
#define NUMBER_SAMPL_ADC 1400 //number of samples adc buffer

static struct b_pool* pb_adc;

static void init_header_buffer_adc(struct b_pool* pbuf, int size, struct head_data_adc* phead);

void ADC_config(void) {
	DMA_InitTypeDef DMA_InitStructure_ADC;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitSt_C, GPIO_InitSt_D;
	ADC_CommonInitTypeDef  ADC_struct;
	ADC_InitTypeDef ADC_InitStructure;	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//CONFIGURE DMA FOR ADC
	DMA_InitStructure_ADC.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure_ADC.DMA_PeripheralBaseAddr = ADC_DR_ADDRESS;
	DMA_InitStructure_ADC.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure_ADC.DMA_BufferSize = NUMBER_SAMPL_ADC;
	DMA_InitStructure_ADC.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_ADC.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_ADC.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure_ADC.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure_ADC.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_ADC.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure_ADC.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure_ADC.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure_ADC.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure_ADC);
	//enable DMA interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);
	//CONFIGURE GPIO FOR ADC; 
	/*ADC Channel 10 -> PC0*/
  GPIO_InitSt_C.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitSt_C.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitSt_C.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitSt_C);
	//ADC COMMON INIT
	ADC_struct.ADC_Mode = ADC_Mode_Independent ;
	ADC_struct.ADC_Prescaler = ADC_Prescaler_Div8; //700kHz
	ADC_struct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_struct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_struct);
  //ADC CHANNEL INIT
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
	ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);
	// ENABLE ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);
	// ENABLE ADC1
  ADC_Cmd(ADC1, ENABLE);
	/*configure TIM3*/
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 0;        
  TIM_TimeBaseStructure.TIM_Prescaler = 0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	/* Input Trigger selection */
	TIM_ETRConfig(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
  TIM_SelectInputTrigger(TIM3, TIM_TS_ETRF);
  /* Slave Mode selection: Trigger Mode */
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Trigger);
	/*Trigger ADC -> PD2*/
	GPIO_InitSt_D.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitSt_D.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitSt_D.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitSt_D);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
}

void init_header_buffer_adc(struct b_pool* pbuf, int size, struct head_data_adc* phead) {
	struct head *h;
	struct head_data_adc* ph;
	h = (struct head*)pb_adc->pbuf;
	h->count = LAST;
	h->dst = BAG;
	h->src = MASTER;
	h->type = DATA;
	h->size = size;
	ph = (struct head_data_adc*)((char*)pb_adc->pbuf + sizeof(struct head));
	ph->hash = phead->hash;
	ph->nsec = phead->nsec;
	ph->sec = phead->sec;
	return;
}

void interDMA2_Stream0(void)
{
	TIM_Cmd(TIM3, DISABLE);
	ADC_Cmd(ADC1, DISABLE);
	DMA_Cmd(DMA2_Stream0, DISABLE);
	while(DMA2_Stream0->CR & DMA_SxCR_EN);
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0|DMA_FLAG_HTIF0|DMA_FLAG_TEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_FEIF0);
	enqueue_buf(pb_adc);
	update_current();
	set_cur_status_meas(STOP);
}

void set_task_adc(struct signal* psig, struct head_data_adc* phadc) {
	if(pb_adc == NULL || !check_buffer_is_free(pb_adc)) 
		pb_adc = alloc_buf(SIZE_BIG_BUFFER);
	init_header_buffer_adc(pb_adc, sizeof(struct head_data_adc) + SIZE_BIG_BUFFER * sizeof(uint16_t), phadc);
	DMA_SetCurrDataCounter(DMA2_Stream0, NUMBER_SAMPL_ADC);
	DMA2_Stream0->M0AR = (uint32_t)((char*) pb_adc->pbuf + sizeof(struct head) + sizeof(struct head_data_adc));
	TIM3->ARR = psig->arr_adc;
	TIM_PrescalerConfig(TIM3, psig->psc_adc, TIM_PSCReloadMode_Update);
	/*Refill register TIM1CCR1*/
	TIM3->EGR |= 0x0001;
	DMA_Cmd(DMA2_Stream0, ENABLE);
	ADC_Cmd(ADC1, ENABLE);	
  ADC_DMACmd(ADC1, DISABLE);
	ADC_DMACmd(ADC1, ENABLE);
}
