#include "emmiter.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "signal.h"
#include "pwm.h"

#define TIM1_CCR2_ADDRESS    0x40010038

void pwm_Config() {
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitSt_AB;
	DMA_InitTypeDef DMA_InitStructure_TIM1;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
	//PWM INITIALIZATION
	/* GPIOA and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	/* GPIOA Configuration: Channel 1 as alternate function push-pull */
  GPIO_InitSt_AB.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitSt_AB.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitSt_AB.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitSt_AB.GPIO_OType = GPIO_OType_PP;
  GPIO_InitSt_AB.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitSt_AB); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	/* GPIOB Configuration: Channel 1N as alternate function push-pull */
  GPIO_InitSt_AB.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitSt_AB);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
	/* GPIOC Configuration: Reset*/
	/*GPIO_InitSt_C.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitSt_C.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitSt_C.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitSt_C.GPIO_OType = GPIO_OType_PP;
  GPIO_InitSt_C.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitSt_C); */
	//CONFIGURE DMA FOR TIM1
	DMA_DeInit(DMA2_Stream2);
  DMA_InitStructure_TIM1.DMA_Channel = DMA_Channel_6;  
  DMA_InitStructure_TIM1.DMA_PeripheralBaseAddr = (uint32_t)(TIM1_CCR2_ADDRESS) ;
  DMA_InitStructure_TIM1.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure_TIM1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_TIM1.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_TIM1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure_TIM1.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure_TIM1.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure_TIM1.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure_TIM1.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure_TIM1.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure_TIM1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure_TIM1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure_TIM1);
	//enable DMA interrupt
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//enable TIM1_UIF interrupt
	TIM_ITConfig(TIM1, TIM_IT_Trigger, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_DeInit(TIM1);
	/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	/* Channel 2 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  //TIM_OCInitStructure.TIM_Pulse = pwm_flash[0];
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Input Trigger selection */
	TIM_ETRConfig(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
  TIM_SelectInputTrigger(TIM1, TIM_TS_ETRF);
  /* Slave Mode selection: Trigger Mode */
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);
	/*Trigger PWM -> PA12*/
	GPIO_InitSt_AB.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitSt_AB.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitSt_AB.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitSt_AB);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_TIM1);
	/* DMA enable*/
  DMA_Cmd(DMA2_Stream2, ENABLE);  
  /* TIM1 Update DMA Request enable */
  TIM_DMACmd(TIM1, TIM_DMA_CC2, ENABLE);
	/* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
	/* in reset PWM amplifier*/
	//GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}


                                            /*****inter_DMA2_Stream2*****/
 void inter_DMA2_Stream2(void) {
   /* TIM1 Update DMA Request disable */
   TIM_DMACmd(TIM1, TIM_DMA_CC2, DISABLE);
	 /*enable UIE TIM1*/
	 TIM1->DIER |= TIM_DIER_UIE;
	 return;
 }
 
                                            /*****inter_TIM1_UP_TIM10*****/
 void inter_TIM1_UP_TIM10(void) {
	 	 /* TIM1 counter disable */
   TIM_Cmd(TIM1, DISABLE);
	 //set in Reset
	 //GPIO_ResetBits(GPIOC, GPIO_Pin_8);
	 /*disable UIE TIM1*/
	 TIM1->DIER &= ~TIM_DIER_UIE;
	 /*disable DMA*/
	 DMA_Cmd(DMA2_Stream2, DISABLE); 
	 while(DMA2_Stream2->CR & DMA_SxCR_EN);
	 DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2|DMA_FLAG_HTIF2|DMA_FLAG_TEIF2|DMA_FLAG_DMEIF2|DMA_FLAG_FEIF2);
	 /* DMA enable*/
   DMA_Cmd(DMA2_Stream2, ENABLE);  
   /* TIM1 Update DMA Request enable */
   TIM_DMACmd(TIM1, TIM_DMA_CC2, ENABLE);
	 return;
 }
 
void inter_TIM1_start(void) {
	set_cur_status_meas(RUN);
	return;
}
void set_task_pwm(struct signal* psig) {
	 DMA_SetCurrDataCounter(DMA2_Stream2, psig->num_sampl);
	 DMA2_Stream2->M0AR = (uint32_t)psig->pbuf;
	 TIM1->ARR = psig->arr_pwm;
	 TIM_PrescalerConfig(TIM1, psig->psc_pwm, TIM_PSCReloadMode_Update);
	 /*Refill register TIM1CCR1*/
	 TIM1->EGR |= 0x0001;
	 return;
 }
