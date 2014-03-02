/**
  ******************************************************************************
  * @file    ADC/DualADC_RegulSimu_DMAmode1/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4x7_eth.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "pwm.h"
#include "emmiter.h"
#include "netconf.h"
#include "spiExchange.h"
#include "i2cExchange.h"
#include "ADC_control.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_DualADC_RegulSimu_DMAmode1
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Global variables-----------------------------------------------------------*/
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	Time_Update();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles ETH interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  	/* Handles all the received frames */
  	while(ETH_GetRxPktSize(DMARxDescToGet) != 0) 
  	{		
	  LwIP_Pkt_Handle();
  	}

  	/* Clear the Eth DMA Rx IT pending bits */
  	ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
  	ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}

void DMA1_Stream5_IRQHandler(void) {
	if(DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == SET) {
		interruptDmaSpi();
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
	}
	return;
}

void DMA1_Stream2_IRQHandler(void) {
	if(DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2) == SET) {
		interruptDmaI2c_Rx();
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
	}
	return;
}

void DMA1_Stream4_IRQHandler(void) {
	if(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == SET) {
		interruptDmaI2c_Tx();
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	}
	return;
}

void I2C3_EV_IRQHandler(void) {
	i2c_hand();
}

void DMA2_Stream0_IRQHandler(void) {
	interDMA2_Stream0();
	DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
}

void DMA2_Stream2_IRQHandler(void) {
	inter_DMA2_Stream2();
	DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
}

void TIM1_UP_TIM10_IRQHandler(void) {
	if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) == SET) {
		inter_TIM1_UP_TIM10();
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	}else if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Trigger) == SET) {
		inter_TIM1_start();
		TIM_ClearFlag(TIM1, TIM_FLAG_Trigger);
	}
	return;
}
