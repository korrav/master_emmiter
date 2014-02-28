#include "time.h"
#include "stm32f4xx_rcc.h"
__IO static uint64_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */

//delay function
void Delay(uint64_t nCount) {
  /* Capture the current local time */
  uint64_t timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime) {}
}

//update function of time
void Time_Update() {
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

uint64_t gettime() {
	return LocalTime;
}

void start_timer() {
	RCC_ClocksTypeDef RCC_Clocks;
	/* Configure Systick clock source as HCLK */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  /* SystTick configuration: an interrupt every 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
}
