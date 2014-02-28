#ifndef __TIME
#define __TIME
#include "stm32f4xx.h"
#define SYSTEMTICK_PERIOD_MS 10 //period of time update

void Delay(uint64_t nCount);
void Time_Update(void);
uint64_t gettime(void);
void start_timer(void);
#endif /* __TIME */
