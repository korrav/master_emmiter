#ifndef __ADC_CONTROL
#define __ADC_CONTROL
#include "stm32f4xx.h"
#include "signal.h"

struct head_data_adc {
	int sec;
	int nsec;
	int hash;
};

void interDMA2_Stream0(void);
void ADC_config(void);
void set_task_adc(struct signal* psig, struct head_data_adc* phadc);
#endif /* __ADC_CONTROL */
