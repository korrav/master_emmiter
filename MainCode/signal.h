#ifndef __SIGNAL
#define __SIGNAL
#include "stm32f4xx.h"
#define NUM_PWM_SIG 3
//description of the signal
struct signal  {
	uint16_t arr;					//contents of the register arr
	uint16_t num_sampl;		//number of samples in the signal
	uint16_t* pbuf;				//a pointer to first sample signal
};

extern struct signal pwm_sig[NUM_PWM_SIG];
#endif /* __SIGNAL */
