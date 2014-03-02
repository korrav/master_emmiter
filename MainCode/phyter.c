#include "phyter.h"
#include "emmiter.h"

//timings
#define TARGET_TIME     100000000   //100 ms
#define SUBTONAMP 	    100000      //100 us
#define W_SUBTONAMP     3200000			//3.2 ms
#define ADDOFHYD        2000000     //2 ms
#define W_ADDOFHYD      1000000     //1 ms
#define ADDTADC1        35600000    //35.6 ms

static struct {
	TimeInternal cur_tarTime;
	TimeInternal tarTime;
	uint32_t subtOnAmp; 
	uint32_t w_subtOnAmp; 
	uint32_t addtOfHyd; 
	uint32_t w_addtOfHyd; 
	uint32_t addtADC1;
} timings;

void Phyter_Config(void) {
	//fill timings
	timings.tarTime.seconds = 5;
	timings.tarTime.nanoseconds = TARGET_TIME;
	timings.subtOnAmp = SUBTONAMP;
	timings.w_subtOnAmp = W_SUBTONAMP;
	timings.addtOfHyd = ADDOFHYD;
	timings.w_addtOfHyd = W_ADDOFHYD;
	timings.addtADC1 = ADDTADC1;
}

void set_current_Time(TimeInternal* t) {
	__disable_irq ();
  PTPTriggerDisable(DP83848_PHY_ADDRESS);
	__enable_irq ();
	setTime(t);
}

TimeInternal set_task_phyter(TimeInternal* curTime) {
	if(curTime != NULL)
		sumtime(&timings.cur_tarTime, &timings.tarTime, &timings.cur_tarTime);
	else
		timings.cur_tarTime = *curTime;
	setTimeMeas(&timings.cur_tarTime, timings.subtOnAmp, timings.w_subtOnAmp, timings.addtOfHyd, timings.w_addtOfHyd, timings.addtADC1);
	return timings.cur_tarTime;
}
void set_tarTime(uint32_t* ptime_s, uint32_t* ptime_ns) {
	TimeInternal t;
	t.seconds = *ptime_s;
	t.nanoseconds = *ptime_ns;
	timings.tarTime = t;
	return;
}

void set_subtOnAmp(uint32_t* t) {
	timings.subtOnAmp = *t;
	return;
}

void set_w_subtOnAmp(uint32_t* t) {
	timings.w_subtOnAmp = *t;
	return;
}
void set_addtOfHyd(uint32_t* t) {
	timings.addtOfHyd = *t;
	return;
}
void set_w_addtOfHyd(uint32_t* t) {
	timings.w_addtOfHyd = *t;
	return;
}
void set_addtADC1(uint32_t* t) {
	timings.addtADC1 = *t;
	return;
}
