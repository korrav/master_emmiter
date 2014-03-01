#include "phyter.h"
#include "ptpd.h"
#include "emmiter.h"

//timings
#define REL_TARGET_TIME 40        //1 minute
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
