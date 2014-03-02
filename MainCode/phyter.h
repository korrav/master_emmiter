#ifndef __PHYTER
#define __PHYTER
#include "stm32f4xx.h"
#include "ptpd.h"
#define REL_TARGET_TIME 20        //sec
void set_current_Time(TimeInternal* t); //set the current time
TimeInternal set_task_phyter(TimeInternal* curTime);
void set_tarTime(uint32_t* ptime_s, uint32_t* ptime_ns);
void set_subtOnAmp(uint32_t* t);
void set_w_subtOnAmp(uint32_t* t);
void set_addtOfHyd(uint32_t* t);
void set_w_addtOfHyd(uint32_t* t);
void set_addtADC1(uint32_t* t);
#endif /* __PHYTER */
