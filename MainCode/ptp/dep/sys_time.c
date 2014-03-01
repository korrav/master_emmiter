/* sys.c */

#include "../ptpd.h"
#include "emmiter.h"

void getTime(TimeInternal *time)
{

    struct ptptime_t timestamp;
    ETH_PTPTime_GetTime(&timestamp);
    time->seconds = timestamp.tv_sec;
    time->nanoseconds = timestamp.tv_nsec;
}

void setTime(const TimeInternal *time)
{

    struct ptptime_t ts;
    ts.tv_sec = time->seconds;
    ts.tv_nsec = time->nanoseconds;
    ETH_PTPTime_SetTime(&ts);

}

TimeInternal tim_add, time;
void setTimeMeas(TimeInternal *t_DAC, uint32_t subtOnAmp, uint32_t w_subtOnAmp, uint32_t addtOfHyd, uint32_t w_addtOfHyd, uint32_t addtADC) {
	tim_add.seconds = 0;
	__disable_irq();
	PTPTriggerDisable(DP83848_PHY_ADDRESS);
	PTPArmTrigger(DP83848_PHY_ADDRESS, TRIG_PWM, (uint32_t)t_DAC->seconds, (uint32_t)t_DAC->nanoseconds, 2000000);
	tim_add.nanoseconds = subtOnAmp;
	subtime(t_DAC, &tim_add, &time);
	PTPArmTrigger(DP83848_PHY_ADDRESS, TRIG_ON_AMP, (uint32_t)time.seconds, (uint32_t)time.nanoseconds, w_subtOnAmp);
	tim_add.nanoseconds = addtOfHyd;
	sumtime(t_DAC, &tim_add, &time);
	PTPArmTrigger(DP83848_PHY_ADDRESS, TRIG_OF_HYD, (uint32_t)time.seconds, (uint32_t)time.nanoseconds, w_addtOfHyd);
	tim_add.nanoseconds = addtADC;
	sumtime(t_DAC, &tim_add, &time);
	PTPArmTrigger(DP83848_PHY_ADDRESS, TRIG_ADC, (uint32_t)time.seconds, (uint32_t)time.nanoseconds, 2000000);
	__enable_irq();
	return;
}

void updateTime(const TimeInternal *time)
{

    struct ptptime_t timeoffset;

    DBGV("updateTime: %ds %dns\n", time->seconds, time->nanoseconds);

    timeoffset.tv_sec = -time->seconds;
    timeoffset.tv_nsec = -time->nanoseconds;

	/* Coarse update method */
    ETH_PTPTime_UpdateOffset(&timeoffset);
    DBGV("updateTime: updated\n");
}

void sumtime(TimeInternal* t1, TimeInternal* t2, TimeInternal* ts) {
	ts->seconds = t1->seconds + t2->seconds + (t1->nanoseconds + t2->nanoseconds)/1000000000;
	ts->nanoseconds = (t1->nanoseconds + t2->nanoseconds)%1000000000;
	return;
}

void subtime(TimeInternal* t1, TimeInternal* t2, TimeInternal* ts) {
	ts->seconds = t1->seconds - t2->seconds;
	if(t1->nanoseconds > t2->nanoseconds)
		ts->nanoseconds = t1->nanoseconds - t2->nanoseconds;
	else {
		ts->nanoseconds = 1000000000 + t1->nanoseconds - t2->nanoseconds;
		ts->seconds --;
	}
	return;
}

UInteger32 getRand(UInteger32 randMax)
{
    return rand() % randMax;
}

Boolean adjFreq(Integer32 adj)
{
    DBGV("adjFreq %d\n", adj);

    if (adj > ADJ_FREQ_MAX)
        adj = ADJ_FREQ_MAX;
    else if (adj < -ADJ_FREQ_MAX)
        adj = -ADJ_FREQ_MAX;

    /* Fine update method */
	ETH_PTPTime_AdjFreq(adj);

    return TRUE;
}
