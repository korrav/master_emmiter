#include "signal.h"
#define NUM_SAMPL_SIG1 3
#define NUM_SAMPL_SIG2 3
#define NUM_SAMPL_SIG3 3
#define ARR1_PWM 32768
#define ARR2_PWM 32768
#define ARR3_PWM 32768
#define PSC1_PWM 0
#define PSC2_PWM 0
#define PSC3_PWM 0
#define ARR1_ADC 32768
#define ARR2_ADC 32768
#define ARR3_ADC 32768
#define PSC1_ADC 3
#define PSC2_ADC 3
#define PSC3_ADC 3

static uint16_t sig1[NUM_SAMPL_SIG1] = { 1000, 2000, 3000};
static uint16_t sig2[NUM_SAMPL_SIG2] = { 1000, 2000, 3000};
static uint16_t sig3[NUM_SAMPL_SIG3] = { 1000, 2000, 3000};

struct signal pwm_sig[NUM_PWM_SIG] = {{ARR1_PWM, PSC1_PWM, ARR1_ADC, PSC1_ADC, NUM_SAMPL_SIG1, sig1}, 
																			{ARR2_PWM, PSC2_PWM, ARR2_ADC, PSC2_ADC, NUM_SAMPL_SIG2, sig2}, 
																			{ARR3_PWM, PSC3_PWM, ARR3_ADC, PSC3_ADC, NUM_SAMPL_SIG3, sig3}};
