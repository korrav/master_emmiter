#include "signal.h"
#define NUM_SAMPL_SIG1 3
#define NUM_SAMPL_SIG2 3
#define NUM_SAMPL_SIG3 3
#define ARR1 32768
#define ARR2 32768
#define ARR3 32768

static uint16_t sig1[NUM_SAMPL_SIG1] = { 1000, 2000, 3000};
static uint16_t sig2[NUM_SAMPL_SIG2] = { 1000, 2000, 3000};
static uint16_t sig3[NUM_SAMPL_SIG3] = { 1000, 2000, 3000};

struct signal pwm_sig[NUM_PWM_SIG] = {{ARR1, NUM_SAMPL_SIG1, sig1}, {ARR2, NUM_SAMPL_SIG2, sig2}, {ARR3, NUM_SAMPL_SIG3, sig3}};
