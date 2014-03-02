#ifndef __PWM
#define __PWM
struct signal;
void pwm_Config(void);
void set_task_pwm(struct signal* psig); //set a task for pwm
void inter_TIM1_UP_TIM10(void);
void inter_TIM1_start(void);
void inter_DMA2_Stream2(void);
#endif /* __PWM */
