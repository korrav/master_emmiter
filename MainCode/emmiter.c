#include "emmiter.h"
#include "stm32f4x7_eth_bsp.h"
#include "udp_echoserver.h"
#include "allocator.h"
#include "generate_message.h"
#include "lwipopts.h"
#include "netconf.h"
#include "time.h"
#include <stdio.h>
#include "ptpd.h"
#include "i2cExchange.h"
#include "spiExchange.h"
#include "ADC_control.h"
#include "pwm.h"
#include "signal.h"
#include "ptpd.h"
#include "phyter.h"

/*#define PLL_M_HSE      25
#define PLL_N_HSE      336
#define PLL_P_HSE      2
#define PLL_Q_HSE      7*/

//debug struct
static struct {
	int32_t num;
	int32_t time;
} debug;

static uint64_t d_time;

//structure of the current state
static struct current_state {
	uint16_t id_sig; //number of the current pwm signal
	uint8_t status;
} cur;

struct timings_meas{
	int32_t cur_tarTime_sec;
	int32_t cur_tarTime_nsec;
	int32_t restart;
	int32_t tarTime_sec;
	int32_t tarTime_nsec;
	int32_t subtOnAmp; 
	int32_t w_subtOnAmp; 
	int32_t addtOfHyd; 
	int32_t w_addtOfHyd; 
	int32_t addtADC1;
};

static void NVIC_Configuration(void);
static void handl_command(struct b_pool* pbuf);
static void enable_Debug(int32_t num, int32_t time);
static void disable_Debug(void);
static void get_enable_Debug(int32_t *num, int32_t *time);
static void debug_info(void);
static void init_current_state(struct current_state* c, TimeInternal* t);
static void stop_meas(void); //stop measure
static void start_meas(int32_t *time); //start measure
static struct b_pool* package_answer_to_bag(int** pbuf, int size); //packing response sent to BAG
//static void clockHseInit(void);
static void transPackage(struct b_pool* pbuf, void (*pfunc)(uint8_t* pBuffer, unsigned int size));

int main(void) {
	struct b_pool* pbuf;
	struct head* h;
	init_pools_buffers();
	//configure NVIC
	NVIC_Configuration();
	//start time
	start_timer();
	//configure spi
	SPI_Config();
	//configure i2c
	I2C_Config();
	//configure adc
	ADC_config();
	//configure pwm
	pwm_Config();
	//ETHERNET INITIALIZATION
	ETH_BSP_Config();
	/* Initilaize the LwIP stack */
  LwIP_Init();
	//enable clock
	/*__disable_irq();
	clockHseInit();
	__enable_irq();*/
	/*Initilaize the PTP stack*/
	PTPd_Init();
	//udp initialization
	udp_echoserver_init();
	//configure state
	//init_current_state(&cur);
	cur.status = STOP;
	cur.id_sig = 0;
  while (1)
  {  
    //PROCESSING OF PEREODIC TASKS FOR LWIP
    LwIP_Periodic_Handle(gettime());
		//PROCESSING OF PEREODIC TASKS FOR PTP
		ptpd_Periodic_Handle(gettime());
		//PROCESSING OF QUEUING BUFFERS
		pbuf = pull_out_queue();
		if(pbuf != NULL) {
			h = (struct head*)pbuf->pbuf;
			if(h->type == DATA)
				transPackage(pbuf, spi_write);
			else {
				if(h->dst & MASTER) {
					h->dst &= ~MASTER;
					if(h->type == COMMAND)
						handl_command(pbuf);
				}
				if(h->dst & BAG || h->dst & INTERFACE)
					transPackage(pbuf, i2c_write);
				if(h->dst & SLAVE)
					transPackage(pbuf, eth_write);
			}
			free_buf(pbuf);
		}
		//PROCESSING OF DEBUG_INFO
		if(debug.time > 0) {
			if(debug.num == -1 || debug.num > 0)  {
				debug_info();
			}
			else
				debug.time = 0;
			if (debug.num > 0)
				debug.num --;
		}
		//PROCESSING OF TEMP BUFFER
		
  }   
}

void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	//configures the priority grouping
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	/* Enable the Ethernet global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
}

uint8_t get_cur_status_meas(void) {
	return cur.status;
}

 void set_cur_status_meas(uint8_t stat) {
	cur.status = stat;
	 return;
}

void stop_meas(void) {
	while(get_cur_status_meas() == RUN);
	PTPTriggerDisable(DP83848_PHY_ADDRESS);
	return;
}

void start_meas(int32_t *time) {
	TimeInternal t;
	stop_meas();
	t.seconds = time[0];
	t.nanoseconds = time[1];
	set_current_Time(&t);
	Delay(10000); //10s
	t.seconds += REL_TARGET_TIME;
	t.nanoseconds = 0;
	init_current_state(&cur, &t);
	return;
}
void transPackage(struct b_pool* pbuf, void (*pfunc)(uint8_t* pBuffer, unsigned int size)) {
	struct head *htr, *h = (struct head*)pbuf->pbuf;
	struct b_pool* ptr = alloc_buf(SIZE_LITTLE_BUFFER);
	unsigned int fullSize = h->size;
	unsigned int trans =0, fillSize = 0, count = 0;
	htr = (struct head*)ptr->pbuf;
	while(fullSize > 0) {
		memcpy(htr, h, sizeof(struct head));
		if(fullSize + sizeof(struct head) <= SIZE_LITTLE_BUFFER) {
			trans = fullSize;
			htr->count = LAST;
		}
		else {
			trans = SIZE_LITTLE_BUFFER - sizeof(struct head);
			htr->count = count++;
		}
		htr->size = trans;
		memcpy(ptr->pbuf + sizeof(struct head), pbuf->pbuf + sizeof(struct head) + fillSize, trans);
		pfunc((uint8_t*)htr, trans + sizeof(struct head));
		fillSize += trans;
		fullSize -= trans;
	}
	free_buf(ptr);
	return;
}

void handl_command(struct b_pool* pbuf) {
	struct b_pool* ptbuf;
	struct timings_meas *t_meas;
	int* ans;
	int* comBuf = (int*)((char*)pbuf->pbuf + sizeof(struct head));
	switch(comBuf[0]) {
		case ENABLE_DEBUG:
			enable_Debug(comBuf[1], comBuf[2]);
			ptbuf = package_answer_to_bag(&ans, 2 * sizeof(int));
			ans[0] = ENABLE_DEBUG;
			ans[1] = OK;
			transPackage(ptbuf, eth_write);
			free_buf(ptbuf);
		break;
		case GET_ENABLE_DEBUG:
			ptbuf = package_answer_to_bag(&ans, 3 * sizeof(int));
			ans[0] = A_GET_ENABLE_DEBUG;
			get_enable_Debug(&ans[1], &ans[2]);
			transPackage(ptbuf, eth_write);
			free_buf(ptbuf);
		break;
		case DISABLE_DEBUG:
			disable_Debug();
			ptbuf = package_answer_to_bag(&ans, 2 * sizeof(int));
			ans[0] = DISABLE_DEBUG;
			ans[1] = OK;
			transPackage(ptbuf, eth_write);
			free_buf(ptbuf);
		case STOP_MEAS:
			stop_meas();
			ptbuf = package_answer_to_bag(&ans, 2 * sizeof(int));
			ans[0] = STOP_MEAS;
			ans[1] = OK;
			transPackage(ptbuf, eth_write);
			free_buf(ptbuf);
		break;
		case START_MEAS:
			start_meas(&comBuf[1]);
			ptbuf = package_answer_to_bag(&ans, 2 * sizeof(int));
			ans[0] = START_MEAS;
			ans[1] = OK;
			transPackage(ptbuf, eth_write);
			free_buf(ptbuf);
		case SET_TIMINGS:
			t_meas =(struct timings_meas*)&comBuf[1];
			stop_meas();
			if(t_meas->addtADC1 != -1)
				set_addtADC1((uint32_t*)&t_meas->addtADC1);
			if(t_meas->addtOfHyd != -1)
				set_addtOfHyd((uint32_t*)&t_meas->addtOfHyd);
			if(t_meas->tarTime_sec != -1) {
				set_tarTime((uint32_t*)&t_meas->tarTime_sec, (uint32_t*)&t_meas->tarTime_sec);
			}
			if(t_meas->subtOnAmp != -1)
				set_subtOnAmp((uint32_t*)&t_meas->subtOnAmp);
			if(t_meas->w_addtOfHyd != -1)
				set_w_addtOfHyd((uint32_t*)&t_meas->w_addtOfHyd);
			if(t_meas->w_subtOnAmp != -1)
				set_w_subtOnAmp((uint32_t*)&t_meas->w_subtOnAmp);
			if(t_meas->restart != -1) {
				int32_t buf[2];
				buf[0] = t_meas->cur_tarTime_sec;
				buf[1] = t_meas->cur_tarTime_sec;
				start_meas(buf);
			}
		break;
	}
}

void enable_Debug(int32_t num, int32_t time) {
	debug.num = num;
	debug.time = time;
	return;
}

void disable_Debug(void) {
	debug.time = 0;
	return;
}

void get_enable_Debug(int32_t *num, int32_t *time) {
	*num = debug.num;
	*time = debug.time;
	return;
}

struct d_info {
	//PTP PHYTER REGISTERS
	int32_t id;
	int32_t ptp_ctl;
	int32_t ptp_sts;
	int32_t ptp_tsts;
	int32_t ptp_rate;
	int32_t ptp_trig0;
	uint32_t ptp_trig0_sec;
	uint32_t ptp_trig0_nsec;
	uint32_t ptp_trig0_puls;
	int32_t ptp_trig2;
	uint32_t ptp_trig2_sec;
	uint32_t ptp_trig2_nsec;
	uint32_t ptp_trig2_puls;
	int32_t ptp_trig6;
	uint32_t ptp_trig6_sec;
	uint32_t ptp_trig6_nsec;
	uint32_t ptp_trig6_puls;
	int32_t ptp_trig7;
	uint32_t ptp_trig7_sec;
	uint32_t ptp_trig7_nsec;
	uint32_t ptp_trig7_puls;
	int32_t ptp_txcfg0;
	int32_t ptp_txcfg1;
	int32_t ptp_rxcfg0;
	int32_t ptp_rxcfg1;
	int32_t ptp_rxcfg3;
	int32_t ptp_rxcfg4;
	int32_t ptp_coc;
	int32_t ptp_clksrc;
	int32_t ptp_gpiomon;
};

void debug_info(void) {
	struct b_pool* pbuf;
	struct head *h;
	struct d_info* ans;
	uint64_t t = gettime();
	if(t >= d_time) {
		d_time = 10 * debug.time + t;
		pbuf = alloc_buf(SIZE_LITTLE_BUFFER);
		h = (struct head*)pbuf->pbuf;
		ans = (struct d_info*)((char*)pbuf->pbuf + sizeof(struct head));
		h->type = ANSWER;
		h->dst = BAG;
		h->src = MASTER;
		h->count = LAST;
		h->size = sizeof(struct d_info);
		ans->id = DEBUG_INFO;
		d_time = 10 * debug.time + t;
		__disable_irq();
		//PTP PHYTER REGISTERS
		ans->ptp_ctl = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_CTL_PAGE, PTP_CTL);
		ans->ptp_sts = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_STS_PAGE, PTP_STS);
		ans->ptp_tsts = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS,PTP_TSTS_PAGE, PTP_TSTS);
		ans->ptp_rate = (ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RATE_PAGE, PTP_RATEL) | (ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RATE_PAGE, PTP_RATEH) << 16));
		ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG, 0 << TRIG_CSEL_SHIFT);
		ans->ptp_trig0 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG);
		PTPgetArmTrigger(DP83848_PHY_ADDRESS, 0, &ans->ptp_trig0_sec, &ans->ptp_trig0_nsec, &ans->ptp_trig0_puls);
		ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG, 2 << TRIG_CSEL_SHIFT);
		ans->ptp_trig2 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG);
		PTPgetArmTrigger(DP83848_PHY_ADDRESS, 2, &ans->ptp_trig2_sec, &ans->ptp_trig2_nsec, &ans->ptp_trig2_puls);
		ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG, 6 << TRIG_CSEL_SHIFT);
		PTPgetArmTrigger(DP83848_PHY_ADDRESS, 6, &ans->ptp_trig6_sec, &ans->ptp_trig6_nsec, &ans->ptp_trig6_puls);
		ans->ptp_trig6 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG);
		ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG, 7 << TRIG_CSEL_SHIFT);
		ans->ptp_trig7 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TRIG_PAGE, PTP_TRIG);
		PTPgetArmTrigger(DP83848_PHY_ADDRESS, 7, &ans->ptp_trig7_sec, &ans->ptp_trig7_nsec, &ans->ptp_trig7_puls);
		ans->ptp_txcfg0 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TXCFG_PAGE, PTP_TXCFG0);
		ans->ptp_txcfg1 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_TXCFG_PAGE, PTP_TXCFG1);
		ans->ptp_rxcfg0 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RXCFG_PAGE, PTP_RXCFG0);
		ans->ptp_rxcfg1 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RXCFG_PAGE, PTP_RXCFG1);
		ans->ptp_rxcfg3 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RXCFG_PAGE, PTP_RXCFG3);
		ans->ptp_rxcfg4 = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_RXCFG_PAGE, PTP_RXCFG4);
		ans->ptp_coc = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_COC_PAGE, PTP_COC);
		ans->ptp_clksrc = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_CLKSRC_PAGE, PTP_CLKSRC);
		ans->ptp_gpiomon = ETH_ReadPHYRegister_p(DP83848_PHY_ADDRESS, PTP_GPIOMON_PAGE, PTP_GPIOMON);
		__enable_irq();
		transPackage(pbuf, i2c_write);
		free_buf(pbuf);
	}
	return;
}

void init_current_state(struct current_state* c, TimeInternal* t) {
	struct head_data_adc hdata;
	c->id_sig = 0;
	set_task_pwm(&pwm_sig[cur.id_sig]);
	hdata.hash = cur.id_sig;
	hdata.sec = t->seconds;
	hdata.nsec = t->nanoseconds;
	set_task_adc(&pwm_sig[cur.id_sig], &hdata);
	set_task_phyter(t);
}


void update_current(void) {
	struct head_data_adc hdata;
	TimeInternal t;
	//update signals
	if(++cur.id_sig == NUM_PWM_SIG)
		cur.id_sig = 0;
	set_task_pwm(&pwm_sig[cur.id_sig]);
	t = set_task_phyter(NULL);
	hdata.hash = cur.id_sig;
	hdata.sec = t.seconds;
	hdata.nsec = t.nanoseconds;
	set_task_adc(&pwm_sig[cur.id_sig], &hdata);
	
}

struct b_pool* package_answer_to_bag(int** pbuf, int size) {
	struct b_pool* ptbuf;
	struct head *h;
	int* ans;
	ptbuf = alloc_buf(SIZE_LITTLE_BUFFER);
	h = (struct head*)ptbuf->pbuf;
	ans = (int*)((char*)ptbuf->pbuf + sizeof(struct head));
	*pbuf = ans;
	h->type = ANSWER;
	h->dst = BAG;
	h->src = MASTER;
	h->count = LAST;
	h->size = size;
	return ptbuf;
}
/*void clockHseInit(void) {
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
	RCC_PLLCmd(DISABLE);
	RCC_HSEConfig(RCC_HSE_Bypass);
	RCC_WaitForHSEStartUp();
	RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M_HSE, PLL_N_HSE, PLL_P_HSE, PLL_Q_HSE);
	RCC_PLLCmd(ENABLE);
	while((RCC->CR & RCC_CR_PLLRDY) == 0);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
	return;
}*/
