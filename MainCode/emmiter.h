#ifndef __EMMITER
#define __EMMITER
#include "stm32f4xx.h"
#include "stm32f4x7_eth_bsp.h"
#include "udp_echoserver.h"
#include "allocator.h"
#include "generate_message.h"
#include "lwipopts.h"
#include "netconf.h"
#include "time.h"
#include <stdbool.h>
#include <stdio.h>
#include "ptpd.h"
#include "i2cExchange.h"
#include "spiExchange.h"
#include "ADC_control.h"
#include "pwm.h"
#include "signal.h"

enum srcdst { BAG = 1, INTERFACE = 2, MASTER = 4, SLAVE = 8};
enum typ { DATA, COMMAND, ANSWER};
enum id_command {ENABLE_DEBUG, GET_ENABLE_DEBUG, DISABLE_DEBUG};
enum id_answer {A_GET_ENABLE_DEBUG, DEBUG_INFO};

//packet header format
#define LAST -1
struct head{
	int size; //buffer size
	int count; //serial number of buffer
	int dst;	//destination
	int src;	//source
	int type;	//buffer type
};

//triggers numbers
#define TRIG_DAC    0
#define TRIG_ADC    2
#define TRIG_ON_AMP 6
#define TRIG_OF_HYD 7

//slave or master
#define IS_MASTER 0
#define IS_SLAVE  1 

//configure Ethernet
/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   0x06
#define MAC_ADDR1   0x02
#define MAC_ADDR2   0
#define MAC_ADDR3   0xFE
#define MAC_ADDR4   0xFF
#define MAC_ADDR5   ID_EMMITER
 
#if ID_EMMITER == IS_MASTER
/*Static IP ADDRESS MAD: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   203
#define IP_ADDR3   41

/*Static IP ADDRESS visMAD: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0_VISMAD   192
#define IP_ADDR1_VISMAD   168
#define IP_ADDR2_VISMAD   203
#define IP_ADDR3_VISMAD   42
#else
/*Static IP ADDRESS MAD: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   203
#define IP_ADDR3   42

/*Static IP ADDRESS visMAD: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0_VISMAD   192
#define IP_ADDR1_VISMAD   168
#define IP_ADDR2_VISMAD   203
#define IP_ADDR3_VISMAD   41
#endif //ID_EMMITER == IS_MASTER

/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

#define UDP_MODULE_PORT    3021   /* define the UDP local connection port */

#define MII_MODE
#define CHECKSUM_BY_HARDWARE  //checksum payoland ip4

void update_current(void); //update the current system status (current signal, timings, etc.)
#endif /* __EMMITER */
