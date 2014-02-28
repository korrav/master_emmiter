#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"
#include "stm32f4x7_eth.h"

struct ptptime_t {
  s32_t tv_sec;
  s32_t tv_nsec;
};

err_t ethernetif_init(struct netif *netif);
err_t ethernetif_input(struct netif *netif);

void ETH_PTPTime_SetTime(struct ptptime_t * timestamp);
void ETH_PTPTime_GetTime(struct ptptime_t * timestamp);

void ETH_PTPTime_UpdateOffset(struct ptptime_t * timeoffset);
void ETH_PTPTime_AdjFreq(int32_t Adj);

  /* Examples of subsecond increment and addend values using SysClk = 72 MHz
   
   Addend * Increment = 2^63 / SysClk

    ptp_tick = Increment * 10^9 / 2^31
  
   +-----------+-----------+------------+
   | ptp tick  | Increment | Addend     |
   +-----------+-----------+------------+
   |  119 ns   |   255     | 0x1DF170C7 |
   |  100 ns   |   215     | 0x238391AA |
   |   50 ns   |   107     | 0x475C1B20 |
   |   20 ns   |    43     | 0xB191D856 |
   |   14 ns   |    30     | 0xFE843E9E |
   +-----------+-----------+------------+ 
  */

//#define ADJ_FREQ_BASE_ADDEND      0xDA2835AC   //accuracy 7 ns
//#define ADJ_FREQ_BASE_INCREMENT   15
#define ADJ_FREQ_BASE_ADDEND      0x7FFFFFFF   //accuracy 12 ns
#define ADJ_FREQ_BASE_INCREMENT   26

#endif
