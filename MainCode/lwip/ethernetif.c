/**
 * @file
 * Ethernet Interface for standalone applications (without RTOS) - works only for 
 * ethernet polling mode (polling for ethernet frame reception)
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include "ptpd.h"
#include "lwip/mem.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "stm32f4x7_eth.h"
#include "emmiter.h"
#include <string.h>

/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'


/* Ethernet Rx & Tx DMA Descriptors */
extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];

/* Ethernet Driver Receive buffers  */
extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; 

/* Ethernet Driver Transmit buffers */
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; 

/* Global pointers to track current transmit and receive descriptors */
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/* Global pointer for last received frame infos */
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;

static u32 ETH_GetCurrentTxBuffer(void);
static void ETH_RxPkt_ChainMode_CleanUp(FrameTypeDef * frame);

#if LWIP_PTP
ETH_DMADESCTypeDef  DMAPTPRxDscrTab[ETH_RXBUFNB], DMAPTPTxDscrTab[ETH_TXBUFNB];/* Ethernet Rx & Tx PTP Helper Descriptors */
extern __IO ETH_DMADESCTypeDef  *DMAPTPTxDescToSet;
extern __IO ETH_DMADESCTypeDef  *DMAPTPRxDescToGet;
static void ETH_PTPStart(void);
static void ETH_PTPRxPkt_ChainMode(FrameTypeDef * frame);
static void ETH_PTPRxPkt_ChainMode_CleanUp(FrameTypeDef * frame, struct ptptime_t * timestamp);
static u32 ETH_PTPTxPkt_ChainMode(u16 FrameLength, struct ptptime_t * timestamp);
static u8 * ETH_PTPTxPkt_PrepareBuffer(void);
#endif

u32_t ETH_PTPSubSecond2NanoSecond(u32_t SubSecondValue)
{
  uint64_t val = SubSecondValue * 1000000000ll;
  val >>=31;
  return val;
}

u32_t ETH_PTPNanoSecond2SubSecond(u32_t SubSecondValue)
{
  uint64_t val = SubSecondValue * 0x80000000ll;
  val /= 1000000000;
  return val;
}

void ETH_PTPTime_GetTime(struct ptptime_t * timestamp) {
	__disable_irq ();
	PHY_getTime(DP83848_PHY_ADDRESS, &timestamp->tv_sec, &timestamp->tv_nsec);
	__enable_irq ();
}


/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
#ifdef CHECKSUM_BY_HARDWARE
  int i; 
#endif
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;
  
  /* initialize MAC address in ethernet MAC */ 
  ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr); 

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
#ifdef LWIP_IGMP
  netif->flags |= NETIF_FLAG_IGMP;
#endif
   /************************************************************************/
#if LWIP_PTP
  /* Initialize Tx Descriptors list: Chain Mode */
  ETH_DMAPTPTxDescChainInit(DMATxDscrTab, DMAPTPTxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  /* Initialize Rx Descriptors list: Chain Mode  */
  ETH_DMAPTPRxDescChainInit(DMARxDscrTab, DMAPTPRxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
#else
  /* Initialize Tx Descriptors list: Chain Mode */
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  /* Initialize Rx Descriptors list: Chain Mode  */
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
#endif

  /* Enable Ethernet Rx interrrupt */
  { int i;
    for(i=0; i<ETH_RXBUFNB; i++)
    {
      ETH_DMARxDescReceiveITConfig(&DMARxDscrTab[i], ENABLE);
    }
  }
  
#ifdef CHECKSUM_BY_HARDWARE
  /* Enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
  for(i=0; i<ETH_TXBUFNB; i++)
    {
      ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
    }
#endif

   /* Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config */

#if LWIP_PTP
  /* Enable PTP Timestamping */
  ETH_PTPStart();
  /* ETH_PTPStart(ETH_PTP_CoarseUpdate); */
#endif
		
  /* Enable MAC and DMA transmission and reception */
  ETH_Start();

}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  struct pbuf *q;
  int framelength = 0;
  u8 *buffer;
	
#if LWIP_PTP
  struct ptptime_t timestamp;
  buffer = ETH_PTPTxPkt_PrepareBuffer();
#else
  buffer = (u8 *)(DMATxDescToSet->Buffer1Addr);
#endif
  
  /* copy frame from pbufs to driver buffers */
  for(q = p; q != NULL; q = q->next) 
  {
    memcpy((u8_t*)&buffer[framelength], q->payload, q->len);
	  framelength = framelength + q->len;
  }
  
  /* Note: padding and CRC for transmitted frame 
     are automatically inserted by DMA */
#if LWIP_PTP
  if( ETH_SUCCESS == ETH_PTPTxPkt_ChainMode(framelength, &timestamp) ) {
    p->time_sec = timestamp.tv_sec;
    p->time_nsec = timestamp.tv_nsec;
  } else {
    return ERR_IF;
  }
#else
  /* Prepare transmit descriptors to give to DMA*/ 
  ETH_Prepare_Transmit_Descriptors(framelength);
#endif
  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p, *q;
  u16_t len;
  int l =0;
  FrameTypeDef frame;
  u8 *buffer;
  //__IO ETH_DMADESCTypeDef *DMARxNextDesc;
	
#if LWIP_PTP
  struct ptptime_t timestamp;
#endif
 
  p = NULL;
	frame.descriptor = NULL;
  
  /* get received frame */
#if LWIP_PTP
  frame.PTPdescriptor = NULL;
  ETH_PTPRxPkt_ChainMode(&frame);
#else
  frame = ETH_Get_Received_Frame();
#endif  
  /* Obtain the size of the packet and put it into the "len" variable. */
  len = frame.length;
  buffer = (u8 *)frame.buffer;
  
  /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  
  /* copy received frame to pbuf chain */
  if (p != NULL)
  {
    for (q = p; q != NULL; q = q->next)
    {
      memcpy((u8_t*)q->payload, (u8_t*)&buffer[l], q->len);
      l = l + q->len;
    }    
  }
  
  /* Release descriptors to DMA */
#if LWIP_PTP
  ETH_PTPRxPkt_ChainMode_CleanUp(&frame, &timestamp);
  if(p != NULL)
  {
	p->time_sec = timestamp.tv_sec;
	p->time_nsec = timestamp.tv_nsec;
  }
#endif
#if !LWIP_PTP
  /* Check if frame with multiple DMA buffer segments */
  if (DMA_RX_FRAME_infos->Seg_Count > 1)
  {
    DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
  }
  else
  {
    DMARxNextDesc = frame.descriptor;
  }
  
  /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
  for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++)
  {  
    DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
    DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
  }
  
  /* Clear Segment_Count */
  DMA_RX_FRAME_infos->Seg_Count =0;
  
  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }
#endif  //!LWIP_PTP
	ETH_RxPkt_ChainMode_CleanUp(&frame);
  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
err_t ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return ERR_MEM;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);
  
  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
  return err;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/*******************************************************************************
* Function Name  : ETH_GetCurrentTxBuffer
* Description    : Return the address of the buffer pointed by the current descritor.
* Input          : None
* Output         : None
* Return         : Buffer address
*******************************************************************************/
static u32 ETH_GetCurrentTxBuffer(void)
{
  /* Return Buffer address */
  return (DMATxDescToSet->Buffer1Addr);
}

/*******************************************************************************
* Function Name  : ETH_RxPkt_ChainMode_CleanUp
* Description    : Return memory to DMA.
* Input          : frame: farme size and location
* Output         : None
* Return         : None
*******************************************************************************/
static void ETH_RxPkt_ChainMode_CleanUp(FrameTypeDef * frame)
{
  /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
  frame->descriptor->Status = ETH_DMARxDesc_OWN;

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
  {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }
}

#if LWIP_PTP

/*******************************************************************************
* Function Name  : ETH_PTPTxPkt_PrepareBuffer
* Description    : Return correct Tx Buffer according to timestamps usage
* Input          : None
* Output         : None
* Return         : Current Tx Buffer
*******************************************************************************/
static u8 * ETH_PTPTxPkt_PrepareBuffer() {
  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
  {
    /* Return ERROR: OWN bit set */
    return NULL;
  }

  DMATxDescToSet->Buffer1Addr = DMAPTPTxDescToSet->Buffer1Addr;
  DMATxDescToSet->Buffer2NextDescAddr = DMAPTPTxDescToSet->Buffer2NextDescAddr;

  return (u8 *) ETH_GetCurrentTxBuffer();
}

/*******************************************************************************
* Function Name  : ETH_PTPRxPkt_ChainMode
* Description    : Receives a packet.
* Input          : None
* Output         : frame: farme size and location
* Return         : None
*******************************************************************************/
static void ETH_PTPRxPkt_ChainMode(FrameTypeDef * frame)
{
  u32 framelength = 0;
  frame->length = 0;
  frame->buffer = 0;

  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
  {
    frame->length = ETH_ERROR;

    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
    {
      /* Clear RBUS ETHERNET DMA flag */
      ETH->DMASR = ETH_DMASR_RBUS;
      /* Resume DMA reception */
      ETH->DMARPDR = 0;
    }

    /* Return error: OWN bit set */
    return;
  }

  if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))
  {
    /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
    framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

    /* Get the addrees of the actual buffer */
    frame->buffer = DMAPTPRxDescToGet->Buffer1Addr;
  }
  else
  {
    /* Return ERROR */
    framelength = ETH_ERROR;
  }

  frame->length = framelength;


  frame->descriptor = DMARxDescToGet;
  frame->PTPdescriptor = DMAPTPRxDescToGet;

  /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
  /* Chained Mode */
  /* Selects the next DMA Rx descriptor list for next buffer to read */
  DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMAPTPRxDescToGet->Buffer2NextDescAddr);
  if(DMAPTPRxDescToGet->Status != 0)
  {
    DMAPTPRxDescToGet = (ETH_DMADESCTypeDef*) (DMAPTPRxDescToGet->Status);
  }
  else
  {
    DMAPTPRxDescToGet++;
  }

  /* Return Frame */
  return;
}


/*******************************************************************************
* Function Name  : ETH_PTPRxPkt_ChainMode_CleanUp
* Description    : Correct chain mode descriptor addresses
* Input          : frame: farme size and location
* Output         : timestamp: PTP packet timestamp
* Return         : None
*******************************************************************************/
static void ETH_PTPRxPkt_ChainMode_CleanUp(FrameTypeDef * frame, struct ptptime_t * timestamp)
{
  timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(frame->descriptor->Buffer1Addr);
  timestamp->tv_sec = frame->descriptor->Buffer2NextDescAddr;

  frame->descriptor->Buffer1Addr = frame->PTPdescriptor->Buffer1Addr;
  frame->descriptor->Buffer2NextDescAddr = frame->PTPdescriptor->Buffer2NextDescAddr;
}




/*******************************************************************************
* Function Name  : ETH_PTPTxPkt_ChainMode
* Description    : Transmits a packet, from application buffer, pointed by ppkt.
* Input          : - FrameLength: Tx Packet size.
* Output         : None
* Return         : ETH_ERROR: in case of Tx desc owned by DMA
*                  ETH_SUCCESS: for correct transmission
*******************************************************************************/
static u32 ETH_PTPTxPkt_ChainMode(u16 FrameLength, struct ptptime_t * timestamp)
{

  /* Setting the Frame Length: bits[12:0] */
  DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);

  /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
  DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

  /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
  DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

  /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
  if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
  {
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    ETH->DMATPDR = 0;
  }

  timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(DMATxDescToSet->Buffer1Addr);
  timestamp->tv_sec = DMATxDescToSet->Buffer2NextDescAddr;

  /* Clear the DMATxDescToSet status register TTSS flag */
  DMATxDescToSet->Status &= ~ETH_DMATxDesc_TTSS;

  DMATxDescToSet->Buffer1Addr = DMAPTPTxDescToSet->Buffer1Addr;
  DMATxDescToSet->Buffer2NextDescAddr = DMAPTPTxDescToSet->Buffer2NextDescAddr;

  /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
  /* Chained Mode */
  /* Selects the next DMA Tx descriptor list for next buffer to send */
  DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);

  if(DMAPTPTxDescToSet->Status != 0)
  {
    DMAPTPTxDescToSet = (ETH_DMADESCTypeDef*) (DMAPTPTxDescToSet->Status);
  }
  else
  {
    DMAPTPTxDescToSet++;
  }

  /* Return SUCCESS */
  return ETH_SUCCESS;
}

/*******************************************************************************
* Function Name  : ETH_PTPStart
* Description    : Initialize timestamping ability of ETH
* Input          : UpdateMethod:
*                       ETH_PTP_FineUpdate   : Fine Update method
*                       ETH_PTP_CoarseUpdate : Coarse Update method 
* Output         : None
* Return         : None
*******************************************************************************/
static void ETH_PTPStart(void) {
	PHY_DP83640_init(DP83848_PHY_ADDRESS);
}

#endif /* LWIP_PTP */

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampAdjFreq
* Description    : Updates time stamp addend register
* Input          : Correction value in thousandth of ppm (Adj*10^9)
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_AdjFreq(int32_t Adj)
{
	uint16_t reg = 0;
	int64_t rate = (abs(Adj) * 34360000000LL)/(1000000000LL + abs(Adj));
	reg = (rate >> 16) & 0x03FF;
	if(Adj < 0) 
		reg |= PTP_RATE_DIR;
	ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS,PTP_RATE_PAGE, PTP_RATEH, reg);
	ETH_WritePHYRegister_p(DP83848_PHY_ADDRESS,PTP_RATE_PAGE, PTP_RATEL, rate & 0xFFFF);
	return;
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampUpdateOffset
* Description    : Updates time base offset
* Input          : Time offset with sign
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_UpdateOffset(struct ptptime_t * timeoffset)
{
    uint32_t Sign;
    uint32_t SecondValue;
    uint32_t NanoSecondValue;
    uint32_t SubSecondValue;
    uint32_t addend;

    /* determine sign and correct Second and Nanosecond values */
    if(timeoffset->tv_sec < 0 || (timeoffset->tv_sec == 0 && timeoffset->tv_nsec < 0)) {
        Sign = ETH_PTP_NegativeTime;
        SecondValue = -timeoffset->tv_sec;
        NanoSecondValue = -timeoffset->tv_nsec;
    } else {
        Sign = ETH_PTP_PositiveTime;
        SecondValue = timeoffset->tv_sec;
        NanoSecondValue = timeoffset->tv_nsec;
    }

    /* convert nanosecond to subseconds */
    SubSecondValue = ETH_PTPNanoSecond2SubSecond(NanoSecondValue);
 
    /* read old addend register value*/
    addend = ETH_GetPTPRegister(ETH_PTPTSAR);

    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);

    /* Write the offset (positive or negative) in the Time stamp update high and low registers. */
    ETH_SetPTPTimeStampUpdate(Sign, SecondValue, SubSecondValue);
    /* Set bit 3 (TSSTU) in the Time stamp control register. */
    ETH_EnablePTPTimeStampUpdate();
    /* The value in the Time stamp update registers is added to or subtracted from the system */
    /* time when the TSSTU bit is cleared. */
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTU) == SET);      

    /* write back old addend register value */
    ETH_SetPTPTimeStampAddend(addend);
    ETH_EnablePTPTimeStampAddend();
}

/*******************************************************************************
* Function Name  : ETH_PTPTimeStampSetTime
* Description    : Initialize time base
* Input          : Time with sign
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_PTPTime_SetTime(struct ptptime_t * timestamp)
{
   uint32_t SecondValue;
   uint32_t NanoSecondValue;
   if(timestamp->tv_sec < 0) {
		 SecondValue = 0;
		 NanoSecondValue = 0;
	 }
	 else {
		 SecondValue = timestamp->tv_sec;
		 NanoSecondValue = abs(timestamp->tv_nsec);
	 }
	 __disable_irq ();
   PHY_setTime(DP83848_PHY_ADDRESS, &SecondValue, &NanoSecondValue);
	 __enable_irq ();
   return;	
}

#if SYS_LIGHTWEIGHT_PROT
/*
  This optional function does a "fast" critical region protection and returns
  the previous protection level. This function is only called during very short
  critical regions. An embedded system which supports ISR-based drivers might
  want to implement this function by disabling interrupts. Task-based systems
  might want to implement this by using a mutex or disabling tasking. This
  function should support recursive calls from the same task or interrupt. In
  other words, sys_arch_protect() could be called while already protected. In
  that case the return value indicates that it is already protected.

  sys_arch_protect() is only required if your port is supporting an operating
  system.
*/
sys_prot_t sys_arch_protect(void)
{
    sys_prot_t old_val = ETH->DMAIER;
    ETH->DMAIER = 0x00000000; // disable ethernet interrupts
	return old_val;
}

/*
  This optional function does a "fast" set of critical region protection to the
  value specified by pval. See the documentation for sys_arch_protect() for
  more information. This function is only required if your port is supporting
  an operating system.
*/
void sys_arch_unprotect(sys_prot_t pval)
{ 
	//( void ) pval;
    ETH->DMAIER = pval;
}
#endif



