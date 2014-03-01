/**
  ******************************************************************************
  * @file    udp_echoserver.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "udp_echoserver.h"
#include "generate_message.h"
#include "allocator.h"
#include <string.h>

static struct udp_pcb *upcb  = NULL;

static struct { 
	struct ip_addr coMadIPaddr;
	u16_t port;
} coMadAddr;

static struct b_pool *pb_eth_B, *pb_eth_L;
static struct receiver recE;

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);

void udp_echoserver_init(void)
{
	upcb = udp_new();   
	udp_bind(upcb, IP_ADDR_ANY, UDP_MODULE_PORT);
  udp_recv(upcb, udp_echoserver_receive_callback, NULL);
	IP4_ADDR(&coMadAddr.coMadIPaddr,IP_ADDR0_VISMAD, IP_ADDR1_VISMAD, IP_ADDR2_VISMAD, IP_ADDR3_VISMAD);
	coMadAddr.port = UDP_MODULE_PORT;
	
	pb_eth_B = alloc_buf(SIZE_BIG_BUFFER);
	init_receiver(&recE,  pb_eth_B->pbuf, pb_eth_B->size);
	pb_eth_L = alloc_buf(SIZE_LITTLE_BUFFER);
}

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	if(pb_eth_L->size < p->tot_len || port!= coMadAddr.port)
		return;
	pbuf_copy_partial(p, pb_eth_L->pbuf, p->tot_len, 0);
	if(generate_message(pb_eth_L->pbuf, &recE) != NOT_FULL) {
		enqueue_buf(pb_eth_B);
		pb_eth_B = alloc_buf(SIZE_BIG_BUFFER);
		init_receiver(&recE,  pb_eth_B->pbuf, pb_eth_B->size);
	}
	pbuf_free(p);
	return;
}

void eth_write(uint8_t* pBuffer, unsigned int size) {
	struct pbuf *p;
	p = pbuf_alloc(PBUF_TRANSPORT, size, PBUF_POOL);
	if (p != NULL) {
		pbuf_take(p, pBuffer, size);
    /* send udp data */
		udp_connect(upcb, &coMadAddr.coMadIPaddr, coMadAddr.port);
    udp_send(upcb, p);  
		udp_disconnect(upcb);
    /* free pbuf */
    pbuf_free(p);
  }
}
