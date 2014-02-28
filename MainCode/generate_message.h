#ifndef __GENERATE_MESSAGE
#define __GENERATE_MESSAGE
#include "stm32f4xx.h"
struct head;

//a pool structure, wherein the message is generated
struct receiver {
	union {
		struct head* head;
		char* pbuf;
	} rec;
	unsigned int size_buf;
	int cur_count;
	unsigned int fill_size;
};

//function generates a message from the received data packets
#define NOT_FULL -1
int generate_message(char *buf, struct receiver *rec); 
void init_receiver(struct receiver *rec, char* buf, unsigned int size_buf);
#endif /* __GENERATE_MESSAGE */
