#include "emmiter.h"
#include "generate_message.h"
#include <string.h>

int generate_message(char *buf, struct receiver *rec) {
	struct head* h = (struct head*)buf;
	switch (h->count) {
		case 0:
			if(h->size <= rec->size_buf) {
				memcpy(rec->rec.pbuf, buf, sizeof(struct head) + h->size);
				rec->cur_count = 0;
				rec->fill_size = h->size;
			}
			break;
		case LAST:
			if(h->type != rec->rec.head->type || h->src != rec->rec.head->src || h->dst != rec->rec.head->dst) { //new complete message
				if( h->size > rec->size_buf)
					return NOT_FULL;
				memcpy(rec->rec.pbuf, buf, sizeof(struct head) + h->size);
				rec->cur_count = 0;
				rec->fill_size = 0;
				return h->size;		
			}
			else if (h->size + rec->fill_size < rec->size_buf) {
				memcpy(&rec->rec.pbuf[rec->fill_size], buf + sizeof(struct head), h->size);
				h->size += rec->fill_size;
				rec->cur_count = 0;
				rec->fill_size = 0;
				return h->size;
			}		
			break;
		default:
			if(h->type == rec->rec.head->type && h->src == rec->rec.head->src && h->dst == rec->rec.head->dst && h->count - 1 == rec->rec.head->count) { //serial fragment
				if(h->size + rec->fill_size > rec->size_buf)
					break;
				memcpy(&rec->rec.pbuf[rec->fill_size], buf + sizeof(struct head), h->size);
				rec->cur_count = h->count;
				rec->fill_size += h->size;
			}
	}
	return NOT_FULL;
};

void init_receiver(struct receiver *rec, char* buf, unsigned int size_buf) {
	rec->rec.pbuf = buf;
	rec->size_buf = size_buf;
	rec->cur_count = 0;
	rec->fill_size = 0;
	return;
}

/*void* fragmentation(void* pbuf, uint32_t* psize, uint32_t size_element) {
	struct head* ph = (struct head*)pbuf;
	uint32_t sz = 0;
	if(*psize < sizeof(struct head))
		return NULL;
	if(ph->count == LAST)
		ph->count = 0;
	if(*psize < size_element)
		sz = *psize;
	else
		sz = size_element;
	*psize -= sz;
	ph->size = sz;
	
}*/
