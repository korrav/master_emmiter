#include "emmiter.h"
#include "allocator.h"
#include <string.h>
#define NUM_BIG_BUFFER 8
#define NUM_LITTLE_BUFFER 13

static struct b_pool head_bb[NUM_BIG_BUFFER]; 
static struct b_pool head_lb[NUM_LITTLE_BUFFER];

static char big_buffer[NUM_BIG_BUFFER][SIZE_BIG_BUFFER];
static char little_buffer[NUM_LITTLE_BUFFER][SIZE_LITTLE_BUFFER];

enum status_buffer {FREE, BUSY};

static struct list_head queue_process;

//functions operate on the list
static void init_entry_list (struct list_head* entry) {
	entry->next = entry;
	entry->prev = entry;
	return;
}
static void add_list_entry(struct list_head* entry, struct list_head* list) {
	list->next->prev = entry;
	entry->next = list->next;
	list->next = entry;
	entry->prev = list;
	return;
}

static void remove_list_entry(struct list_head* entry) {
	entry->prev->next = entry->next;
	entry->next->prev = entry->prev;
	entry->next = entry;
	entry->prev = entry;
	return;
}

//other function 

void init_pools_buffers(void) {
	int i;
	for(i = 0; i < NUM_BIG_BUFFER; i++) {
		init_entry_list(&head_bb[i].head);
		head_bb[i].status = FREE;
		head_bb[i].pbuf = &big_buffer[i][0];
		head_bb[i].size = SIZE_BIG_BUFFER;
	}
	for(i = 0; i < NUM_LITTLE_BUFFER; i++) {
		init_entry_list(&head_lb[i].head);
		head_lb[i].status = FREE;
		head_lb[i].pbuf = &little_buffer[i][0];
		head_lb[i].size = SIZE_LITTLE_BUFFER;
	}
	init_entry_list(&queue_process);
	return;
}

struct b_pool* alloc_buf(size_t size) {
	struct b_pool* b = NULL;
	int i;
	__disable_irq ();
	if(size < SIZE_LITTLE_BUFFER) {
		for(i = 0; i < NUM_LITTLE_BUFFER; i++) {
			if(head_lb[i].status == FREE) {
				head_lb[i].status = BUSY;
				b = &head_lb[i];
				break;
			}
		}
	}
	else if (size < SIZE_BIG_BUFFER) {
		for(i = 0; i < NUM_BIG_BUFFER; i++) {
			if(head_bb[i].status == FREE) {
				head_bb[i].status = BUSY;
				b = &head_bb[i];
				break;
			}
		}
	}
	__enable_irq ();
	return b;
}

void free_buf (struct b_pool* buffer) {
	buffer->status = FREE;
	return;
}

void enqueue_buf(struct b_pool* buffer) {
	__disable_irq ();
	add_list_entry(&buffer->head, &queue_process);
	__enable_irq ();
	return;
}

struct b_pool* pull_out_queue(void) {
	struct b_pool* b = NULL;
	__disable_irq ();
	if(&queue_process != queue_process.prev) {
		b = (struct b_pool*)queue_process.prev;
		remove_list_entry(queue_process.prev);
	}
	__enable_irq ();
	return b;
}

struct b_pool* increase_buf(struct b_pool* buffer) {
	struct b_pool* b = buffer;
	if(buffer->size < SIZE_BIG_BUFFER) {
		b = alloc_buf(SIZE_BIG_BUFFER);
		if(!b)
			return buffer;
		else {
			memcpy(b->pbuf, buffer->pbuf, buffer->size);
			free_buf(buffer);
		}
	}
	return b;
}

uint32_t check_buffer_is_free(struct b_pool* pbuf) {
	uint32_t status = 0;
	if(pbuf->status == FREE)
		status =1;
	return status;
}
