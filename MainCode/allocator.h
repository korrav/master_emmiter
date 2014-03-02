#ifndef __ALLOCATOR
#define __ALLOCATOR
#include "stm32f4xx.h"
#include "stdio.h"

#define SIZE_BIG_BUFFER 3000
#define SIZE_LITTLE_BUFFER 1500

struct list_head {
	struct list_head* next;
	struct list_head* prev;
};

struct b_pool {
	struct list_head head;
	int status;
	int size;
	char* pbuf;
};

void init_pools_buffers(void); //initialize the buffer pools
struct b_pool* alloc_buf(size_t size); //allocate a buffer from the pool
void free_buf (struct b_pool* buffer); //release of the buffer pool
void enqueue_buf(struct b_pool* buffer);//put a buffer in the processing queue
struct b_pool* increase_buf(struct b_pool* buffer); //exchange buffer to another, having a larger size while preserving the content
struct b_pool* pull_out_queue(void); //extract from the head of the buffer queue
uint32_t check_buffer_is_free(struct b_pool* pbuf); //check whether the buffer is free
#endif /* __ALLOCATOR */
