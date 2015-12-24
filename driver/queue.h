#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <linux/kernel.h>

#define QUEUE_SIZE 256

typedef struct
{
  u16 arr[QUEUE_SIZE];
  int read, write;
} queue_t;

int queue_get_room(queue_t* queue);
int queue_enqueue(queue_t* queue, u16 item);
int queue_dequeue(queue_t* queue, u16* item);
int queue_is_empty(queue_t* queue);
int queue_is_full(queue_t* queue);

#endif
