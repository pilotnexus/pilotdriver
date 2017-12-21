#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <linux/kernel.h>

#define QUEUE_SIZE 2048

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

int queue_read_seq_block(queue_t *queue, void **start);
int queue_write_seq_block(queue_t *queue, void **start);
void queue_skip_read_block(queue_t *queue, int bytes_to_skip);
void queue_skip_write_block(queue_t *queue, int bytes_to_skip);
#endif
