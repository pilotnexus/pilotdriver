
#ifdef _LINUX_KERNEL_
#include <linux/kernel.h>
#include "module.h"
#endif
#include "queue.h"

int queue_get_room(queue_t* queue)
{
  /* cache read / write for thread safety as they might be modified during the evaluation */
  int read, write;
  read = queue->read;
  write = queue->write;

  return (read > write) ? (read - write - 1) : (QUEUE_SIZE - 1 - write + read);
}

int queue_is_full(queue_t* queue)
{
  return (((queue->write + 1) % QUEUE_SIZE) == queue->read);
}

int queue_is_empty(queue_t* queue)
{
  return (queue->write == queue->read);
}

int queue_enqueue(queue_t* queue, u16 item)
{
  if (queue_is_full(queue)) {
    return 0;
  }
  else {
    queue->arr[queue->write] = item;
    queue->write = ((queue->write + 1) == QUEUE_SIZE) ? 0 : queue->write + 1;
    return 1;
  }
}

int queue_dequeue(queue_t* queue, u16* item)
{
  if (queue_is_empty(queue)) {
    return 0;
  } 
  else {
    *item = queue->arr[queue->read];
    queue->read = ((queue->read + 1) == QUEUE_SIZE) ? 0 : queue->read + 1;
    return 1;
  }
}

//start is the start address of the block
//returns number of bytes to read, always an even number since one element has 2 bytes!
int queue_read_seq_block(queue_t *queue, void **start)
{
  if (queue->write == queue->read)
    return 0;
  else if (queue->write > queue->read) //does not overflow over boundaries
  {
    *start = (void *)&queue->arr[queue->read];
    return (queue->write - queue->read) << 1;
  }
  else //overflows, only return buffer until boundaries since otherwise it is not sequential
  {
    *start = (void *)&queue->arr[queue->read];
    return (QUEUE_SIZE - queue->read) << 1; //times two to get the actual bytesize since element is u16    
  }
}

int queue_write_seq_block(queue_t *queue, void **start)
{
  *start = (void *)&queue->arr[queue->write];
  if (queue->write >= queue->read) //write is ahead or equal, upper bound is the limit
    {
      return (QUEUE_SIZE - queue->write) << 1;
    }
    else //overflows, only return buffer until boundaries since otherwise it is not sequential
    {
      return (queue->read - queue->write) << 1; //times two to get the actual bytesize since element is u16    
    }
}

//bytes_to_skip need to be en even number since one element is 2 bytes long!
void queue_skip_read_block(queue_t *queue, int bytes_to_skip)
{
  int read = queue->read + (bytes_to_skip >> 1);
  queue->read = (read >= QUEUE_SIZE) ? 0 : read;
}

//bytes_to_skip need to be en even number since one element is 2 bytes long!
void queue_skip_write_block(queue_t *queue, int bytes_to_skip)
{
  int write = queue->write + (bytes_to_skip >> 1);
  queue->write = (write >= QUEUE_SIZE) ? 0 : write;
}
