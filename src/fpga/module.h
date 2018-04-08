#ifndef __MODULE_H__
#define __MODULE_H__

#include <linux/module.h>
#include <linux/wait.h>       /* waitqueue */
#include <linux/mutex.h>   /* mutex */
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 
#include "common.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Amesberger");
MODULE_DESCRIPTION("PiloT FPGA kernel module");

#define RECEIVEBUFFER_SIZE 4000

int pilot_fpga_try_send_fpga_cmd(module_slot_t slot, uint8_t *data, int size, int timeout);

typedef struct {
  uint8_t done;
  volatile int is_updated;
  volatile int bitstream_size;
  volatile int bitstream_pos;
} fpga_module_t;

typedef struct {
  uint8_t buffer[pilot_cmd_t_data_size];
  uint8_t cmd_buffer[pilot_cmd_t_data_size];
  uint32_t length;
  volatile int is_updated;
} fpga_cmd_buffer_t;

typedef struct {
  int driver_id;
  int is_cmd_handler_registered;
  int timeout;
  fpga_module_t modules[MODULES_COUNT];
  char recv_buf[RECEIVEBUFFER_SIZE];
  fpga_cmd_buffer_t cmd[MODULES_COUNT];
  volatile int recv_buf_index;
  wait_queue_head_t receive_queue;
} internals_t;

#endif
