#include <linux/module.h>
#include <linux/wait.h>       /* waitqueue */
#include <linux/mutex.h>   /* mutex */
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 
#include "common.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Amesberger");
MODULE_DESCRIPTION("PiloT FPGA kernel module");

#define RECEIVEBUFFER_SIZE 2560

typedef struct {
  uint8_t done;
  volatile int is_updated;
} fpga_module_t;

typedef struct {
  int driver_id;
  int is_cmd_handler_registered;
  int timeout;
  fpga_module_t modules[MODULES_COUNT];
  char recv_buf[RECEIVEBUFFER_SIZE];
  volatile int recv_buf_index;
  wait_queue_head_t receive_queue;
} internals_t;