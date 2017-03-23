/* 
  
*/
#ifndef __TYPES_H__
#define __TYPES_H__

#include <linux/list.h>
#include <linux/proc_fs.h>   // needed for functions to manage /proc/xxx files
#include "queue.h"
#include "pilotstm.h"

typedef enum {
  GPIO_MODE_INPUT,
  GPIO_MODE_OUTPUT,
  GPIO_MODE_ALT
} gpio_mode_t;

/* holds configuration info about gpios */
typedef struct { 
  int gpio;             // set to the gpio that should be requested
  gpio_mode_t gpio_mode;// set to the gpio mode that should be configured
  int gpio_alternative; // set to the alternative that the gpio should be configured, if gpio_mode == GPIO_MODE_ALT
  int gpio_is_requested;// set if the gpio was successfully requested, otherwise 0
} gpio_config_t;

/* holds info about a registered driver */
typedef struct {
  int id;                      // auto assigned id of the driver
  register_driver_t* driver;   // the registered driver, as provided by the driver when calling pilot_register_driver()
  unsigned int totalBytesSent; // running total of all bytes sent
  unsigned int totalBytesRecv; // running total of all bytes recv
} driver_t;

/* holds info about a module */
typedef struct {
  char* name_module;                                /* name of the module - pointer to a zero terminated string */
  driver_t* driver;                                 /* pointer to the driver that controls it, if any */
  //module_type_t module_type;                        /* type of the module, returned by the drivers callback_assign_slot() function */
  pilot_module_type_t type;                          /* type of the module, returned by the firmware */
  volatile int type_is_updated;                     /* set when the module type is updated */
  stm_bufferstate_t bufferstate[MODULE_PORT_COUNT]; /* current bufferstate of the module on the stm */
  pilot_eeprom_uid_t uid;                            /* last received uid of the module */
  volatile int uid_is_updated;                      /* set when the uid is updated */
  pilot_eeprom_hid_t hid;                            /* last received hid of the module */
  volatile int hid_is_updated;                      /* set when the hid is updated */
  pilot_eeprom_fid_t fid;                            /* last received fid of the module */
  volatile int fid_is_updated;                      /* set when the fid is updated */
  pilot_eeprom_data_t user[EEPROM_USER_DATA_COUNT];  /* last received user data of the module */
  volatile int user_is_updated[EEPROM_USER_DATA_COUNT]; /* set when the user data is updated */
} module_t;

#define MAX_CMD_TYPE 256

/* holds statistical info which can be read using /proc/pilot/stats */
typedef struct {
  unsigned int recv_byte_count[256];
  unsigned int sent_byte_count[256];
  unsigned int recv_cmd_count;
  unsigned int sent_cmd_count;
  unsigned int recv_cmd_type_count[MAX_CMD_TYPE];
  unsigned int sent_cmd_type_count[MAX_CMD_TYPE];
  unsigned int crc_errors;
} stats_t;

#define MAX_TARGETS 255
typedef struct {
  void(*callbacks[MAX_TARGETS])(char data);
} stream_callback_t;

typedef struct {
  pilot_test_run_result_t result;
  int index_failed;
  int count_failed_low;
  int count_failed_high;
  int count_failed;
  int count_success;
  int count_total;
} test_result_t;

/* hold private members in internals_t */
typedef struct 
{
  volatile unsigned int* Spi0;     /* pointer to the memory mapped Spi interface */
  driver_t drivers[DRIVERS_COUNT]; /* registered drivers */
  module_t modules[MODULES_COUNT]; /* configured modules */
  queue_t TxQueue;                 /* data to be send to the microcontroller as commands */
  queue_t RxQueue;                 /* received data */

  spinlock_t       list_cmd_handler_lock; /* a spinlock to synchronize list_cmd_handler access */
  struct list_head list_cmd_handler;      /* the head of the list of registered command handlers - a standalone list_head structure */

  pilot_current_cmd_t current_cmd; /* collects the current command that is being received from the stm */

  int irq_data_m2r; /* the requested irq handler, stores return valued of rpc_irq_init() and used as argument for rpc_irq_deinit() */
  
  volatile unsigned int* GpioGet; /* pointer to the memory mapped access to read gpio input values  */
  volatile unsigned int* GpioSet;
  volatile unsigned int* GpioClr;

  struct proc_dir_entry* proc_pilot_dir; /* base proc directory '/proc/pilot' */
  struct proc_dir_entry* proc_pilot_modules_dir[MODULES_COUNT]; /* module directories '/proc/pilot/moduleX' */
  struct proc_dir_entry* proc_pilot_modules_eeprom_dir[MODULES_COUNT]; /* module directories '/proc/pilot/moduleX/eeprom' */

  int spiclk; /* holds the spiclk divisor 250 / spiclk = clock speed in MHz */

  stats_t stats; /* stats can be read from '/proc/pilot/stats' */
  pilot_cmd_t last_recv_cmd; /* last received cmd, used by '/proc/pilot/last_recv_cmd' */
  stm_bufferstate_t stm_bufferstate; /* current bufferstate of the stm */

  test_result_t test_result; /* last received test run result */
  volatile int test_result_is_updated; /* set when the test_result is updated */

  uint32_t uid; /* last received uid */
  volatile int uid_is_updated; /* set when the uid is updated */

  stream_callback_t stream_callback;

} internals_t;

#endif
