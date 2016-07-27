#ifndef __TYPES_H__
#define __TYPES_H__

#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */

typedef uint16_t io_bits_t; /* 1 module has at most 16 pins (io16), so 16 bits are sufficient to store a boolean state for each gpio */

/* macros to manipulate io_bits_t */
#define io_bits_t_set(bits,bit) (bits |= (1 << bit))
#define io_bits_t_clr(bits,bit) (bits &= ~(1 << bit))
#define io_bits_t_get(bits,bit) (bits & (1 << bit))

#define IO_COUNT 8

typedef enum {
  pilot_io_module_type_invalid,
  pilot_io_module_type_i8,
  pilot_io_module_type_o8,
  pilot_io_module_type_io16,
  pilot_io_module_type_ai8,
  pilot_io_module_type_counter8
} pilot_io_module_type_t;

/* holds internal state of the gpio modules */
typedef struct {
  module_slot_t slot;                       /* slot of the module */
  int registered;                           /* 0..not registered, 1..currently registered */
  pilot_io_module_type_t module_type;        /* type of the module */
  io_bits_t gpio_requested;                 /* gpio requested, bit 0 => free, bit 1 => requested */
  io_bits_t gpio_direction;                 /* gpio direction, bit 0 => input, bit 1 => output */
  u64 gpio_states[IO_COUNT]; /* 0 => low, 1 => high for inputs / outputs, integer value for counter */
  volatile int is_state_updated[IO_COUNT];
  struct gpio_chip gpio_chip;     /* corresponding gpio_chip struct for the kernel */
} gpio_module_t;

#define COUNTER_COUNT 8

/* holds internal state of the counter module */
typedef struct {
  struct proc_dir_entry* proc_counters[COUNTER_COUNT]; /* holds the counter entries */
} counter_module_t;

typedef struct {
  struct proc_dir_entry *proc_ia[IO_COUNT]; /* holds the analog input entries */
} ai8_module_t;

/* struct that groups internal members */
typedef struct {
  int driverId;  /* as supplied by the main pilot driver pilot_register_driver() call */
  gpio_module_t gpio_modules[MODULES_COUNT];
  counter_module_t counter_modules[MODULES_COUNT];
  ai8_module_t ai8_modules[MODULES_COUNT];
  struct proc_dir_entry* proc_module_dir[MODULES_COUNT]; /* holds the base module directory entry (/proc/pilot/moduleX) */
  int gpio_base; /* gpio base number, gpio allocation starts from here */
  int gpio_max;  /* gpio max number, gpio allocation stops here */
  struct proc_dir_entry* proc_pilot_dir;
  int answer_timeout;
} internals_t;

#endif