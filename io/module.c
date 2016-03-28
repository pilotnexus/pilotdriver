#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */
#include <linux/seq_file.h>
#include <linux/string.h>     /* included for the memset() function */
#include <asm/gpio.h>         /* needed for struct gpio_chip, gpiochip_add(), gpiochip_remove() */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h"
#include "common.h"
#include "types.h"            /* io_module_type_t */
#include "io_stm.h"           /* defines for encoding/decoding gpio state */
#include "io_commands.h"
#include "irq.h"              /* support for generating irq for input changes */

MODULE_LICENSE("GPL");

// *******************************************************************
// START forward declaration
static int  __init pilot_io_init(void); /* kernel module entry function */
static void __exit pilot_io_exit(void); /* kernel module exit function */

static void pilot_io_callback_recv         (module_slot_t slot, module_port_t port, spidata_t rx); /* callback function when data arrives */
static int  pilot_io_callback_assign_slot  (module_slot_t slot, const pilot_module_type_t *module_type);    /* callback function when a slot is assigned to the driver */
static int  pilot_io_callback_unassign_slot(module_slot_t slot);               /* callback function when a slot is unassigned from the driver */
static int  pilot_io_callback_can_assign   (const pilot_module_type_t *module_type); /* callback function when a slot is assigned but the argument used does not specify the driver */
static pilot_cmd_handler_status_t pilot_io_callback_cmd_received(pilot_cmd_t cmd); /* callback function for main driver cmd handler */
static void pilot_io_request_counter_values(module_slot_t slot, pilot_counter_target_t counter_target);
static void pilot_io_request_input_value(module_slot_t slot, pilot_input_target_t input);

static int pilot_io_register_gpio_module(module_slot_t slot, pilot_io_module_type_t module_type);
static int pilot_io_unregister_gpio_module(module_slot_t slot);
static pilot_io_module_type_t pilot_io_get_module_type(const pilot_module_type_t *module_type);
static int pilot_io_register_counter_module(module_slot_t slot);

static const char* pilot_io_get_module_string(pilot_io_module_type_t module_type);
static int  pilot_io_gpio_get_next_base(int gpio_count);
static void pilot_io_gpio_chip_init(module_slot_t slot, struct gpio_chip* gpio_chip, pilot_io_module_type_t module_type);

static void pilot_io_proc_init(void);
static void pilot_io_proc_deinit(void);

/* END forward declaration of proc functions */
/////////////////////////////////////////////////////////////

static void pilot_io_proc_init_counter_module(module_slot_t slot);
static void pilot_io_proc_deinit_counter_module(module_slot_t slot);

static void pilot_io_proc_init_ia8_module(module_slot_t slot);
static void pilot_io_proc_deinit_ia8_module(module_slot_t slot);

/////////////////////////////////////////////////////////////
/* START forward declaration of gpiolib callback functions */
static int  pilot_io_gpio_chip_cb_request          (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
static void pilot_io_gpio_chip_cb_free             (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
/* only supported on a newer kernel */
//static int  pilot_io_gpio_chip_cb_get_direction    (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
static int  pilot_io_gpio_chip_cb_direction_input  (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
static int  pilot_io_gpio_chip_cb_get              (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
static int  pilot_io_gpio_chip_cb_direction_output (struct gpio_chip* chip, unsigned offset, int value);         /* callback function for kernel gpiolib */
static int  pilot_io_gpio_chip_cb_set_debounce     (struct gpio_chip* chip, unsigned offset, unsigned debounce); /* callback function for kernel gpiolib */
static void pilot_io_gpio_chip_cb_set              (struct gpio_chip* chip, unsigned offset, int value);         /* callback function for kernel gpiolib */
static int  pilot_io_gpio_chip_cb_to_irq           (struct gpio_chip* chip, unsigned offset);                    /* callback function for kernel gpiolib */
static void pilot_io_gpio_chip_cb_dbg_show         (struct seq_file *s, struct gpio_chip* chip);                 /* callback function for kernel gpiolib */
/* END forward declaration of gpiolib callback functions */
/////////////////////////////////////////////////////////////


// END forward declaration
// *******************************************************************

// *******************************************************************
// START local members
/* description of our io module driver, used to register with the main pilot driver */
static register_driver_t register_driver = {
  .name                   = "io",
  .callback_assign_slot   = pilot_io_callback_assign_slot,
  .callback_unassign_slot = pilot_io_callback_unassign_slot,
  .callback_recv          = pilot_io_callback_recv,
  .callback_can_assign    = pilot_io_callback_can_assign
};

#define MAX_LABEL_LENGTH 255
static char gpio_chip_labels[MODULES_COUNT][MAX_LABEL_LENGTH];

/* internal variables */
static internals_t _internals = { 
  .driverId = -1, 
  .gpio_base=  55, /* default to the first gpio number that is not within the default ngpio range on the rpi */
  .gpio_max = 255, /* some sane default, see include/asm-generic/gpio.h */
  .gpio_modules = { { module_slot_1 }, { module_slot_2 }, { module_slot_3 }, { module_slot_4 } },
  .answer_timeout = 100
};

/* declare the pilot command handler */
static pilot_cmd_handler_t pilot_cmd_handler = {
  .callback_cmd_received = pilot_io_callback_cmd_received
};

static const struct file_operations proc_pilot_io_gpio_base_fops,
                                    proc_pilot_io_gpio_max_fops,
                                    proc_pilot_module_counter_fops,
                                    proc_pilot_module_ia8_fops;

// END local members
// *******************************************************************

/* main entry point */
module_init(pilot_io_init);

/* main exit point */
module_exit(pilot_io_exit);

/* initialization routine, called when the module is loaded */
static int __init pilot_io_init()
{
  int ret = SUCCESS;
  
  /* register with the base driver */
  if ((_internals.driverId = pilot_register_driver(&register_driver)) < 0)
  {
    LOG(KERN_ERR, "pilot_register_driver() failed with %i", _internals.driverId);
    ret = -1;
  }
  else
  {
    /* register our pilot command handler */
    pilot_register_cmd_handler(&pilot_cmd_handler);

    /* init the proc file system */
    pilot_io_proc_init();

    /* request an autoconfiguration of all modules */
    pilot_auto_configure();
  }

  return ret;
}

/* io module cleanup function, called when removing the module */
static void __exit pilot_io_exit()
{
  int i;

  /* unregister our pilot command handler */
  pilot_unregister_cmd_handler(&pilot_cmd_handler);

  /* unregister with the base driver */
  if (_internals.driverId >= 0)
    pilot_unregister_driver( _internals.driverId );

  /* unassign all slots */
  for (i = 0; i < MODULES_COUNT; i++)
    pilot_io_callback_unassign_slot((module_slot_t)i);

  /* unregister /proc filesystem entries */
  pilot_io_proc_deinit();
}

static const char io16[]     = "io16";
static const char i8[]       = "i8";
static const char o8[]       = "o8";
static const char counter8[] = "counter8";
static const char ia8[]      = "ia8";

/* helper function that retrieves the io_module_type_t */
static pilot_io_module_type_t pilot_io_get_module_type(const pilot_module_type_t *module_type)
{
  pilot_io_module_type_t io_type;

  if (strncmp(io16, module_type->name, 4) == 0)
    io_type = pilot_io_module_type_io16;
  else if (strncmp(i8, module_type->name, 2) == 0)
    io_type = pilot_io_module_type_i8;
  else if (strncmp(o8, module_type->name, 2) == 0)
    io_type = pilot_io_module_type_o8;
  else if (strncmp(counter8, module_type->name, 8) == 0)
    io_type = pilot_io_module_type_counter8;
  else if (strncmp(ia8, module_type->name, 3) == 0)
    io_type = pilot_io_module_type_ia8;
  else
    io_type = pilot_io_module_type_invalid;

  return io_type;
}

static const char* pilot_io_get_module_string(pilot_io_module_type_t module_type)
{
  switch (module_type)
  {
    case pilot_io_module_type_i8: return i8;
    case pilot_io_module_type_o8: return o8;
    case pilot_io_module_type_io16: return io16;
    case pilot_io_module_type_counter8: return counter8;
    default: return NULL;
  }
}

/* the module type specifies the number of needed gpios */
static int pilot_io_gpio_get_next_base(int gpio_count)
{
  /* start with the base */ 
  int i, base_taken, gpio_base; struct gpio_chip* gc;

  /* look for the next free gpio range */

  /* test all plausible gpios, start with the globally defined base up to the defined max */  
  for (gpio_base = _internals.gpio_base; i < _internals.gpio_max; gpio_base++) 
  {
    base_taken = 0;

    /* check if the wanted gpio range is not already used by an registered module */
    for (i = 0; i < MODULES_COUNT; i++)
    {
      if (_internals.gpio_modules[i].registered) {
        gc = &_internals.gpio_modules[i].gpio_chip;
        /* is the gpio range taken? */
        if ( (gpio_base >= gc->base && (gpio_base <= (gc->base + gc->ngpio)) ) /* is the start inside the gpio_chip range */ ||
             (((gpio_base + gpio_count) >= gc->base) && ((gpio_base + gpio_count) <= (gc->base + gc->ngpio))) /* is the end inside the gpio_chip range */ )
        {
          base_taken = 1;
          break;
        }
      }
    }

    if (!base_taken)
      break;
  }  

  return gpio_base;
}

/* sets a gpio_chip struct up */
static void pilot_io_gpio_chip_init(module_slot_t slot,          /* slot number */
                                   struct gpio_chip* gpio_chip, /* gpio_chip to initialize */
                                   pilot_io_module_type_t module_type)   /* type of the inserted module */
{
  int gpio_base, gpio_count;
  LOG_DEBUG("pilot_io_gpio_chip_init(slot=%i, module_type=%i) called", slot, module_type);

  /* get the number of needed gpios */
  switch (module_type) {
    case pilot_io_module_type_i8:
    case pilot_io_module_type_o8:
      gpio_count = 8; break;
    case pilot_io_module_type_io16: gpio_count = 16; break;
    default: gpio_count = 0; break;
  }  

  /* calc the next gpio base for this module */
  gpio_base = pilot_io_gpio_get_next_base(gpio_count);
  LOG_DEBUG("pilot_io_gpio_get_next_base returned: %i", gpio_base);

  /* update the chip label */
  sprintf(gpio_chip_labels[slot], "pilot%s_%i", pilot_io_get_module_string(module_type), slot);
  gpio_chip->label = gpio_chip_labels[slot];
  gpio_chip->owner = THIS_MODULE;

  gpio_chip->request           = pilot_io_gpio_chip_cb_request;
  gpio_chip->free              = pilot_io_gpio_chip_cb_free;
  gpio_chip->direction_input   = pilot_io_gpio_chip_cb_direction_input;
  gpio_chip->get               = pilot_io_gpio_chip_cb_get;
  gpio_chip->direction_output  = pilot_io_gpio_chip_cb_direction_output;
  gpio_chip->set_debounce      = pilot_io_gpio_chip_cb_set_debounce;
  gpio_chip->set               = pilot_io_gpio_chip_cb_set;
  gpio_chip->to_irq            = pilot_io_gpio_chip_cb_to_irq;
  gpio_chip->dbg_show          = pilot_io_gpio_chip_cb_dbg_show;
  gpio_chip->can_sleep         = 1;
  gpio_chip->ngpio             = gpio_count;
  gpio_chip->base              = gpio_base;
}

static int pilot_io_register_gpio_module(module_slot_t slot, pilot_io_module_type_t module_type)
{
  int ret; struct gpio_chip* gpio_chip; gpio_module_t* gpio_module; int i;

  LOG_DEBUG("pilot_io_register_gpio_module(slot=%i, module_type=%i)", slot, module_type);

  gpio_module = &_internals.gpio_modules[slot];

  gpio_chip = &gpio_module->gpio_chip;

  /* prepare the gpio_chip that describes this module to the kernel */
  pilot_io_gpio_chip_init(slot, gpio_chip, module_type);

  if ((ret = gpiochip_add(gpio_chip)) == SUCCESS)
  {
    /* update the gpio_module */
    gpio_module->module_type = module_type;
    gpio_module->registered     = 1;
    //gpio_module->gpio_state     = 0;

    for (i = 0; i < IO_COUNT; i++)
      gpio_module->gpio_states[i] = 0;

    gpio_module->gpio_direction = (module_type == pilot_io_module_type_o8) ? 0xff : 0;
  }
  else
  {
    LOG(KERN_ERR, "gpiochip_add() failed!");
  }

  return ret;
}

static int pilot_io_unregister_gpio_module(module_slot_t slot)
{
  int i;
  int ret = SUCCESS;
  struct gpio_chip* gpio_chip;
  LOG_DEBUG("pilot_io_unregister_gpio_module(slot=%i)", slot);
  if (_internals.gpio_modules[slot].registered) {
    gpio_chip = &_internals.gpio_modules[slot].gpio_chip;
    gpiochip_remove(gpio_chip);
    _internals.gpio_modules[slot].registered = 0;
  }

  /* reset the states */
  for (i = 0; i < IO_COUNT; i++)
    _internals.gpio_modules[slot].gpio_states[i] = 0;

  return ret;
}

static int pilot_io_register_ia8_module(module_slot_t slot)
{
  LOG_DEBUG("pilot_io_register_ia8_module(slot=%i)", slot);

  pilot_io_proc_init_ia8_module(slot);

  return SUCCESS;
}

static int pilot_io_register_counter_module(module_slot_t slot)
{
  LOG_DEBUG("pilot_io_register_counter_module(slot=%i)", slot);

  pilot_io_proc_init_counter_module(slot);

  return SUCCESS;
}

/* requests the value of the input from the stm by sending a get_input command */
static void pilot_io_request_input_value(module_slot_t slot, pilot_input_target_t input)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, module_port_1);
  cmd.type = pilot_cmd_type_input_get_input;
  cmd.data[pilot_input_index_target] = input;
  pilot_send_cmd(&cmd);
}

/* requests the values of the counters from the stm by sending a get_counters command */
static void pilot_io_request_counter_values(module_slot_t module_slot, pilot_counter_target_t counter_target)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target                          = target_t_from_module_slot_and_port(module_slot, module_port_1);
  cmd.type                            = pilot_cmd_type_input_get_counter;
  cmd.data[pilot_counter_index_target] = counter_target;
  pilot_send_cmd(&cmd);
}

/* sends a set_value command */
static void pilot_io_set_output_value(module_slot_t slot, pilot_output_target_t output, int value)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target                              = target_t_from_module_slot_and_port(slot, 0);
  cmd.type                                = pilot_cmd_type_output_set_value;
  cmd.data[(int)pilot_output_index_target] = output;
  cmd.data[(int)pilot_output_index_value]  = value;
  pilot_send_cmd(&cmd);
}

/* sends a set direction command */
static void pilot_io_set_io_direction(module_slot_t slot, pilot_io16_block_t block, pilot_io16_direction_t direction)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target                                             = target_t_from_module_slot_and_port(slot, 0);
  cmd.type                                               = pilot_cmd_type_io16_set_direction;
  cmd.data[(int)pilot_io16_set_direction_index_block]     = block;
  cmd.data[(int)pilot_io16_set_direction_index_direction] = direction;
  pilot_send_cmd(&cmd);
}

static int pilot_io_try_get_counter(int module_index, int counter_index, int timeout)
{
  unsigned long timestamp;
  counter_module_t* counter_module;
  gpio_module_t* gpio_module;
  int timedout = 0;
  LOG_DEBUG("pilot_io_try_get_counter() called");

  counter_module = &_internals.counter_modules[module_index];
  gpio_module = &_internals.gpio_modules[module_index];

  /* reset the is_state_updated flag */
  gpio_module->is_state_updated[counter_index] = 0;

  /* request a transmission of the counter */
  pilot_io_request_counter_values(module_index, counter_index);

  /* pick a time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the request is fulfilled or the timeout is reached */
  LOG_DEBUG("pilot_io_try_get_counter() waits for the counter request to be fulfilled");
  while(!gpio_module->is_state_updated[counter_index])
  {
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
  }

  if (timedout)
  {
    LOG_INFO("pilot_io_try_get_counter() timeout reached while waiting for counter value!");
  }

  return timedout ? -1 : SUCCESS;
}

/* sends a set counter value command to the stm */
static void pilot_io_set_counter_value(module_slot_t module_slot,
                                      pilot_counter_target_t counter_target,
                                      u64 counter_value)
{
  pilot_cmd_t cmd;

  LOG_DEBUG("pilot_io_set_counter_value() called");

  memset(&cmd, 0, sizeof(pilot_cmd_t));                                                                       /* clear the cmd struct */
  cmd.target                             = target_t_from_module_slot_and_port(module_slot, module_port_1); /* set the target */
  cmd.type                               = pilot_cmd_type_input_set_counter;                                  /* set the cmd type */
  cmd.data[pilot_counter_index_target]    = counter_target;                                                   /* set the counter target */
  cmd.data[pilot_counter_index_value + 0] = (counter_value >> 6*8) & 0xFF;                                    /* set the counter value */
  cmd.data[pilot_counter_index_value + 1] = (counter_value >> 5*8) & 0xFF;
  cmd.data[pilot_counter_index_value + 2] = (counter_value >> 4*8) & 0xFF;
  cmd.data[pilot_counter_index_value + 3] = (counter_value >> 3*8) & 0xFF;
  cmd.data[pilot_counter_index_value + 4] = (counter_value >> 2*8) & 0xFF;
  cmd.data[pilot_counter_index_value + 5] = (counter_value >> 1*8) & 0xFF;
  cmd.data[pilot_counter_index_value + 6] = (counter_value >> 0*8) & 0xFF;

  pilot_send_cmd(&cmd); /* send the command */
}

static int pilot_io_try_get_ia(int module_index, int input_index, int timeout)
{
  unsigned long timestamp;
  int timedout = 0;
  gpio_module_t *gpio_module;
  LOG_DEBUG("pilot_io_try_get_ia() called");

  gpio_module = &_internals.gpio_modules[module_index];

  /* reset the is_state_updated flag */
  gpio_module->is_state_updated[input_index] = 0;

  /* request a transmission of the input */
  pilot_io_request_input_value(module_index, input_index);

  /* 1 second in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the request is fulfilled or the timeout is reached */
  LOG_DEBUG("pilot_io_try_get_ia() waits for the input request to be fulfilled");
  while(!gpio_module->is_state_updated[input_index])
  {
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
  }

  if (timedout)
  {
    LOG_INFO("pilot_io_try_get_ia() timeout reached while waiting for input value!");
  }

  return timedout ? -1 : SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// START gpiolib callback function implementations
static int  pilot_io_gpio_chip_cb_request (struct gpio_chip* chip, unsigned offset)
{
  gpio_module_t* gpio_module;
  LOG_DEBUG("pilot_io_gpio_chip_cb_request(offset=%i) called", offset);

  gpio_module = container_of(chip, gpio_module_t, gpio_chip);

  if (!io_bits_t_get(gpio_module->gpio_requested, offset)) {
    LOG_DEBUG("requesting gpio with base %i and offset %i was successful", chip->base, offset);
    io_bits_t_set(gpio_module->gpio_requested, offset);
    return SUCCESS;
  }

  LOG_DEBUG("requesting gpio with base %i and offset %i failed", chip->base, offset);
  return -EBUSY;
}

static void pilot_io_gpio_chip_cb_free (struct gpio_chip* chip, unsigned offset)
{
  gpio_module_t* gpio_module;
  LOG_DEBUG("pilot_io_gpio_chip_cb_free(offset=%i) called", offset);

  gpio_module = container_of(chip, gpio_module_t, gpio_chip);

  if (io_bits_t_get(gpio_module->gpio_requested, offset)) {
    LOG_DEBUG("freed gpio with base %i and offset %i", chip->base, offset);
    io_bits_t_clr(gpio_module->gpio_requested, offset);  
  }
}

//static int  pilot_io_gpio_chip_cb_get_direction    (struct gpio_chip* chip, unsigned offset)
//{
//  return 0;
//}

static int  pilot_io_gpio_chip_cb_direction_input  (struct gpio_chip* chip, unsigned offset)
{
  gpio_module_t* gpio_module; 
  pilot_io16_block_t block;

  LOG_DEBUG("pilot_io_gpio_chip_cb_direction_input()");

  gpio_module = container_of(chip, gpio_module_t, gpio_chip);

  /* check what type of io module it is*/
  switch (gpio_module->module_type) {
    case pilot_io_module_type_i8:
    case pilot_io_module_type_io16:

      /* get the block that the output belongs to */
      if (offset >= 0 && offset < 4)
        block = pilot_io16_block_0_to_3;
      else if (offset >= 4 && offset < 8)
        block = pilot_io16_block_4_to_7;
      else if (offset >= 8 && offset < 12)
        block = pilot_io16_block_8_to_11;
      else if (offset > 12 && offset < 16)
        block = pilot_io16_block_12_to_15;
      else
        return -EINVAL;

      /* send the set direction cmd */
      pilot_io_set_io_direction(gpio_module->slot, block, pilot_io16_direction_input);
      return SUCCESS;

    case pilot_io_module_type_o8: /* changing the direction to input is not allowed on the output module */
    default:
      return -EINVAL;
  }

}

static int  pilot_io_gpio_chip_cb_direction_output (struct gpio_chip* chip, unsigned offset, int value)
{
  gpio_module_t* gpio_module;
  pilot_io16_block_t block;

  LOG_DEBUG("pilot_io_gpio_chip_cb_direction_output()");

  gpio_module = container_of(chip, gpio_module_t, gpio_chip);

  /* if it's an IO16 module, send set io direction cmd first */
  if (gpio_module->module_type == pilot_io_module_type_io16)
  {
    /* get the block that the output belongs to */
    if (offset >= 0 && offset < 4)
      block = pilot_io16_block_0_to_3;
    else if (offset >= 4 && offset < 8)
      block = pilot_io16_block_4_to_7;
    else if (offset >= 8 && offset < 12)
      block = pilot_io16_block_8_to_11;
    else if (offset > 12 && offset < 16)
      block = pilot_io16_block_12_to_15;
    else
      return -EINVAL;

    pilot_io_set_io_direction(gpio_module->slot, block, pilot_io16_direction_output);
  }

  if (gpio_module->module_type == pilot_io_module_type_o8 ||
      gpio_module->module_type == pilot_io_module_type_io16)
  {
    /* set the output value */
    pilot_io_set_output_value(gpio_module->slot, (pilot_output_target_t)offset, value);

    return SUCCESS;
  }
  else
    return -EINVAL;
}

static int pilot_io_get_slot(gpio_module_t *gpio_module)
{
  if (&_internals.gpio_modules[0] == gpio_module)
    return 0;
  else if (&_internals.gpio_modules[1] == gpio_module)
    return 1;
  else if (&_internals.gpio_modules[2] == gpio_module)
    return 2;
  else
    return 3;
}

static int pilot_io_gpio_chip_cb_get (struct gpio_chip* chip, unsigned offset)
{
  gpio_module_t* gpio_module;
  int input_index, timedout, ret;
  module_slot_t slot;
  unsigned long timestamp;

  timedout = 0;
  input_index = offset;

  LOG_DEBUG("pilot_io_gpio_chip_cb_get(offset=%i)", offset);

  /* get the gpio_module responsible for the gpio_chip */
  gpio_module = container_of(chip, gpio_module_t, gpio_chip); 

  /* get the slot of the gpio module */
  slot = pilot_io_get_slot(gpio_module);

  /* reset the input state */
  gpio_module->is_state_updated[input_index] = 0;

  LOG_DEBUG("requesting input state slot=%i, index=%i", slot, input_index);

  /* request the input state from the pilot */
  pilot_io_request_input_value(slot, (pilot_input_target_t)input_index);

  /* 100ms in the future */
  timestamp = jiffies + (HZ / 10); 

  /* wait for the input state to be updated */
  while(!gpio_module->is_state_updated[input_index])
  {
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
  }

  /* return the state of the input if successfull */
  if (!timedout)
  {
    ret = gpio_module->gpio_states[input_index];
  }
  else
  {
    /* return an error if not successful */
    LOG_INFO("pilot_io_gpio_chip_cb_get() timeout reached while waiting for input value!");
    ret = -1;
  }

  return ret;
}

static int  pilot_io_gpio_chip_cb_set_debounce(struct gpio_chip* chip, unsigned offset, unsigned debounce)
{
  LOG_DEBUG("pilot_io_gpio_chip_cb_set_debounce(offset=%i, debounce=%i) called", offset, debounce);
  return -ENOSYS;
}

static void pilot_io_gpio_chip_cb_set(struct gpio_chip* chip, unsigned offset, int value)
{
  gpio_module_t* gpio_module;
  LOG_DEBUG("pilot_io_gpio_chip_cb_set(offset=%i, value=%i) called", offset, value);
  gpio_module = container_of(chip, gpio_module_t, gpio_chip);

  if (gpio_module->module_type == pilot_io_module_type_o8 ||
      gpio_module->module_type == pilot_io_module_type_io16)
  {
    /* save the internal state */
    gpio_module->gpio_states[offset] = value;

    /* send the set output value cmd */
    pilot_io_set_output_value(gpio_module->slot, (pilot_output_target_t)offset, value);
  }
}

static int  pilot_io_gpio_chip_cb_to_irq(struct gpio_chip* chip, unsigned offset)
{
  LOG_DEBUG("pilot_io_gpio_chip_cb_to_irq(offset=%i) called", offset);

  return -ENOSYS;
}

static void pilot_io_gpio_chip_cb_dbg_show(struct seq_file *s, struct gpio_chip* chip)
{
  LOG_DEBUG("pilot_io_gpio_chip_cb_dbg_show() called");
}

// END gpiolib callback function implementations
//////////////////////////////////////////////////////////////////////

// *******************************************************************
// START pilot interface function implementation

static void pilot_io_callback_recv(module_slot_t slot, module_port_t port, spidata_t data)
{
  LOG_DEBUG("pilot_io_callback_recv(slot=%i, data=%i) called.", slot, data);
  //pilot_io_handle_recv(slot, data);
}

/* callback that gets called by the main driver, if a module slot is assigned to the driver */
static int pilot_io_callback_assign_slot(module_slot_t slot, const pilot_module_type_t *module_type)
{
  pilot_io_module_type_t io_type;

  LOG_DEBUG("pilot_io_callback_assign_slot(slot=%i, module_type) called", slot);

  // we got assigned to a slot, create the io device
  io_type = pilot_io_get_module_type(module_type);

  /* if the type is unknown, default to i8 */
  if (io_type == pilot_io_module_type_invalid) {
    io_type = pilot_io_module_type_i8;
  }

  switch (io_type)
  {
    case pilot_io_module_type_i8:
    case pilot_io_module_type_o8:
    case pilot_io_module_type_io16:
      pilot_io_register_gpio_module(slot, io_type); /* register the gpio module */
      break;

    case pilot_io_module_type_counter8:
      pilot_io_register_counter_module(slot); /* register the counter module */
      break;

    case pilot_io_module_type_ia8:
      pilot_io_register_ia8_module(slot);
      break;

    default: LOG(KERN_ERR, "trying to assign unknown module type: %i", io_type);
  }

  /* register the irqs for this module */
  pilot_io_irq_register(slot, io_type, &_internals.gpio_modules[(int)slot].gpio_chip);

  return io_type;
}

/* callback function that gets called by the main driver, if a module slot is unassigned from the driver */
static int pilot_io_callback_unassign_slot(module_slot_t slot)
{
  LOG_DEBUG("pilot_io_callback_unassign_slot(slot=%i) called", slot);

  /* remove the input proc entries, if any */
  pilot_io_proc_deinit_ia8_module(slot);

  /* remove the counter proc entries, if any */
  pilot_io_proc_deinit_counter_module(slot);

  /* destroy the gpio chip */
  pilot_io_unregister_gpio_module(slot);

  /* unregister the irq chip */
  pilot_io_irq_unregister(slot);
  
  return SUCCESS;
}

/* returns true if it is one of our supported modules */
static int pilot_io_callback_can_assign(const pilot_module_type_t *module_type)
{
  return (pilot_io_get_module_type(module_type) == pilot_io_module_type_invalid) ? -1 : SUCCESS;
}

/* callback function that gets called by the main driver when a command is received */
static pilot_cmd_handler_status_t pilot_io_callback_cmd_received(pilot_cmd_t cmd)
{
  gpio_module_t* gpio_module;
  counter_module_t* counter_module;
  int module_index, counter_index, input_index;
  u64 value;
  pilot_cmd_handler_status_t ret;

  LOG_DEBUG("pilot_io_callback_cmd_received() called");

  /* get the module index */
  module_index = (int)target_t_get_module_slot(cmd.target); 
  gpio_module = &_internals.gpio_modules[module_index];
  ret = pilot_cmd_handler_status_ignored;

  /* sanity check the module_index */
  if (module_index >= 0 && module_index < MODULES_COUNT) 
  {
    switch (cmd.type)
    {
      /* handle input counters messages */
      case pilot_cmd_type_input_get_counter:

        /* get the counter module */
        counter_module = &_internals.counter_modules[module_index];

        /* get the counter_index, it's in the first data byte */
        counter_index = cmd.data[(int)pilot_counter_index_target];

        /* update the counter value, it is encoded in the last 7 data bytes */
        value = (((u64)cmd.data[(int)pilot_counter_index_value + 0]) << (6*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 1]) << (5*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 2]) << (4*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 3]) << (3*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 4]) << (2*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 5]) << (1*8)) |
                (((u64)cmd.data[(int)pilot_counter_index_value + 6]) << (0*8));

        gpio_module->gpio_states[counter_index] = value;
        mb();
        gpio_module->is_state_updated[counter_index] = 1;

        LOG_DEBUG("updated module %i counter %i to value %llu", module_index, counter_index, value);
        ret = pilot_cmd_handler_status_handled; /* mark the command as handled */
        break;

      /* handle input message */
      case pilot_cmd_type_input_get_input:
        input_index = cmd.data[(int)pilot_input_index_target];
        //value = cmd.data[(int)pilot_input_index_value] ? 1 : 0;
        //value = cmd.data[pilot_cmd_t_data_size-1] ? 1 : 0;

        /* update the input value, it is encoded in the last 4 bytes */
        value = (((u64)cmd.data[(int)pilot_input_index_value + 0]) << (3*8)) |
                (((u64)cmd.data[(int)pilot_input_index_value + 1]) << (2*8)) |
                (((u64)cmd.data[(int)pilot_input_index_value + 2]) << (1*8)) |
                (((u64)cmd.data[(int)pilot_input_index_value + 3]) << (0*8));

        gpio_module->gpio_states[input_index] = value;
        mb();
        gpio_module->is_state_updated[input_index] = 1;
        ret = pilot_cmd_handler_status_handled;
        LOG_DEBUG("updated module %i input %i to value %llu", module_index, input_index, value);
        break;

      /* handle input changed message */
      case pilot_cmd_type_input_changed:
        /* generate irq about input change */
        pilot_io_irq_raise(module_index);
        ret = pilot_cmd_handler_status_handled;
        break;

      /* unknown message */
      default: break;
    }
  }

  return ret;
}

// END pilot interface function implementation
// *******************************************************************

/////////////////////////////////////////////////////////////////////
// START proc file system functions


#define pilot_io_proc_gpio_base_name "io_gpio_base"
#define pilot_io_proc_gpio_max_name "io_gpio_max"

static void pilot_io_proc_init()
{
  int module_index;
  struct proc_dir_entry *pilot_dir;

   /* get the base proc entry (/proc/pilot) from the base driver */
  _internals.proc_pilot_dir = pilot_dir = pilot_get_proc_pilot_dir();

  /* get the module proc entries (/proc/pilot/moduleX) from the base driver */
  for (module_index = 0; module_index < MODULES_COUNT; module_index++)
    _internals.proc_module_dir[module_index] = pilot_get_proc_module_dir((module_slot_t)module_index);

  /* register /proc/pilot/io_gpio_base */
  proc_create_data(pilot_io_proc_gpio_base_name, 0666, pilot_dir, &proc_pilot_io_gpio_base_fops, NULL);

  /* register /proc/pilotio_gpio_max */
  proc_create_data(pilot_io_proc_gpio_max_name, 0666, pilot_dir, &proc_pilot_io_gpio_max_fops, NULL);
}

static void pilot_io_proc_deinit()
{
  /* remove /proc/pilot/io_gpio_base */
  remove_proc_entry(pilot_io_proc_gpio_base_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/io_gpio_max */
  remove_proc_entry(pilot_io_proc_gpio_max_name, _internals.proc_pilot_dir);
}

static char* pilot_io_proc_counter_names[] = { "counter0", "counter1", "counter2", "counter3", "counter4", "counter5", "counter6", "counter7" };

/* initializes the /proc/ file system for the specified slot and module */
static void pilot_io_proc_init_counter_module(module_slot_t slot)
{
  int counter_index; 
  counter_module_t* counter_module;
  //struct proc_dir_entry* entry;

  LOG_DEBUG("pilot_io_proc_init_counter_module(slot=%i)", slot);

  counter_module = &_internals.counter_modules[(int)slot];

  /* register /proc/pilot/module[slot]/counter[0-8] */
  /* register a file foreach counter */
  for (counter_index = 0; counter_index < COUNTER_COUNT; counter_index++)
  {
    counter_module->proc_counters[counter_index] = 
      proc_create_data(pilot_io_proc_counter_names[counter_index],
                       0666,
                       _internals.proc_module_dir[(int)slot],
                       &proc_pilot_module_counter_fops,
                       (void*)((slot << 3) | counter_index));
  }
}

static void pilot_io_proc_deinit_counter_module(module_slot_t slot)
{
  int counter_index;
  struct proc_dir_entry* entry;
  counter_module_t* counter_module = &_internals.counter_modules[(int)slot];

  if (counter_module != NULL)
  {
    for (counter_index = 0; counter_index < COUNTER_COUNT; counter_index++)
    {
      entry = counter_module->proc_counters[counter_index];

      if (entry != NULL)
      {
        /* remove the counter entry */
        remove_proc_entry(pilot_io_proc_counter_names[counter_index],
                          _internals.proc_module_dir[(int)slot]);

        /* reset the proc_counter */
        counter_module->proc_counters[counter_index] = NULL;
      }
    }
  }
}

/* callback function that gets called when the content of the file /proc/pilot/moduleX/counterY is written */
static int pilot_io_proc_pilot_module_counter_write(struct file *file, const char *__user buf, size_t count, loff_t *off)
{
  int ret, data;
  u64 new_value;

  int module_index, counter_index;

  data = (int)PDE_DATA(file->f_inode);

  module_index = (int)data >> 3;
  counter_index = (int)data & 0x07;

  if (kstrtoull_from_user(buf, count, 10, &new_value) == SUCCESS)
  {
    pilot_io_set_counter_value(module_index, counter_index, new_value);
    ret = count;
  }
  else 
    ret = -EINVAL;

  return ret;
}

static char* pilot_io_proc_ia8_names[] = { "inputanalog0", "inputanalog1", "inputanalog2", "inputanalog3", "inputanalog4", "inputanalog5", "inputanalog6", "inputanalog7" };

static void pilot_io_proc_init_ia8_module(module_slot_t slot)
{
  int input_index;
  ia8_module_t* ia8_module;

  LOG_DEBUG("pilot_io_proc_init_counter_module(slot=%i)", slot);

  ia8_module = &_internals.ia8_modules[(int)slot];

  /* register /proc/pilot/module[slot]/inputanalog[0-8] */
  /* register a file foreach input */
  for (input_index = 0; input_index < IO_COUNT; input_index++)
  {
    ia8_module->proc_ia[input_index] = 
      proc_create_data(
        pilot_io_proc_ia8_names[input_index],
        0666,
        _internals.proc_module_dir[(int)slot],
        &proc_pilot_module_ia8_fops,
        (void*)((slot << 3) | input_index) /* encode the slot and input */
      );

    //  create_proc_entry(pilot_io_proc_ia8_names[input_index],    /* name */
    //                    0666,                                   /* writable file */
    //                    _internals.proc_module_dir[(int)slot]); /* parent dir */

    //entry->read_proc  = pilot_io_proc_ia8_callback_read;   /* file read callback */
    //entry->write_proc = NULL;  /* file write callback */
    //entry->data       = (void*)((slot << 3) | input_index); /* make the calls discernable for the read/write functions by examining the data parameter */
  }
}

static void pilot_io_proc_deinit_ia8_module(module_slot_t slot)
{
  int input_index;
  ia8_module_t *ia8_module;
  struct proc_dir_entry* entry;

  ia8_module = &_internals.ia8_modules[(int)slot];

  if (ia8_module != NULL)
  {
    for (input_index = 0; input_index < IO_COUNT; input_index++)
    {
      entry = ia8_module->proc_ia[input_index];

      if (entry != NULL)
      {
        /* remove the counter entry */
        remove_proc_entry(pilot_io_proc_ia8_names[input_index],
                          _internals.proc_module_dir[(int)slot]);

        /* reset the proc_counter */
        ia8_module->proc_ia[input_index] = NULL;
      }
    }
  }
}

static int pilot_io_proc_pilot_io_gpio_base_show(struct seq_file *file, void *data)
{
  seq_printf(file, "%i\n", _internals.gpio_base);
  return 0;
}

static int pilot_io_proc_pilot_io_gpio_base_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_io_proc_pilot_io_gpio_base_show, PDE_DATA(inode));
}

static int pilot_io_proc_pilot_io_gpio_base_write(struct file* file, const char* __user buf, size_t count, loff_t* off)
{
  int ret, new_value;

  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL;
  else if (new_value >= 0 && new_value <= _internals.gpio_max)
  {
    LOG_DEBUG("changing gpio_base from %i to %i", _internals.gpio_base, new_value);
    _internals.gpio_base = new_value;
    ret = count;
  }
  else
    ret = -EINVAL;

  return ret;
}

static const struct file_operations proc_pilot_io_gpio_base_fops = {
  .open = pilot_io_proc_pilot_io_gpio_base_open,
  .llseek = seq_lseek,
  .read = seq_read,
  .release = single_release,
  .write = pilot_io_proc_pilot_io_gpio_base_write
};

/* max gpio specific  */

static int pilot_io_proc_pilot_io_gpio_max_show(struct seq_file *file, void *data)
{
  seq_printf(file, "%i\n", _internals.gpio_max);
  return 0;
}

static int pilot_io_proc_pilot_io_gpio_max_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_io_proc_pilot_io_gpio_max_show, NULL);
}

static const struct file_operations proc_pilot_io_gpio_max_fops = {
  .open = pilot_io_proc_pilot_io_gpio_max_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release
};

/* max gpio specific */

static int pilot_io_proc_pilot_module_counter_show(struct seq_file *file, void *data)
{
  int module_index, counter_index, ret;
  counter_module_t *counter_module;
  gpio_module_t *gpio_module;

  module_index = (int)file->private >> 3;
  counter_index = (int)file->private & 0x7;

  if (pilot_io_try_get_counter(module_index, counter_index, _internals.answer_timeout) == -1) /* try to update the counter before we write it to the user */
    ret = -EFAULT; /* return an error if getting the counter fails */
  else
  {
    counter_module = &_internals.counter_modules[module_index];
    gpio_module = &_internals.gpio_modules[module_index];

    seq_printf(file, "%llu", gpio_module->gpio_states[counter_index]);
    ret = 0;
  }

  return ret;
}

static int pilot_io_proc_pilot_module_counter_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_io_proc_pilot_module_counter_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_counter_fops = {
  .open = pilot_io_proc_pilot_module_counter_open,
  .llseek = seq_lseek,
  .read = seq_read,
  .release = single_release,
  .write = pilot_io_proc_pilot_module_counter_write
};

static int pilot_io_proc_pilot_module_ia8_show(struct seq_file *file, void *data)
{
  /* get the module index and counter index from the data */
  int module_index, input_index, ret;
  ia8_module_t *ia8_module;
  gpio_module_t *gpio_module;

  module_index = (int)file->private >> 3;
  input_index = (int)file->private & 0x7;

  LOG_DEBUG("pilot_io_proc_pilot_module_ia8_show(module_index=%i, input_index=%i) called", module_index, input_index);

  if (pilot_io_try_get_ia(module_index, input_index, _internals.answer_timeout) == -1) /* try to update the counter before we write it to the user */
    ret = -EFAULT; /* return an error if getting the counter fails */
  else
  {
    ia8_module = &_internals.ia8_modules[module_index];
    gpio_module = &_internals.gpio_modules[module_index];
    seq_printf(file, "%llu", gpio_module->gpio_states[input_index]);
    ret = 0;
  }

  return ret;
}

static int pilot_io_proc_pilot_module_ia8_open(struct inode *inode, struct file *file)
{
  LOG_DEBUG("pilot_io_proc_pilot_module_ia8_open() data=%i", (int)PDE_DATA(inode));
  return single_open(file, pilot_io_proc_pilot_module_ia8_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_ia8_fops = {
  .open = pilot_io_proc_pilot_module_ia8_open,
  .llseek = seq_lseek,
  .read = seq_read,
  .release = single_release
};

// END proc file system functions
/////////////////////////////////////////////////////////////////////
