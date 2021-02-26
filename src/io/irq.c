#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include "irq.h"

struct internals_t {  
  struct gpio_chip *gpio_chips[MODULES_COUNT];
  struct gpio_chip *gpio_chip_irq;
};

/* holds all internal state */
static struct internals_t _internals = { };
static void pilot_io_irq_tasklet_handler(struct tasklet_struct *unused);
static DECLARE_TASKLET(pilot_io_irq_tasklet, pilot_io_irq_tasklet_handler);

static void pilot_io_irq_tasklet_handler(struct tasklet_struct *unused)
{
  int i;
  struct gpio_chip *gpio_chip = _internals.gpio_chip_irq;

  LOG_DEBUG("pilot_io_irq_tasklet_handler() called");

  if (gpio_chip != NULL)
  {
    /* raise the irqs for all gpios of chip */
    for (i = 0; i < gpio_chip->ngpio; i++)
      generic_handle_irq(gpio_to_irq(gpio_chip->base + i));

     _internals.gpio_chip_irq = NULL;
  }
}

/* registers the irq chip for the specified module */
void pilot_io_irq_register(module_slot_t slot, pilot_io_module_type_t type, struct gpio_chip *gpio_chip)
{
  _internals.gpio_chips[(int)slot] = gpio_chip;
}

/* unregisters the irq chip for the specified module */
void pilot_io_irq_unregister(module_slot_t slot)
{
  _internals.gpio_chips[(int)slot] = NULL;
}

/* raise the input changed irq for the specified module */
void pilot_io_irq_raise(module_slot_t slot)
{
  struct gpio_chip *gpio_chip;

  LOG_DEBUG("pilot_io_irq_raise(slot=%i) called", slot);

  gpio_chip = _internals.gpio_chips[(int)slot];

  /* raise the irq */
  if (gpio_chip != NULL &&
      _internals.gpio_chip_irq == NULL)
  {    
    _internals.gpio_chip_irq = gpio_chip;
    tasklet_schedule(&pilot_io_irq_tasklet);
  }
}
