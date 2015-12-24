#ifndef __IRQ_H__
#define __IRQ_H__
#include "common.h"
#include "types.h"

void pilot_io_irq_register(module_slot_t slot, pilot_io_module_type_t type, struct gpio_chip *gpio_chip);
void pilot_io_irq_unregister(module_slot_t slot);
void pilot_io_irq_raise(module_slot_t slot);

#endif
