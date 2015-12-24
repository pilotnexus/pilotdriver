#ifndef __TTY_H__
#define __TTY_H__

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include "../driver/export.h"

int  pilot_tty_register_driver    (void);
void pilot_tty_unregister_driver  (void);
void pilot_tty_register_device    (module_slot_t slot, module_port_t port);
void pilot_tty_unregister_device  (module_slot_t slot, module_port_t port);
void pilot_tty_received_data      (module_slot_t slot, module_port_t, uint8_t data);
//void pilot_tty_bufferstate_changed(module_slot_t slot, module_port_t port, stm_bufferstate_t bufferstate);

#endif
