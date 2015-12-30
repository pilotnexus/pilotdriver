/* defines information used for linux kernal management */
/* main pilot kernel module for microcontroller communication */
/* written by mdk 15.03.2013 */

#ifndef __MODULE_H__
#define __MODULE_H__

#include <linux/module.h>

#define RPC_MC_DRIVER
#include "export.h" // exported symbols containing pilot_ext_XXX()

MODULE_LICENSE    ("GPL");
MODULE_AUTHOR     ("mdk");
MODULE_DESCRIPTION("Pilot kernel module base driver");

/* exports the listed symbol into kernel space, so that other modules can call them */
EXPORT_SYMBOL(pilot_register_driver);
EXPORT_SYMBOL(pilot_unregister_driver);
EXPORT_SYMBOL(pilot_send);
EXPORT_SYMBOL(pilot_send_cmd);
EXPORT_SYMBOL(pilot_get_free_send_buffer_size);
EXPORT_SYMBOL(pilot_register_cmd_handler);
EXPORT_SYMBOL(pilot_unregister_cmd_handler);
EXPORT_SYMBOL(pilot_get_proc_module_dir);
EXPORT_SYMBOL(pilot_get_proc_pilot_dir);
EXPORT_SYMBOL(pilot_get_stm_bufferstate);
EXPORT_SYMBOL(pilot_auto_configure);
EXPORT_SYMBOL(pilot_register_stream_handler);
EXPORT_SYMBOL(pilot_unregister_stream_handler);

#endif
