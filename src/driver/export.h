/* this file contains the definition of exported symbols, used by other kernel modules */

#ifndef __EXPORT_H__
#define __EXPORT_H__

#include <linux/list.h>
#include "common_base.h"

/* register_driver_t is used for registering a module driver when call pilot_ext_register_driver() */
typedef struct {
  char* name; /* name of the driver */
  void (*callback_recv)(module_slot_t slot, module_port_t port, spidata_t); /* callback function that gets called when data arrives for this driver */
  int (*callback_assign_slot)(module_slot_t slot, const pilot_module_type_t *module_type); /* callback function that gets called when a slot is assigned. */
  int (*callback_unassign_slot)(module_slot_t slot); /* callback function that gets called when a slot is unassigned. return 0 to indicate SUCCESS. */  
  int (*callback_can_assign)(const pilot_module_type_t *arg); /* callback function that gets called when a slot should be assigned, but the argument does not specify the id/driver name. */
} register_driver_t;

typedef enum {
  pilot_cmd_handler_status_ignored, /* the command handler ignored the command */
  pilot_cmd_handler_status_handled  /* the command handler handled the command */
} pilot_cmd_handler_status_t;

/* rpcp registered command handler - is called back when a command is received */
typedef struct {
  pilot_cmd_handler_status_t (*callback_cmd_received)(pilot_cmd_t cmd); /* the method that will be called when a command is received */
  struct list_head list;
} pilot_cmd_handler_t;

#ifdef RPC_MC_DRIVER
extern 
#endif
  /* description: registers the driver */
  /* parameters:  driver: the driver to register */
  /* returns:     assigned driver id on success, otherwise negative */
  int pilot_register_driver (register_driver_t* driver);

#ifdef RPC_MC_DRIVER
extern 
#endif
  /* description: unregisters the module driver with the specified id */
  /* parameters:  driverId - the id returned when calling pilot_unregister_driver */
  void pilot_unregister_driver (int driverId);

#ifdef RPC_MC_DRIVER
extern
#endif
  /* description: sends the data to the module */
  /* parameters:  target: target module to send the data to */
  /*              data: pointer to the data to send */
  /*              count: length of data elements to send */
  /* returns:     void */
  void pilot_send (target_t target, const char* data, int count);

#ifdef RPC_MC_DRIVER
extern
#endif
  /* description: sends the supplied command to the stm */
  /* parameters:  cmd: the command to send */
  void pilot_send_cmd(pilot_cmd_t* cmd);

#ifdef RPC_MC_DRIVER
extern
#endif
  /* description: gets the number of remaining free bytes of the send buffer for the specified target */
  int pilot_get_free_send_buffer_size(target_t module);

#ifdef RPC_MC_DRIVER
extern
#endif
  stm_bufferstate_t pilot_get_stm_bufferstate(void);

#ifdef RPC_MC_DRIVER
extern
#endif
  int pilot_register_cmd_handler(pilot_cmd_handler_t* cmd_handler);

#ifdef RPC_MC_DRIVER
extern
#endif
  int pilot_unregister_cmd_handler(pilot_cmd_handler_t* cmd_handler);

#ifdef RPC_MC_DRIVER
extern
#endif
  struct proc_dir_entry* pilot_get_proc_module_dir(module_slot_t slot);

#ifdef RPC_MC_DRIVER
extern
#endif
  struct proc_dir_entry* pilot_get_proc_pilot_dir(void);

#ifdef RPC_MC_DRIVER
extern
#endif
  void pilot_auto_configure(void);

#ifdef RPC_MC_DRIVER
extern
#endif
  void pilot_register_stream_handler(target_t target, void (*stream_callback)(char data));

#ifdef RPC_MC_DRIVER
extern
#endif
  void pilot_unregister_stream_handler(target_t target);

#endif