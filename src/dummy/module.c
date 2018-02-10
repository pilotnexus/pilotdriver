#include <linux/kernel.h>    // needed for KERN_INFO
#include "module.h"          // include defines that describe the module
#include "../driver/export.h"

#define MODULE_NAME "pilotdummy"
#include "../driver/common.h"

// START forward declaration
static int  __init dummy_init(void); /* kernel module entry function */
static void __exit dummy_exit(void); /* kernel module exit function */

static int  dummy_pilot_register(void);    /* registers with the rpcp base driver */
static void dummy_pilot_unregister(void); /* unregisters with the rpcp base driver */

static void dummy_pilot_callback_recv         (module_slot_t slot, module_port_t port, spidata_t rx); /* callback function when data arrives */
static int  dummy_pilot_callback_assign_slot  (module_slot_t slot, const pilot_module_type_t *module_type); /* callback function when a slot is assigned to the driver */
static int  dummy_pilot_callback_unassign_slot(module_slot_t slot);               /* callback function when a slot is unassigned from the driver */

//static void dummy_send_init(int module); /* dummy function that sends some data to the module */

// END forward declaration

typedef struct {
  int driverId;
} internals_t;

static internals_t internals = { -1 };

/* description of our dummy module driver */
static register_driver_t dummy_register_driver = {
  .name                   = "dummy",
  .callback_assign_slot   = dummy_pilot_callback_assign_slot,
  .callback_unassign_slot = dummy_pilot_callback_unassign_slot,
  .callback_recv          = dummy_pilot_callback_recv
};

// main entry point
module_init(dummy_init);

// main exit point
module_exit(dummy_exit);

/* dummy module initialization function, called when inserting the module */
static int dummy_init(void)
{
  int ret = SUCCESS;

  LOG_DEBUG("dummy_init() called");
  
  // register at the pilot driver
  if (dummy_pilot_register() != SUCCESS)
  {
    LOG(KERN_ERR, "error calling dummy_register()");
    ret = -1;
  }

  return ret;
}

/* dummy module cleanup function, called when removing the module */
static void dummy_exit(void)
{
  LOG_DEBUG("dummy_exit() called");
  dummy_pilot_unregister();
  LOG_DEBUG("Goodbye!");
}

/* dummy register function, tries to register with the main driver */
static int dummy_pilot_register()
{
  int id = -1, ret = SUCCESS;
  LOG_DEBUG("dummy_register called");

  id = pilot_register_driver(&dummy_register_driver);

  if (id < 0) {
    LOG(KERN_ERR, "pilot_ext_register_driver() failed with: %i", id);
    ret = -1;
  }
  else
  {
    LOG_DEBUG("pilot_ext_register_driver() successful. id = %i", id);
    internals.driverId = id;
  }
  
  return ret;
}

/* dummy unregister funtion that unregisters from the main driver */
static void dummy_pilot_unregister(void)
{
  LOG_DEBUG("dummy_unregister() called");

  if (internals.driverId >= 0)
  {
    LOG_DEBUG("unregistering id %i", internals.driverId);
    pilot_unregister_driver(internals.driverId);
  }

  //if (registered != 0) {
  //  LOG_DEBUG("unregistering %i\n", registered);
  //  pilot_ext_unregister(registered);
  //}
}

/* function sends initialization to the module */
//static void dummy_send_init(int module) 
//{
//  char data[] = { 1, 2, 3, 3, 2, 1 };
//  LOG_DEBUG( "sending init data to microcontroller" );
//  pilot_send(module, data, 6);
//}

// ************************************************************
// callback functions 

/* callback function that gets called when data arrives for the registered module */
static void dummy_pilot_callback_recv(module_slot_t slot, module_port_t port, spidata_t rx)
{
  LOG_DEBUG("dummy_recv_callback(slot=%i, rx=%X)", rx); 
}

static int dummy_pilot_callback_assign_slot(module_slot_t slot, const pilot_module_type_t *module_type)
{
  LOG_DEBUG("dummy_pilot_callback_assign_slot() called with slot=%i", slot);

  return SUCCESS;
}

static int dummy_pilot_callback_unassign_slot(module_slot_t slot)
{
  LOG_DEBUG("dummy_pilot_callback_unassign_slot() called with slot=%i", slot);
  return SUCCESS;
}