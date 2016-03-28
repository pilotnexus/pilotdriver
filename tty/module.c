#include <asm/delay.h>        /* needed for udelay() function */
#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */
#include <linux/seq_file.h>   /* needed for seq_file struct and functions */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h"
#include "tty.h"
#include "common.h"

MODULE_LICENSE("GPL");

// *******************************************************************
// START forward declaration
static int  __init pilot_tty_init(void); /* kernel module entry function */
static void __exit pilot_tty_exit(void); /* kernel module exit function */

typedef enum {
  pilot_tty_module_type_invalid,
  pilot_tty_module_type_gsm,
  pilot_tty_module_type_gps,
  pilot_tty_module_type_can,
  pilot_tty_module_type_can_2p,
  pilot_tty_module_type_rs485,
  pilot_tty_module_type_rs485_2p,
  pilot_tty_module_type_rs485_4w,
  pilot_tty_module_type_rs232,
  pilot_tty_module_type_rs232_2p,
  pilot_tty_module_type_ow,
  pilot_tty_module_type_ow_2p,
  pilot_tty_module_type_pro
} pilot_tty_module_type_t;

static pilot_tty_module_type_t pilot_tty_get_module_type        (const pilot_module_type_t *module_type);
static void                   pilot_tty_callback_recv          (module_slot_t slot, module_port_t port, spidata_t rx); /* callback function when data arrives */
static int                    pilot_tty_callback_assign_slot   (module_slot_t slot, const pilot_module_type_t *module_type);    /* callback function when a slot is assigned to the driver */
static int                    pilot_tty_callback_unassign_slot (module_slot_t slot);               /* callback function when a slot is unassigned from the driver */
static int                    pilot_tty_callback_can_assign    (const pilot_module_type_t *module_type); /* callback function when a slot is assigned but the argument used does not specify the driver */

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd);

static void pilot_tty_proc_init  (module_slot_t slot, pilot_tty_module_type_t type);
static void pilot_tty_proc_deinit(module_slot_t slot, pilot_tty_module_type_t type);

static void pilot_tty_proc_gps_init (module_slot_t slot);
static void pilot_tty_proc_gps_deinit(module_slot_t slot);

// END forward declaration
// *******************************************************************

// *******************************************************************
// START local members
/* description of our tty module driver */
static register_driver_t register_driver = {
  .name                          = "tty",
  .callback_assign_slot          = pilot_tty_callback_assign_slot,
  .callback_unassign_slot        = pilot_tty_callback_unassign_slot,
  .callback_recv                 = pilot_tty_callback_recv,
  .callback_can_assign           = pilot_tty_callback_can_assign
};

/* rpcp struct with callback functions for the main rpcp driver */
static pilot_cmd_handler_t pilot_cmd_handler = {
  .callback_cmd_received = pilot_callback_cmd_received
};

typedef struct {
  int driverId;
  int is_cmd_handler_registered;
  pilot_tty_module_type_t module_types[MODULES_COUNT];
  volatile int enable_gps_updated[MODULES_COUNT];
  volatile int enable_gps_value[MODULES_COUNT];
  volatile int enable_onewire_updated[MODULES_COUNT];
  volatile int enable_onewire_value[MODULES_COUNT];
  volatile int enable_gsm_updated[MODULES_COUNT];
  volatile int enable_gsm_value[MODULES_COUNT];
  int answer_timeout;
} internals_t;

/* internal variables */
static internals_t _internals = { 
  .driverId =-1, 
  .module_types = { pilot_tty_module_type_invalid, pilot_tty_module_type_invalid, pilot_tty_module_type_invalid, pilot_tty_module_type_invalid },
  .answer_timeout = 300
};

static const struct file_operations proc_pilot_module_enable_gps_fops,
                                    proc_pilot_module_enable_onewire_fops,
                                    proc_pilot_module_enable_gsm_fops;

// END local members
// *******************************************************************

// main entry point
module_init(pilot_tty_init);

// main exit point
module_exit(pilot_tty_exit);

/* initialization routine, called when the module is loaded */
static int __init pilot_tty_init()
{
  int ret = -1;
  
  // register with the base driver
  if ((_internals.driverId = pilot_register_driver(&register_driver)) < 0)
  {
    LOG(KERN_ERR, "pilot_register_driver() failed with %i", _internals.driverId);
  }
  else
  {
    if ((ret = pilot_tty_register_driver()) >= 0)
    {
      /* register the cmd_handler */
      if (pilot_register_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      {
        LOG_DEBUG("pilot_register_cmd_handler() succeeded");
        _internals.is_cmd_handler_registered = 1;
        ret = SUCCESS;

        /* request an autoconfiguration of all modules */
        pilot_auto_configure();
      }
    }
  }

  return ret;
}

/* tty module cleanup function, called when removing the module */
static void __exit pilot_tty_exit()
{
  // unregister with the base driver
  if (_internals.driverId >= 0)
    pilot_unregister_driver( _internals.driverId );

  // unregister the tty driver
  pilot_tty_unregister_driver();

  /* unregister with the base driver */
  if (_internals.is_cmd_handler_registered) {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      _internals.is_cmd_handler_registered = 0;
  }
}

static const char ow[]       = "ow";
static const char ow_2p[]    = "ow_2p";
static const char can[]      = "can";
static const char can_2p[]   = "can_2p";
static const char gps[]      = "gps";
static const char gsm[]      = "gsm_2g";
static const char rs485[]    = "r485";
static const char rs485_2p[] = "r485_2p";
static const char rs485_4w[] = "r485_4w";
static const char rs232[]    = "r232";
static const char rs232_2p[] = "r232_2p";
static const char pro[]      = "pro";

#define IS_MODULE_TYPE(m, n) (strncmp(n, m->name, strlen(n)) == 0)

static pilot_tty_module_type_t pilot_tty_get_module_type(const pilot_module_type_t *module_type)
{
  pilot_tty_module_type_t tty_type;

  if (IS_MODULE_TYPE(module_type, gsm))
    tty_type = pilot_tty_module_type_gsm;
  else if (IS_MODULE_TYPE(module_type, pro))
    tty_type = pilot_tty_module_type_pro;
  else if (IS_MODULE_TYPE(module_type, gps))
    tty_type = pilot_tty_module_type_gps;
  else if (IS_MODULE_TYPE(module_type, can_2p))
    tty_type = pilot_tty_module_type_can_2p;
  else if (IS_MODULE_TYPE(module_type, can))
    tty_type = pilot_tty_module_type_can;
  else if (IS_MODULE_TYPE(module_type, rs485_2p))
    tty_type = pilot_tty_module_type_rs485_2p;
  else if (IS_MODULE_TYPE(module_type, rs485_4w))
    tty_type = pilot_tty_module_type_rs485_4w;
  else if (IS_MODULE_TYPE(module_type, rs485))
    tty_type = pilot_tty_module_type_rs485;
  else if (IS_MODULE_TYPE(module_type, rs232_2p))
    tty_type = pilot_tty_module_type_rs232_2p;
  else if (IS_MODULE_TYPE(module_type, rs232))
    tty_type = pilot_tty_module_type_rs232;
  else if (IS_MODULE_TYPE(module_type, ow_2p))
    tty_type = pilot_tty_module_type_ow_2p;
  else if (IS_MODULE_TYPE(module_type, ow))
    tty_type = pilot_tty_module_type_ow;
  else
    tty_type = pilot_tty_module_type_invalid;

  return tty_type;
}

// *******************************************************************
// START pilot interface function implementation

static void pilot_tty_callback_recv(module_slot_t slot, module_port_t port, spidata_t data)
{
  //LOG_DEBUG("pilot_tty_callback_recv(slot=%i, data=%x)", slot, data);

  /* forward the received data to the tty port */
  pilot_tty_received_data(slot, port, (uint8_t)data);
}

static int pilot_tty_get_ports(pilot_tty_module_type_t tty_type)
{
  int ports;
  switch (tty_type)
  {
    case pilot_tty_module_type_can_2p:
    case pilot_tty_module_type_ow_2p:
    case pilot_tty_module_type_rs485_2p:
    case pilot_tty_module_type_rs232_2p:
      ports = 2;
      break;

    default: ports = 1; break;
  }
  return ports;
}

static int pilot_tty_callback_assign_slot(module_slot_t slot, const pilot_module_type_t *module_type)
{
  pilot_tty_module_type_t tty_type;
  int ports;

  /* identify other module types by parsing arg */
  tty_type = pilot_tty_get_module_type(module_type);

  /* store the module type */
  _internals.module_types[(int)slot] = tty_type;

  /* get the ports */
  ports = pilot_tty_get_ports(tty_type);

  // we got assigned to a slot, create the tty device
  pilot_tty_register_device(slot, module_port_1);

  /* if the device has 2 ports, create a second device */
  if (ports == 2)
    pilot_tty_register_device(slot, module_port_2);

  /* initialize it's proc files if any */
  pilot_tty_proc_init(slot, tty_type);
  
  return tty_type;
}

static int pilot_tty_callback_unassign_slot(module_slot_t slot)
{
  pilot_tty_module_type_t type = _internals.module_types[(int)slot];

  /* destroy the proc devices, if any */
  pilot_tty_proc_deinit(slot, type);

  // we got unassigned - destroy the tty device
  pilot_tty_unregister_device(slot, module_port_1);
  pilot_tty_unregister_device(slot, module_port_2);

  /* reset the module type */
  _internals.module_types[(int)slot] = pilot_tty_module_type_invalid;

  return SUCCESS;
}

static int pilot_tty_callback_can_assign(const pilot_module_type_t *module_type)
{
  return (pilot_tty_get_module_type(module_type) == pilot_tty_module_type_invalid) ? -1 : SUCCESS;
}

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  //int i,j; //stm_bufferstate_t bufferstate;
  pilot_cmd_handler_status_t ret;
  module_slot_t slot;

  slot = target_t_get_module_slot(cmd.target);

  LOG_DEBUG("pilot_callback_cmd_received() called");

  switch (cmd.type)
  {
    /* todo is wakeup necessary? */
    //case pilot_cmd_type_bufferstate: break;
    //case pilot_cmd_type_usart_bufferstate:
      /* update the bufferstate of all modules/ports */
      //for (i = 0; i < MODULES_COUNT; i++) {
      //  for (j = 0; j <MODULE_PORT_COUNT; j++) {
      //    /* get the bufferstate */
      //    bufferstate = (stm_bufferstate_t)cmd.data[i*MODULE_PORT_COUNT + j];
      //    pilot_tty_bufferstate_changed(i, j, bufferstate);
      //  }
      //}
      //ret = pilot_cmd_handler_status_handled;
      //break;

    case pilot_cmd_type_onewire_get_enable:
      _internals.enable_onewire_value[(int)slot] = cmd.data[(int)pilot_onewire_enable_index_value];
      mb();
      _internals.enable_onewire_updated[(int)slot] = 1;
      ret = pilot_cmd_handler_status_handled;
      break;

    case pilot_cmd_type_gps_get_enable:
      _internals.enable_gps_value[(int)slot] = cmd.data[(int)pilot_gps_enable_index_value];
      mb();
      _internals.enable_gps_updated[(int)slot] = 1;
      ret = pilot_cmd_handler_status_handled;
      break;

    case pilot_cmd_type_gsm_get_enable:
      _internals.enable_gsm_value[(int)slot] = cmd.data[(int)pilot_gsm_enable_index_value];
      mb();
      _internals.enable_gsm_updated[(int)slot] = 1;
      ret = pilot_cmd_handler_status_handled;
      break;
    
    default: ret = pilot_cmd_handler_status_ignored; break;
  }

  return ret;
}

// END pilot interface function implementation
// *******************************************************************

///////////////////////////////
/* START gps proc functions */

#define module_slot_t_from_proc_data(data) ((module_slot_t)data)
#define module_slot_t_to_proc_data(slot) ((void*)slot)

static void pilot_tty_send_gps_set_enable(module_slot_t slot, int enable)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_gps_set_enable;
  cmd.data[(int)pilot_gps_enable_index_value] = enable ? 1 : 0;
  pilot_send_cmd(&cmd);
}

static void pilot_tty_send_gps_get_enable(module_slot_t slot)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_gps_get_enable;
  pilot_send_cmd(&cmd);
}

static int pilot_tty_gps_get_enable(module_slot_t slot, int timeout, int* enabled)
{
  int timedout;
  unsigned long timestamp;

  timedout = 0;

  /* reset the updated state */
  _internals.enable_gps_updated[(int)slot] = 0;

  /* send a request for the gps enable flag */
  pilot_tty_send_gps_get_enable((int)slot);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!_internals.enable_gps_updated[(int)slot])
  {
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
    udelay(100);
  }

  if (!timedout)
    *enabled = _internals.enable_gps_value[(int)slot];

  return timedout ? 0 : 1;
}

static char* pilot_tty_enable_gps_proc_name = { "enable_gps" };

static void pilot_tty_proc_gps_init(module_slot_t slot)
{
  struct proc_dir_entry *module_dir;

  /* retrieve the parent module dir from the base driver */
  module_dir = pilot_get_proc_module_dir(slot);

  /* create the /proc/pilot/moduleX/enable_gps entry */
  proc_create_data(pilot_tty_enable_gps_proc_name, 0666, module_dir, &proc_pilot_module_enable_gps_fops, module_slot_t_to_proc_data(slot));
}

static void pilot_tty_proc_gps_deinit(module_slot_t slot)
{
  remove_proc_entry(pilot_tty_enable_gps_proc_name, pilot_get_proc_module_dir(slot));
}

/* END gps proc functions */
///////////////////////////

static void pilot_tty_send_gsm_set_enable(module_slot_t slot, int enable)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_gsm_set_enable;
  cmd.data[(int)pilot_gsm_enable_index_value] = enable ? 1 : 0;
  pilot_send_cmd(&cmd);
}

static void pilot_tty_send_gsm_get_enable(module_slot_t slot)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_gsm_get_enable;
  pilot_send_cmd(&cmd);
}

static int pilot_tty_gsm_get_enable(module_slot_t slot, int timeout, int* enabled)
{
  int timedout;
  unsigned long timestamp;

  timedout = 0;

  /* reset the updated state */
  _internals.enable_gsm_updated[(int)slot] = 0;

  /* send a request for the gsm enable flag */
  pilot_tty_send_gsm_get_enable((int)slot);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!_internals.enable_gsm_updated[(int)slot])
  {
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
    udelay(100);
  }

  if (!timedout)
    *enabled = _internals.enable_gsm_value[(int)slot];

  return timedout ? 0 : 1;
}

static char *pilot_tty_enable_gsm_proc_name = { "enable_gsm" };

static void pilot_tty_proc_gsm_init(module_slot_t slot)
{
  struct proc_dir_entry *module_dir;

  /* retrieve the parent module dir from the base driver */
  module_dir = pilot_get_proc_module_dir(slot);

  /* create the /proc/pilot/moduleX/enable_gsm entry */
  proc_create_data(pilot_tty_enable_gsm_proc_name, 0666, module_dir, &proc_pilot_module_enable_gsm_fops, module_slot_t_to_proc_data(slot));
}

static void pilot_tty_proc_gsm_deinit(module_slot_t slot)
{
  remove_proc_entry(pilot_tty_enable_gsm_proc_name, pilot_get_proc_module_dir(slot));
}

////////////////////////////
/* START 1-wire proc functions */

static void pilot_tty_send_onewire_set_enable(module_slot_t slot, int enable)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_onewire_set_enable;
  cmd.data[(int)pilot_onewire_enable_index_value] = enable ? 1 : 0;
  pilot_send_cmd(&cmd);
}

static void pilot_tty_send_onewire_get_enable(module_slot_t slot)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_onewire_get_enable;
  pilot_send_cmd(&cmd);
}

static int pilot_tty_onewire_get_enable(module_slot_t slot, int timeout, int *enabled)
{
  int timedout;
  unsigned long timestamp;
  timedout = 0;

  /* reset the updated state */
  _internals.enable_onewire_updated[(int)slot] = 0;

  /* send a request for the 1-wire enable flag */
  pilot_tty_send_onewire_get_enable((int)slot);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);
  
  /* wait for the onewire enable to be updated */
  while(!_internals.enable_onewire_updated[(int)slot])
  {
    /* check if the timeout is reached */
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }
    udelay(100);
  }

  if (!timedout)
    *enabled = _internals.enable_onewire_value[(int)slot];

  return timedout ? 0 : 1;
}

static char* pilot_tty_enable_onewire_proc_name = { "enable_onewire" };

static void pilot_tty_proc_onewire_init(module_slot_t slot)
{
  struct proc_dir_entry *module_dir;

  /* retrieve the parent module dir from the base driver */
  module_dir = pilot_get_proc_module_dir(slot);

  /* register a proc file for enabling / disabling onewire (/proc/pilot/module[slot]/enable_onewire) */
  proc_create_data(pilot_tty_enable_onewire_proc_name, 0666, module_dir, &proc_pilot_module_enable_onewire_fops, module_slot_t_to_proc_data(slot));
}

static void pilot_tty_proc_onewire_deinit(module_slot_t slot)
{
  /* remove the enable_onewire file */
  remove_proc_entry(pilot_tty_enable_onewire_proc_name, pilot_get_proc_module_dir(slot));
}

/* END 1-wire proc functions */

static void pilot_tty_proc_init(module_slot_t slot, pilot_tty_module_type_t type)
{
  switch (type)
  {
    case pilot_tty_module_type_gps: pilot_tty_proc_gps_init(slot); break;
    case pilot_tty_module_type_gsm: pilot_tty_proc_gsm_init(slot); break;
    case pilot_tty_module_type_ow:
    case pilot_tty_module_type_ow_2p:
      pilot_tty_proc_onewire_init(slot);
      break;
    default: break;
  }
}

static void pilot_tty_proc_deinit(module_slot_t slot, pilot_tty_module_type_t type)
{
  switch (type)
  {
    case pilot_tty_module_type_gps: pilot_tty_proc_gps_deinit(slot); break;
    case pilot_tty_module_type_gsm: pilot_tty_proc_gsm_deinit(slot); break;
    case pilot_tty_module_type_ow:
    case pilot_tty_module_type_ow_2p:
      pilot_tty_proc_onewire_deinit(slot);
      break;
    default: break;
  }
}

/* enable_gps file operations */

static int pilot_tty_proc_pilot_module_enable_gps_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, data;

  data = (int)PDE_DATA(file->f_inode);

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    return -EINVAL; /* return an error if the conversion fails */
  else
  {
    /* send a gps enable cmd to the pilot */
    pilot_tty_send_gps_set_enable(module_slot_t_from_proc_data(data), new_value);
    return count; /* return that we successfully wrote the supplied bytes */
  }
}

static int pilot_tty_proc_pilot_module_enable_gps_show(struct seq_file *file, void *data)
{
  int enabled, ret, module_index = (int)file->private;

  if (pilot_tty_gps_get_enable(module_index, _internals.answer_timeout, &enabled)) /* try to get the enable state of the gps module */
  {
    seq_printf(file, "%i", enabled);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_tty_proc_pilot_module_enable_gps_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_tty_proc_pilot_module_enable_gps_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_enable_gps_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_tty_proc_pilot_module_enable_gps_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_tty_proc_pilot_module_enable_gps_write
};

/* enable_gsm file operations */

static int pilot_tty_proc_pilot_module_enable_gsm_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, data, ret;
  data = (int)PDE_DATA(file->f_inode);

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL;
  else
  {
    pilot_tty_send_gsm_set_enable(module_slot_t_from_proc_data(data), new_value);
    ret = count;
  }

  return ret;
}

static int pilot_tty_proc_pilot_module_enable_gsm_show(struct seq_file *file, void *data)
{
  int enabled, ret, module_index = (int)file->private;

  if (pilot_tty_gsm_get_enable(module_index, _internals.answer_timeout, &enabled))
  {
    seq_printf(file, "%i", enabled);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_tty_proc_pilot_module_enable_gsm_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_tty_proc_pilot_module_enable_gsm_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_enable_gsm_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_tty_proc_pilot_module_enable_gsm_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_tty_proc_pilot_module_enable_gsm_write
};

/* enable_onewire file operations */

static int pilot_tty_proc_pilot_module_enable_onewire_show(struct seq_file *file, void *data)
{
  int ret, enabled, module_index = (int)file->private;

  if (pilot_tty_onewire_get_enable(module_index, _internals.answer_timeout, &enabled))
  {
    seq_printf(file, "%i", enabled);
    ret = 0;
  }
  else
    ret = -EFAULT;
  return ret;
}

static int pilot_tty_proc_pilot_module_enable_onewire_write(struct file* file, const char* __user buf, size_t count, loff_t *off)
{
  int new_value, data;

  data = (int)PDE_DATA(file->f_inode);

  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    return -EINVAL;
  else
  {
    pilot_tty_send_onewire_set_enable(module_slot_t_from_proc_data(data), new_value);
    return count;
  }
}

static int pilot_tty_proc_pilot_module_enable_onewire_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_tty_proc_pilot_module_enable_onewire_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_enable_onewire_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_tty_proc_pilot_module_enable_onewire_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_tty_proc_pilot_module_enable_onewire_write
};
