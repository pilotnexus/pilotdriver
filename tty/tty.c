/*    
   handles the tty device driver implementation
   written by mdk
*/

#include "tty.h"
#include "common.h"
#include "../driver/export.h"
#include "tty_commands.h"
#include <linux/sched.h>  /* included for tty_wakeup() function */
#include <linux/string.h> /* included for the memset() function */
#include <linux/interrupt.h> /* needed for tasklet_schedule and DECLARE_TASKLET */
#include <linux/spinlock.h>   /* needed for spinlock_t and it's functions */
#include <linux/serial.h>

/* forward declaration of functions */
static int  pilot_tty_install        (struct tty_driver *, struct tty_struct *);
static int  pilot_tty_open           (struct tty_struct *, struct file *);
static void pilot_tty_close          (struct tty_struct *, struct file *);
static int  pilot_tty_write          (struct tty_struct *, 
                                    const unsigned char *, 
                                    int);
static int  pilot_tty_write_room     (struct tty_struct *);
static void pilot_tty_flush_buffer   (struct tty_struct *);
static int  pilot_tty_chars_in_buffer(struct tty_struct *);
static void pilot_tty_set_termios    (struct tty_struct *, struct ktermios *);
static void pilot_tty_stop           (struct tty_struct *);
static void pilot_tty_start          (struct tty_struct *);
static void pilot_tty_hangup         (struct tty_struct *);
static int  pilot_tty_tiocmget       (struct tty_struct *tty);
static int  pilot_tty_tiocmset       (struct tty_struct *tty,
                                    unsigned int set, 
                                    unsigned int clear);
static int  pilot_tty_ioctl          (struct tty_struct* tty,
                                    unsigned int cmd,
                                    unsigned int long arg);
static int pilot_tty_break_ctl       (struct tty_struct* tty, int state);
//static void pilot_tty_wakeup_tasklet (unsigned long);

/* tty_operations */
static const struct tty_operations pilot_tty_ops = {
  .install         = pilot_tty_install,
  .open            = pilot_tty_open,
  .close           = pilot_tty_close,
  .write           = pilot_tty_write,

  .write_room      = pilot_tty_write_room,
  .flush_buffer    = pilot_tty_flush_buffer,
  .chars_in_buffer = pilot_tty_chars_in_buffer,
  .ioctl           = pilot_tty_ioctl,
  .set_termios     = pilot_tty_set_termios,
  .stop            = pilot_tty_stop,
  .start           = pilot_tty_start,
  .hangup          = pilot_tty_hangup,
  .tiocmget        = pilot_tty_tiocmget,
  .tiocmset        = pilot_tty_tiocmset,
  .break_ctl       = pilot_tty_break_ctl
};

/* holds per device information */
typedef struct {
  int Registered; /* 0 if not registered, 1 if registered */
  int OpenCount;  /* the number of times open device was called for this particular device */
  struct tty_struct* tty;
  //struct tty_port tty_port;
  volatile stm_bufferstate_t bufferstate; /* the current state of the buffer */
} device_t;

/* holds per module information */
typedef struct {
  device_t devices[MODULE_PORT_COUNT]; /* holds information for every device */
} module_t;

#define TTY_PORT_COUNT (MODULE_PORT_COUNT * MODULES_COUNT)

#define BAUD_BASE 1000000

/* holds private data used for handling of the tty driver */
typedef struct {
  module_t modules[MODULES_COUNT]; /* holds information for every module slot */
  struct tty_driver* Driver;
  //struct tty_struct* tty_wakeup;
  struct tty_port tty_ports[TTY_PORT_COUNT];
  //int tty_ports_custom_baudrate[TTY_PORT_COUNT];
  int tty_port_custom_divisor[TTY_PORT_COUNT];
} internals_t;

static internals_t _internals;
static spinlock_t _internals_lock;

//static DECLARE_TASKLET(_internals_tty_wakeup_tasklet, pilot_tty_wakeup_tasklet, 0);

/* description: registers the tty device driver 'pilotrpcp' and it's tty operations .
                it does not create any devices in the filesystem, those are created when a slot is assigned.
   returns: 0 if successful, otherwise negative number */
int pilot_tty_register_driver()
{
  struct tty_driver* driver;
  int i /*, j*/;

  LOG_DEBUG("pilot_tty_register_driver() called");

  /* allocate the tty_ports */
  for (i = 0; i < TTY_PORT_COUNT; i++)
    tty_port_init(&_internals.tty_ports[i]);

  /* allocate the driver - we can have at most TTY_PORT_COUNT devices */
  if (!(driver = alloc_tty_driver(TTY_PORT_COUNT)))
    return -ENOMEM;

  LOG_DEBUG("alloc_tty_driver returned %x", (unsigned int)driver);

  driver->owner = THIS_MODULE;
  driver->driver_name = "pilottty";
  driver->name = "ttypilot";
  driver->major = 0;
  driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV; /* we register our devices ourself */
  //pDriver->num = 0;
  driver->num = 8;
  driver->type = TTY_DRIVER_TYPE_SERIAL;
  driver->subtype = SERIAL_TYPE_NORMAL;
  driver->init_termios = tty_std_termios; /* use some defaults */
  driver->init_termios.c_ispeed = 9600;
  driver->init_termios.c_ospeed = 9600;
  driver->init_termios.c_cflag = B9600 | CREAD | CS8 | CLOCAL;
  driver->init_termios.c_lflag = 0;

  /* initialize the function callbacks of tty_driver struct */
  tty_set_operations(driver, &pilot_tty_ops);

  ///* link the tty_ports to the driver */
  //for (i = 0; i < TTY_PORT_COUNT; i++)
  //  tty_port_link_device(&_internals.tty_ports[i], pDriver, i);
  //for (i = 0; i < MODULES_COUNT; i++)
  //  for (j = 0; j < MODULE_PORT_COUNT; j++)
  //    tty_port_link_device(&_internals.modules[i].devices[j].tty_port, driver, i);


  LOG_DEBUG("calling tty_register_driver()");
  if (tty_register_driver(driver))
  {
    LOG(KERN_ERR, "tty_register_driver() failed");
    put_tty_driver(driver); /* deallocate the driver if registering fails */
    return -1;
  }

  //LOG_DEBUG("_internals.Driver=%x", (unsigned int)_internals.Driver);  
  _internals.Driver = driver; /* store the pointer in internals */
  LOG_DEBUG("_internals.Driver=%x", (unsigned int)_internals.Driver);

  LOG_DEBUG("pilot_tty_register_driver() successful");

  return SUCCESS;
}

/* description: unregisters the tty device driver */
void pilot_tty_unregister_driver()
{
  int i,j;
  LOG_DEBUG("pilot_tty_unregister_driver() called");

  spin_lock(&_internals_lock);

  if (_internals.Driver != NULL) {
    
    // unregister already registered devices first
    for (i = 0; i < MODULES_COUNT; i++)
      for (j = 0; j < MODULE_PORT_COUNT; j++)
        if (_internals.modules[i].devices[j].Registered)
          pilot_tty_unregister_device((module_slot_t)i, (module_port_t)j);

    /* then unregister the driver */
    tty_unregister_driver(_internals.Driver); 

    /* cleanup the driver struct */
    put_tty_driver(_internals.Driver);
  }

  spin_unlock(&_internals_lock);
}

#define device_index_from(module_slot, module_port) (((int)module_slot)*2 + ((int)module_port))
#define device_index_get_module_slot(index) ((int)(index/2))
#define device_index_get_module_port(index) ((int)(index%2))
#define device_index_get_target(index) ((target_t)index+1)

/* description: registers a device in the filesystem */
void pilot_tty_register_device(module_slot_t slot, module_port_t port)
{
  LOG_DEBUG("pilot_tty_register_device(slot=%i) called", slot);

  /* if the device wasn't registered before... */
  if (!_internals.modules[(int)slot].devices[(int)port].Registered)
  {
    /* mark it as registered */
    _internals.modules[(int)slot].devices[(int)port].Registered = 1;

    /* set the slot as the index, so that we can later retrieve the slot by accessing tty->index */
    tty_register_device(_internals.Driver, device_index_from(slot, port), NULL);
  }

}

/* description: unregisters the tty device for the specified slot */
void pilot_tty_unregister_device(module_slot_t slot, module_port_t port)
{
  LOG_DEBUG("pilot_tty_unregister_device(slot=%i, port=%i) called", slot, port);

  /* if the device was registered before... */
  if (_internals.modules[(int)slot].devices[(int)port].Registered)
  {
    /* mark it as free */
    _internals.modules[(int)slot].devices[(int)port].Registered = 0;

    /* unregister the device */
    tty_unregister_device(_internals.Driver, device_index_from(slot, port));
  }
}

/* description: forwards the received data to the specific tty device, if any */
void pilot_tty_received_data(module_slot_t slot, module_port_t port, uint8_t data)
{
  struct tty_struct* tty;
  device_t *device;

  //LOG_DEBUG("pilot_tty_received_data(slot=%i, data=%i)", slot, data);

  spin_lock(&_internals_lock);
  
  device = &_internals.modules[(int)slot].devices[(int)port];
  tty = device->tty;

  if (device->Registered)
  {
    if (tty != NULL && device->OpenCount)
    {
      tty_insert_flip_char(tty->port, data, TTY_NORMAL);
      tty_flip_buffer_push(tty->port);
    }
  }

  spin_unlock(&_internals_lock);
}

//static void pilot_tty_wakeup_tasklet(unsigned long unused)
//{
//  struct tty_struct* tty = _internals.tty_wakeup;
//  LOG_INFO("waking up tty");
//  tty_wakeup(tty);
//  LOG_INFO("tty_wakeup finished");
//}

/* description: stores the bufferstate for tty device or wakes it up again */
//void pilot_tty_bufferstate_changed(module_slot_t slot, module_port_t port, stm_bufferstate_t bufferstate)
//{
//  struct tty_struct* tty;
//  //LOG_DEBUG("pilot_tty_bufferstate_changed(slot=%i, port=%i, bufferstate=%i) called", slot, port, bufferstate);
//  stm_bufferstate_t old_state = _internals.modules[slot].devices[port].bufferstate;
//
//  if (old_state != bufferstate)
//  {
//    LOG_INFO("bufferstate of slot=%i, port=%i changed from %i to %i", slot, port, old_state, bufferstate);
//    _internals.modules[slot].devices[port].bufferstate = bufferstate;
//  }
//
//  if (old_state   == stm_bufferstate_full && 
//      bufferstate == stm_bufferstate_not_full)
//  {
//    tty = _internals.modules[slot].devices[port].tty;
//    if (tty)
//    {
//      _internals.tty_wakeup = tty;
//      tasklet_schedule(&_internals_tty_wakeup_tasklet);
//    }
//    else
//    {
//      LOG_INFO("tty was NULL!");
//    }
//  }
//}

/* START implementation of tty functions */

static int  pilot_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
  return tty_port_install(&_internals.tty_ports[tty->index], driver, tty);
}

static int pilot_tty_open(struct tty_struct* tty, struct file* file)
{
  int ret; 
  device_t* pDev;

  spin_lock(&_internals_lock);

  pDev = &_internals.modules[(int)device_index_get_module_slot(tty->index)].devices[(int)device_index_get_module_port(tty->index)];

  LOG_DEBUG("pilot_tty_open() called");

  if (pDev->OpenCount++)
  {
    LOG_DEBUG("pilot_tty_open() failed as OpenCount is now %i", pDev->OpenCount);
    ret = -ENODEV;
  }
  else
  {
    LOG_DEBUG("pilot_tty_open() successful");
    pDev->tty = tty; /* associate the tty with this device so that we can forward messages to the tty */
    ret = SUCCESS;
  }

  spin_unlock(&_internals_lock);

  return ret;
}

static void pilot_tty_close(struct tty_struct* tty, struct file* file)
{
  device_t* pDev;

  spin_lock(&_internals_lock);

  pDev = &_internals.modules[(int)device_index_get_module_slot(tty->index)].devices[(int)device_index_get_module_port(tty->index)];

  LOG_DEBUG("pilot_tty_close() called");

  if (--pDev->OpenCount)
  {
    LOG_DEBUG("the device was not closed, the OpenCount is now %i", pDev->OpenCount);
  }
  //else
  //{
  //  LOG_DEBUG("the device was closed.");
  //  pDev->tty = NULL;
  //}

  spin_unlock(&_internals_lock);
}

#define pilot_tty_max_blocksize 8

/* sends the bytes to the base driver  */
/* returns the number of bytes written */
static int pilot_tty_write(struct tty_struct* tty,
                          const unsigned char* buf,
                          int count)
{
  int bytes_written = 0, blocksize, sendbuffersize;

  //LOG_DEBUG("pilot_tty_write(count=%i) called", count);
  //spin_lock(&_internals_lock);

  while (bytes_written < count)
  {
    /* wait for the stm buffer to become available */
    while (pilot_get_stm_bufferstate() == stm_bufferstate_full)
      cpu_relax();

    /* wait for the internal sendbuffer to become available again */
    while ((sendbuffersize = pilot_get_free_send_buffer_size(device_index_get_target(tty->index))) <= 0)
      cpu_relax();

    blocksize = count - bytes_written; /* sent the remaining bytes */
    if (blocksize > sendbuffersize)    /* ...but not more than the remaining sendbuffersize */
      blocksize = sendbuffersize;
    if (blocksize > pilot_tty_max_blocksize)
      blocksize = pilot_tty_max_blocksize;       /* ...or more than max_block size */

    pilot_send(device_index_get_target(tty->index), buf+bytes_written, blocksize); /* send the bytes */
    bytes_written += blocksize;                                                   /* increment the number of bytes written */
  }

  //LOG_INFO("pilot_tty_write(count=%i) returned: %i", count, bytes_written);
  //spin_unlock(&_internals_lock);

  return bytes_written;
}

/* callback function for the linux kernel
   returns the number of free bytes in the tty buffer of the tty driver */
static int pilot_tty_write_room(struct tty_struct* tty)
{
  int bytes_free;
  stm_bufferstate_t bufferstate;

  LOG_DEBUG("pilot_tty_write_room() called");
  //spin_lock(&_internals_lock);

  /* get the bufferstate of the target */
  bufferstate = _internals.modules[device_index_get_module_slot(tty->index)].devices[device_index_get_module_port(tty->index)].bufferstate;

  /* if the bufferstate is full, return 0 */
  if (bufferstate == stm_bufferstate_full)
  {
    LOG_INFO("pilot_tty_write_room() returns 0, because bufferstate = full");
    bytes_free = 0;
  }
  else
  {
    /* wait for the send buffer to become available */
    while ((bytes_free = pilot_get_free_send_buffer_size(device_index_get_target(tty->index))) <= 0)
      cpu_relax();

    if (bytes_free > pilot_tty_max_blocksize)
      bytes_free = pilot_tty_max_blocksize;
  }

  LOG_DEBUG("pilot_tty_write_room() returns %i", bytes_free);
  //spin_unlock(&_internals_lock);

  return bytes_free;
}

static void pilot_tty_flush_buffer(struct tty_struct* tty)
{
  // LOG_DEBUG("pilot_tty_flush_buffer() called");

  /* todo clear the send queue */
  // tty_wakeup(tty);
}

static int pilot_tty_chars_in_buffer(struct tty_struct* tty)
{
  LOG_DEBUG("pilot_tty_chars_in_buffer() called");
  return 0;
}

static void pilot_tty_send_break(int deviceIndex)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = device_index_get_target(deviceIndex);
  cmd.type = pilot_cmd_type_usart_send_break;
  pilot_send_cmd(&cmd);
}

static void pilot_tty_send_configure(int deviceIndex, pilot_cmd_baudrate_t baudrate, pilot_cmd_stopbits_t stopbits, pilot_cmd_parity_t parity, pilot_cmd_wordlength_t wordlength)
{
  pilot_cmd_t cmd;
  LOG_INFO("pilot_tty_send_configure() baudrate=%i, stopbits=%i, parity=%i, wordlength=%i", baudrate, stopbits, parity, wordlength);

  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = device_index_get_target(deviceIndex);

  /* set baudrate */
  cmd.type = pilot_cmd_type_usart_set_baudrate;
  cmd.data[0] = baudrate;
  pilot_send_cmd(&cmd);

  /* set stopbits */
  cmd.type = pilot_cmd_type_usart_set_stopbits;
  cmd.data[0] = stopbits;
  pilot_send_cmd(&cmd);

  /* set parity */
  cmd.type = pilot_cmd_type_usart_set_parity;
  cmd.data[0] = parity;
  pilot_send_cmd(&cmd);

  /* set wordlength */
  cmd.type = pilot_cmd_type_usart_set_wordlength;
  cmd.data[0] = wordlength;
  pilot_send_cmd(&cmd);
}

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

static void pilot_tty_change_settings(struct tty_struct *tty)
{
  pilot_cmd_stopbits_t stopbits;
  pilot_cmd_parity_t parity;
  pilot_cmd_baudrate_t baudrate;
  pilot_cmd_wordlength_t wordlength;
  int databits;
  int cflag = tty->termios.c_cflag;
  int custom_baudrate;

  /* debug test */
  //LOG_INFO("c_cflag & CBAUD: %i", tty->termios.c_cflag & CBAUD);
  //LOG_INFO("baudrate from tty_get_baudrate(): %i", tty_get_baud_rate(tty));
  //LOG_INFO("c_ispeed: %i, c_ospeed: %i", tty->termios.c_ispeed, tty->termios.c_ospeed);

  /* get the baudrate */
  baudrate = rpc_stm_proto_int_to_baudrate(tty_get_baud_rate(tty));

  /* handle custom baudrates */
  if (baudrate == pilot_cmd_baudrate_38400 && _internals.tty_port_custom_divisor[tty->index] != 0)
  {
    LOG_INFO("custom baudrate with divisor: %i", _internals.tty_port_custom_divisor[tty->index]);
    custom_baudrate = BAUD_BASE / _internals.tty_port_custom_divisor[tty->index];
    if (custom_baudrate == 250000)
      baudrate = pilot_cmd_baudrate_250000;
  }

  /* get the stopbits */
  stopbits = (cflag & CSTOPB) ? pilot_cmd_stopbits_2 : pilot_cmd_stopbits_1;

  /* get the parity */
  if (cflag & PARENB)
    parity = (cflag & PARODD) ? pilot_cmd_parity_odd : pilot_cmd_parity_even;
  else
    parity = pilot_cmd_parity_none;

  /* get the requested databits */
  switch (cflag & CSIZE)
  {
    case CS5: databits = 5; break;
    case CS6: databits = 6; break;
    case CS7: databits = 7; break;
    default:
    case CS8:
      databits = 8;
      break;
  }

  /* now calculate the wordlength */
  if ((databits == 7 && parity != pilot_cmd_parity_none) ||  /* 7 databits + 1 parity = wordlength 8 */
    (databits == 8 && parity == pilot_cmd_parity_none))    /* 8 databits + 0 parity = wordlength 8 */
    wordlength = pilot_cmd_wordlength_8;
  else if (databits == 8 && parity != pilot_cmd_parity_none) /* 8 databits + 1 parity = wordlength 9 */
    wordlength = pilot_cmd_wordlength_9;
  else
    wordlength = pilot_cmd_wordlength_8; /* default to wordlength 8 */

  /* send commands to the module */
  pilot_tty_send_configure(tty->index, baudrate, stopbits, parity, wordlength);
}

static void pilot_tty_set_termios(struct tty_struct* tty, struct ktermios* old_termios)
{
  LOG_DEBUG("pilot_tty_set_termios() called");

  /* check if anything changed */
  if (!(old_termios && (tty->termios.c_cflag  == old_termios->c_cflag &&
                        tty->termios.c_ispeed == old_termios->c_ispeed &&
                        tty->termios.c_ospeed == old_termios->c_ospeed)))
  {
    pilot_tty_change_settings(tty);
  }
}

static void pilot_tty_stop(struct tty_struct* tty)
{
  LOG_INFO("pilot_tty_stop() called");
}

static void pilot_tty_start(struct tty_struct* tty)
{
  LOG_INFO("pilot_tty_start() called");
}

static void pilot_tty_hangup(struct tty_struct* tty)
{
  LOG_DEBUG("pilot_tty_hangup() called");
}

static int pilot_tty_tiocmget(struct tty_struct* tty)
{
  LOG_DEBUG("pilot_tty_tiocmget() called");
  return SUCCESS;
}

static int pilot_tty_tiocmset(struct tty_struct* tty,
                            unsigned int set,
                            unsigned int clear)
{
  LOG_DEBUG("pilot_tty_tiocmset() called");
  return SUCCESS;
}

static int pilot_tty_ioctl(struct tty_struct* tty,
                         unsigned int cmd,
                         unsigned long int arg)
{
  int ret;
  struct serial_struct ss;

  LOG_DEBUG("pilot_tty_ioctl(cmd=%i, arg=%li) called", cmd, arg);

  switch (cmd)
  {
    case TIOCGSERIAL:
      /* send the serial_struct to the user */
      memset(&ss, 0, sizeof(ss));
      /* update the custom_divisor */
      //ss.custom_divisor = _internals.tty_port_custom_divisor[tty->index];
      ss.baud_base = BAUD_BASE; /* set the baud_base as this is need for tools to calculate the divisor if they want a custom baudrate */
      ret = (copy_to_user((void __user *)arg, &ss, sizeof(ss))) ? -EFAULT : SUCCESS;
      break;

    case TIOCSSERIAL:
      /* get the serial_struct from the user */
      memset(&ss, 0, sizeof(ss));
      if (copy_from_user(&ss, (void __user *)arg, sizeof(ss)))
        ret = -EFAULT;
      else
      {
        /* update the custom baudrate from the divisor */
        LOG_DEBUG("setting custom divisor to: %i", ss.custom_divisor);
        _internals.tty_port_custom_divisor[tty->index] = ss.custom_divisor;
        ret = SUCCESS;
      }
      break;

    case TIOCMGET: ret = SUCCESS; break;
    case TIOCMSET: ret = SUCCESS; break;
    default: ret = -ENOIOCTLCMD; break;
  }

  return ret;
}

static int pilot_tty_break_ctl(struct tty_struct* tty, int state)
{
  LOG_DEBUG("pilot_tty_break_ctl(state=%i)", state);

  switch(state)
  {
    /* break should be turned on */
    case -1:
      pilot_tty_send_break(tty->index);
      break;

    /* break should be turned off */
    case 0: break;

    default: break;
  }

  return SUCCESS;
}

/* END implementation of tty functions */