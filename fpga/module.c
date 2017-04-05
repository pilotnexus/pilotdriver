#include <asm/delay.h>        /* needed for udelay() function */
#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */
#include <linux/seq_file.h>   /* needed for seq_file struct and functions */
#include <linux/slab.h>       /* needed for kmalloc() */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h"
#include "common.h"
#include "flash.h"

MODULE_LICENSE("GPL");

#define IGNORE -1
#define SELECT_CHIP 0
#define UNSELECT_CHIP 1

#define IN_RESET 0
#define NO_RESET 1

internals_t m_internals ={.timeout=500};

static const char fpga_name[] = "fpga";
#define IS_MODULE_TYPE(m, n) (strncmp(n, m->name, strlen(n)) == 0)

static const char pilot_fpga_proc_bitstream_name[] = "bitstream";

static void pilot_fpga_send_get_fpga_state(module_slot_t slot, int8_t chipselect, int8_t reset)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = rpcp_cmd_type_fpga_state;
  cmd.data[0] = (char)chipselect;
  cmd.data[1] = (char)reset;
  pilot_send_cmd(&cmd);
}

static int pilot_fpga_try_get_fpga_state(module_slot_t slot, int timeout, int8_t chipselect, int8_t reset)
{
  unsigned long timestamp;
  int timedout = 0;

  /* send a request for the fpga resolution */
  pilot_fpga_send_get_fpga_state(slot, chipselect, reset);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!m_internals.modules[slot].is_updated)
  {
    if (time_after(jiffies, timestamp))
    {
      timedout=1;
      break;
    }
    udelay(100);
  }

  m_internals.modules[slot].is_updated = 0;

 return timedout ? 0 : 1;
}

static int pilot_fpga_proc_bitstream_show(struct seq_file *file, void *data)
{
  /* get the module index */
  module_slot_t module_index = (module_slot_t)file->private;

  /* write the display buffer back to the user */
  //seq_write(file, m_internals.modules[module_index].display_buffer->data, DISPLAY_BUFFERSIZE);
  flash_read_id(module_index);

  return 0;
}

static int pilot_fpga_proc_bitstream_open(struct inode *inode, struct file *file)
{
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_open() for slot %i", (int)slot);

  if (!pilot_fpga_try_get_fpga_state(slot, m_internals.timeout, SELECT_CHIP, IN_RESET))
    return -EINVAL;

  return single_open(file, pilot_fpga_proc_bitstream_show, PDE_DATA(inode));
}

static int pilot_fpga_proc_bitstream_flush(struct file *file, fl_owner_t id)
{
  /* get the module slot */
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_flush() for slot %i", (int)slot);

  return 0;
}

static int pilot_fpga_proc_bitstream_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int bytes_written = 0, blocksize, sendbuffersize;
  target_t target = target_t_from_module_slot_and_port((int)PDE_DATA(file->f_inode), 0);


  LOG_DEBUG("called pilot_fpga_proc_bitstream_write(count=%i, off=%i)", count, (int)*off);

  //LOG_DEBUG("pilot_tty_write(count=%i) called", count);
  //spin_lock(&_internals_lock);

  while (bytes_written < count)
  {
    /* wait for the stm buffer to become available */
    while (pilot_get_stm_bufferstate() == stm_bufferstate_full)
      cpu_relax();

    /* wait for the internal sendbuffer to become available again */
    while ((sendbuffersize = pilot_get_free_send_buffer_size(target)) <= 0)
      cpu_relax();

    blocksize = count - bytes_written; /* sent the remaining bytes */
    if (blocksize > sendbuffersize)    /* ...but not more than the remaining sendbuffersize */
      blocksize = sendbuffersize;

    //pilot_send(target, buf+bytes_written, blocksize); /* send the bytes */
    bytes_written += blocksize; /* increment the number of bytes written */
  }

  //write bitstream
  return bytes_written;
}

static int pilot_fpga_proc_bitstream_release(struct inode *inode, struct file *file)
{
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_release() for slot %i", (int)slot);

  pilot_fpga_try_get_fpga_state(slot, m_internals.timeout, UNSELECT_CHIP, IN_RESET); //change to NO_RESET when hardware ready

  return single_release(inode, file);
}

/* file operations for /proc/pilot/moduleX/bitstream */
static const struct file_operations proc_bitstream_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_fpga_proc_bitstream_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = pilot_fpga_proc_bitstream_release,
  .write   = pilot_fpga_proc_bitstream_write,
  .flush   = pilot_fpga_proc_bitstream_flush
};

static const char pilot_fpga_proc_done_name[] = "done";

static int pilot_fpga_proc_done_show(struct seq_file *file, void *data)
{
  int count;
  int ret;
  char buffer[10];

  /* get the module index */
  int module_index = (int)file->private;

  /* send a get resolution command */
  if (!pilot_fpga_try_get_fpga_state(module_index, m_internals.timeout, -1, -1))
    ret = -EFAULT;
  else
  {
    /* create the string */
    count = snprintf(buffer, sizeof(buffer), "%i", m_internals.modules[module_index].done);

    /* write the string to the user */
    seq_write(file, buffer, count);

    ret = 0;
  }

  return ret;
}

static int pilot_fpga_proc_done_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_fpga_proc_done_show, PDE_DATA(inode));
}

static const struct file_operations proc_done_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_fpga_proc_done_open,
  .read    = seq_read,
  .release = single_release,
};

// *******************************************************************
// START pilot interface function implementation

static void pilot_fpga_proc_init(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  proc_create_data(pilot_fpga_proc_bitstream_name, 0666, module_dir, &proc_bitstream_fops, (void*)slot);
  proc_create_data(pilot_fpga_proc_done_name, 0444, module_dir, &proc_done_fops, (void*)slot);
}

static void pilot_fpga_proc_deinit(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  remove_proc_entry(pilot_fpga_proc_bitstream_name, module_dir);
  remove_proc_entry(pilot_fpga_proc_done_name, module_dir);
}

static void pilot_fpga_callback_recv(module_slot_t slot, module_port_t port, spidata_t data)
{
  LOG_DEBUG("pilot_fpga_callback_recv(), slot: %i, data: %i", (int)slot, (int)(data & 0xFF));

  if (m_internals.recv_buf_index + 1 < RECEIVEBUFFER_SIZE)
    m_internals.recv_buf[m_internals.recv_buf_index++] = data & 0xFF;
  else
    LOG_DEBUG("Receive Buffer Overflow");
}

static int pilot_fpga_callback_assign_slot(module_slot_t slot, const pilot_module_type_t *module_type)
{
  int fpga_type = 1;
  LOG_DEBUG("pilot_fpga_callback_assign_slot()");

  /* initialize it's proc files if any */
  pilot_fpga_proc_init(slot);
  
  return fpga_type;
}

static int pilot_fpga_callback_unassign_slot(module_slot_t slot)
{
  LOG_DEBUG("pilot_fpga_callback_unassign_slot()");

  /* destroy the proc devices, if any */
  pilot_fpga_proc_deinit(slot);

  return SUCCESS;
}

static int pilot_fpga_callback_can_assign(const pilot_module_type_t *module_type)
{
  LOG_DEBUG("called pilot_fpga_callback_can_assign()");
  return IS_MODULE_TYPE(module_type, fpga_name) ? SUCCESS : -1;
}

static void pilot_fpga_handle_get_fpga_state_cmd(pilot_cmd_t *cmd)
{
  module_slot_t slot = target_t_get_module_slot(cmd->target);

  m_internals.modules[slot].done = (uint8_t)cmd->data[3];

  mb();
  m_internals.modules[slot].is_updated = 1;
}

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  int ret;

  switch (cmd.type)
  {
    case rpcp_cmd_type_fpga_state:
      pilot_fpga_handle_get_fpga_state_cmd(&cmd);
      break;
    default: ret = pilot_cmd_handler_status_ignored; break;
  }

  return ret;
}

// END pilot interface function implementation
// *******************************************************************

/* rpcp struct with callback functions for the main rpcp driver */
static pilot_cmd_handler_t pilot_cmd_handler = {
  .callback_cmd_received = pilot_callback_cmd_received
};

/* description of our fpga module driver */
static register_driver_t register_driver = {
  .name                          = "fpga",
  .callback_assign_slot          = pilot_fpga_callback_assign_slot,
  .callback_unassign_slot        = pilot_fpga_callback_unassign_slot,
  .callback_recv                 = pilot_fpga_callback_recv,
  .callback_can_assign           = pilot_fpga_callback_can_assign
};

/* initialization routine, called when the module is loaded */
static int __init pilot_fpga_init(void)
{
  int ret = -1;

  // register with the base driver
  if ((m_internals.driver_id = pilot_register_driver(&register_driver)) < 0)
  {
    LOG(KERN_ERR, "pilot_register_driver() failed with %i", m_internals.driver_id);
  }
  else
  {
    /* register the cmd_handler */
    if (pilot_register_cmd_handler(&pilot_cmd_handler) == SUCCESS)
    {
      LOG_DEBUG("pilot_register_cmd_handler() succeeded");
      m_internals.is_cmd_handler_registered = 1;
      ret = SUCCESS;

      /* request an autoconfiguration of all modules */
      pilot_auto_configure();
    }
  }

  return ret;
}

/* fpga module cleanup function, called when removing the module */
static void __exit pilot_fpga_exit(void)
{
  /* unregister with the base driver */
  if (m_internals.driver_id >= 0)
    pilot_unregister_driver(m_internals.driver_id);

  /* unregister with the base driver */
  if (m_internals.is_cmd_handler_registered) {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      m_internals.is_cmd_handler_registered = 0;
  }
}

/* main entry point */
module_init(pilot_fpga_init);

/*  main exit point */
module_exit(pilot_fpga_exit);
