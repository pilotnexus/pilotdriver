#include <asm/delay.h>        /* needed for udelay() function */
#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */
#include <linux/seq_file.h>   /* needed for seq_file struct and functions */
#include <linux/slab.h>       /* needed for kmalloc() */
#include <linux/wait.h>       /* waitqueue */
#include <linux/delay.h>       // needed for msleep()
#include <linux/uaccess.h>     // needed for copy_from_user() function
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h"
#include "../driver/queue.h"
#include "common.h"
#include "flash.h"

MODULE_LICENSE("GPL");

#define IGNORE -1
#define SELECT_CHIP 0
#define UNSELECT_CHIP 1

#define IN_RESET 0
#define NO_RESET 1

DECLARE_WAIT_QUEUE_HEAD(recv_wait); 
internals_t m_internals ={.timeout=500};

struct mutex access_lock;

static const char fpga_name[] = "fpga";
#define IS_MODULE_TYPE(m, n) (strncmp(n, m->name, strlen(n)) == 0)

static const char pilot_fpga_proc_bitstream_name[] = "bitstream";

static void pilot_fpga_send_get_fpga_state(module_slot_t slot, int8_t chipselect, int8_t reset)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_fpga_state;
  cmd.data[0] = (char)chipselect;
  cmd.data[1] = (char)reset;
  cmd.length = MSG_LEN(4); //4 bytes, min size
  pilot_send_cmd(&cmd);
}

static int pilot_fpga_try_get_fpga_state(module_slot_t slot, int timeout, int8_t chipselect, int8_t reset)
{
  unsigned long timestamp;
  int timedout = 0;

  LOG_DEBUG("calling pilot_fpga_send_get_fpga_state()");

  pilot_fpga_send_get_fpga_state(slot, chipselect, reset);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!m_internals.modules[slot].is_updated)
  {
    if (time_after(jiffies, timestamp))
    {
      timedout=1;
      LOG_DEBUG("calling pilot_fpga_send_get_fpga_state() TIMED OUT");
      break;
    }
    udelay(100);
  }

  m_internals.modules[slot].is_updated = 0;

 return timedout ? 0 : 1;
}

static void pilot_fpga_send_fpga_cmd(module_slot_t slot, uint8_t *data, uint16_t size)
{
  int cmdDataLength = size + 2;
  int lentmp = (cmdDataLength >> 2) << 2;
  pilot_cmd_t cmd;

  if (lentmp != cmdDataLength) //adjust for 4 length
    cmdDataLength = lentmp + 4;

  if (cmdDataLength > pilot_cmd_t_data_size)
    cmdDataLength = pilot_cmd_t_data_size;

  if (size > pilot_cmd_t_data_size-2)
    size = pilot_cmd_t_data_size-2;

  LOG_DEBUG("pilot_fpga_send_fpga_cmd(), cmd data length=%i (%i), actual data length=%i",  
    MSG_LEN(cmdDataLength), cmdDataLength, size);


  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  memcpy(cmd.data, (void *)&size, 2);
  memcpy(&cmd.data[2], data, size);
  cmd.type = pilot_cmd_type_fpga_cmd;
  cmd.length = MSG_LEN(cmdDataLength);
  pilot_send_cmd(&cmd);
}

int pilot_fpga_try_send_fpga_cmd(module_slot_t slot, uint8_t *data, int size, int timeout)
{
  unsigned long timestamp;
  int timedout = 0;

  LOG_DEBUGALL("calling pilot_fpga_send_fpga_cmd()");

  pilot_fpga_send_fpga_cmd(slot, data, size);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!m_internals.cmd[slot].is_updated)
  {
    if (time_after(jiffies, timestamp))
    {
      timedout=1;
      LOG_DEBUG("calling pilot_fpga_send_fpga_cmd() TIMED OUT");
      break;
    }
    udelay(100);
  }

  if (timedout == 0) {
    if (size > pilot_cmd_t_data_size-2)
      size = pilot_cmd_t_data_size-2;

    memcpy(data, &m_internals.cmd[slot].cmd_buffer[2], size);
  }

  m_internals.cmd[slot].is_updated = 0;

 return timedout ? 0 : 1;
}

static ssize_t pilot_fpga_proc_bitstream_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
 int module_index = (int)PDE_DATA(filp->f_inode);
 int ret = 0, chunk, pos = (int)*f_pos;

 LOG_DEBUG("called pilot_fpga_proc_bitstream_read() for slot %i, fpos %i", (int)module_index, (int)*f_pos);

 if (m_internals.modules[module_index].bitstream_size == 0)
 {
    flash_power_up(module_index);
    flash_read(module_index, 0x07FF00, (char *)&m_internals.modules[module_index].bitstream_size, sizeof(size_t));
    LOG_DEBUG("reading bitstream (size: %i)", (int)m_internals.modules[module_index].bitstream_size);
 }

 if (m_internals.modules[module_index].bitstream_size > 0 && (pos < m_internals.modules[module_index].bitstream_size))
 {
    chunk = (pos + 256) > m_internals.modules[module_index].bitstream_size ? m_internals.modules[module_index].bitstream_size-pos : 256;
    flash_read(module_index, pos, buf, chunk);
    ret = chunk;
    *f_pos += chunk;
 }

 return ret;
}

/*
static int pilot_fpga_proc_bitstream_show(struct seq_file *file, void *data)
{
  //int addr = 0;
  char buffer[256];
  size_t count = 0, addr=0, chunk;

  int module_index = (int)file->private;
  target_t target = target_t_from_module_slot_and_port(module_index, module_port_2);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_show(target: %i, count=%i)", (int)target, count);

  // power up flash 
  flash_power_up(module_index);

  flash_read(module_index, 0x07FF00, (char *)&count, sizeof(size_t));

  if (count != -1) //flash is not empty
  {
    LOG_DEBUG("reading bitstream (size: %i)", (int)count);

    while (addr < count)
    {
      chunk = (addr + 256) > count ? count-addr : 256;
      flash_read(module_index, addr, buffer, chunk);
      addr += chunk;

      #ifdef DEBUG
        printk("%i", chunk);
      #endif

      if (!seq_write(file, buffer, chunk))
      {
        LOG_DEBUG("Error calling seq_write");
        return -EINVAL;
      }
    }
    #ifdef DEBUG
    printk("\n");
    #endif

  }
  return 0;
}
*/

static int pilot_fpga_proc_bitstream_open(struct inode *inode, struct file *file)
{
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_open() for slot %i", (int)slot);
  m_internals.modules[slot].bitstream_size = 0;
  m_internals.modules[slot].bitstream_pos = 0;

  if (!pilot_fpga_try_get_fpga_state(slot, m_internals.timeout, IGNORE, IN_RESET))
    return -EINVAL;

  //return single_open(file, pilot_fpga_proc_bitstream_show, PDE_DATA(inode));
  return 0;
}

static int pilot_fpga_proc_bitstream_flush(struct file *file, fl_owner_t id)
{
  /* get the module slot */
  #ifdef DEBUG
  module_slot_t slot = (int)PDE_DATA(file->f_inode);
  LOG_DEBUG("called pilot_fpga_proc_bitstream_flush() for slot %i", (int)slot);
  #endif

  return 0;
}

static ssize_t pilot_fpga_proc_bitstream_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int addr = 0, blocksize, bytes_written = 0;
  
  int module_index = (int)PDE_DATA(file->f_inode);
  target_t target = target_t_from_module_slot_and_port(module_index, module_port_1);

  LOG_DEBUG("called pilot_fpga_proc_bitstream_write(target: %i, count=%i, off=%i)", (int)target, count, (int)*off);

  flash_power_up(module_index);
  /* FLASH WRITE ENABLE */
  flash_write_enable(module_index);

  LOG_DEBUG("erasing flash...");
  /* FLASH BULK ERASE */
  flash_bulk_erase(module_index);

  while(flash_busy(module_index))
    if (msleep_interruptible(100))
      return -EINVAL;
      
  LOG_DEBUG("flash ready after erase.");

  //LOG_DEBUG("pilot_tty_write(count=%i) called", count);
  //spin_lock(&_internals_lock);


  while (bytes_written < count)
  {
    target = target_t_from_module_slot_and_port(module_index, module_port_1); //write, no return data
    blocksize = (count - bytes_written) > 256 ? 256 : (count - bytes_written); /* sent the remaining bytes */

    if (!flash_prog(module_index, addr, (char *)(buf+bytes_written), blocksize))
      return -EINVAL;

    addr += 256;
    bytes_written += blocksize; /* increment the number of bytes written */
  }

  if (!flash_prog(module_index, 0x07FF00, (char *)&count, sizeof(size_t)))
    return -EINVAL;

  if (!pilot_fpga_try_get_fpga_state(module_index, 1000, UNSELECT_CHIP, NO_RESET))
     return -EINVAL;
  
  //write bitstream
  return bytes_written;
}

static int pilot_fpga_proc_bitstream_release(struct inode *inode, struct file *file)
{

  #ifdef DEBUG
  module_slot_t slot = (int)PDE_DATA(file->f_inode);
  LOG_DEBUG("called pilot_fpga_proc_bitstream_release() for slot %i", (int)slot);
  #endif

  //pilot_fpga_try_get_fpga_state(slot, m_internals.timeout, UNSELECT_CHIP, IN_RESET); //change to NO_RESET when hardware ready

  //return single_release(inode, file);
  return 0;
}

/* file operations for /proc/pilot/moduleX/bitstream */
static const struct file_operations proc_bitstream_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_fpga_proc_bitstream_open,
  //.read    = seq_read,
  .read    = pilot_fpga_proc_bitstream_read,
  .llseek  = seq_lseek,
  .release = pilot_fpga_proc_bitstream_release,
  .write   = pilot_fpga_proc_bitstream_write,
  .flush   = pilot_fpga_proc_bitstream_flush
};

// *******************************************************************
// START proc "cmd"

static const char pilot_fpga_proc_cmd_name[] = "cmd";

static ssize_t pilot_fpga_proc_cmd_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int bytes_count = count <= pilot_cmd_t_data_size ? count : pilot_cmd_t_data_size;
  
  int module_index = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_fpga_proc_cmd_write(module_index: %i, count=%i, off=%i)", module_index, count, (int)*off);

  if (copy_from_user(m_internals.cmd[module_index].buffer, buf, bytes_count) == 0)
  {
    pilot_fpga_try_send_fpga_cmd(module_index, m_internals.cmd[module_index].buffer, bytes_count, 500);
    m_internals.cmd[module_index].length = bytes_count;
    return bytes_count;
  }
  return -EINVAL;
}
static int pilot_fpga_proc_cmd_show(struct seq_file *file, void *data)
{
  /* get the module index */
  int module_index = (int)file->private;

  /* write the string to the user */
  seq_write(file, (const void *)m_internals.cmd[module_index].buffer, m_internals.cmd[module_index].length);

  return 0;
}

static int pilot_fpga_proc_cmd_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_fpga_proc_cmd_show, PDE_DATA(inode));
}

static const struct file_operations proc_cmd_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_fpga_proc_cmd_open,
  .read    = seq_read,
  .write   = pilot_fpga_proc_cmd_write,
  .release = single_release,
};

// *******************************************************************
// START proc "done"

static const char pilot_fpga_proc_done_name[] = "done";

static int pilot_fpga_proc_done_show(struct seq_file *file, void *data)
{
  int count;
  int ret;
  char buffer[10];

  /* get the module index */
  int module_index = (int)file->private;

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
// START proc "flash_id"

static const char pilot_fpga_proc_flash_id_name[] = "flash_id";

static int pilot_fpga_proc_flash_id_show(struct seq_file *file, void *data)
{
  int ret;
  char buffer[3];
  int density_code;

  /* get the module index */
  int module_index = (int)file->private;

  if (!pilot_fpga_try_get_fpga_state(module_index, m_internals.timeout, IGNORE, IN_RESET))
    return -EFAULT;

  flash_power_up(module_index);

  flash_read_id(module_index, buffer);

  density_code = buffer[1] & 0x1F;

  seq_printf(file, "Manufacturer: %s\nSize:%iMBit\n", buffer[0] == 0x1F ? "Adesto" : "unknown", density_code);
  
  ret = 0;

  return ret;
}

static int pilot_fpga_proc_flash_id_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_fpga_proc_flash_id_show, PDE_DATA(inode));
}

static const struct file_operations proc_flash_id_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_fpga_proc_flash_id_open,
  .read    = seq_read,
  .release = single_release,
};

// *******************************************************************
// START pilot interface function implementation

static void pilot_fpga_proc_init(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  init_waitqueue_head(&m_internals.receive_queue);
  mutex_init(&access_lock);
  proc_create_data(pilot_fpga_proc_bitstream_name, 0666, module_dir, &proc_bitstream_fops, (void*)slot);
  proc_create_data(pilot_fpga_proc_cmd_name, 0666, module_dir, &proc_cmd_fops, (void*)slot);
  proc_create_data(pilot_fpga_proc_done_name, 0444, module_dir, &proc_done_fops, (void*)slot);
  proc_create_data(pilot_fpga_proc_flash_id_name, 0444, module_dir, &proc_flash_id_fops, (void*)slot);
}

static void pilot_fpga_proc_deinit(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  remove_proc_entry(pilot_fpga_proc_bitstream_name, module_dir);
  remove_proc_entry(pilot_fpga_proc_cmd_name, module_dir);
  remove_proc_entry(pilot_fpga_proc_done_name, module_dir);
  remove_proc_entry(pilot_fpga_proc_flash_id_name, module_dir);
}

static void pilot_fpga_callback_recv(module_slot_t slot, module_port_t port, spidata_t data)
{
  LOG_DEBUGALL("pilot_fpga_callback_recv(), slot: %i, data: %i", (int)slot, (int)(data & 0xFF));
  mb();
  if (m_internals.recv_buf_index + 1 < RECEIVEBUFFER_SIZE)
  {
    m_internals.recv_buf[m_internals.recv_buf_index++] = data & 0xFF;
    wake_up_interruptible(&m_internals.receive_queue);
  }
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

static void pilot_fpga_handle_get_fpga_cmd(pilot_cmd_t *cmd)
{
  uint32_t length = cmd->length << 2;
  module_slot_t slot = target_t_get_module_slot(cmd->target);

  mb();
  if (length > pilot_cmd_t_data_size)
    length = pilot_cmd_t_data_size;

  LOG_DEBUG("pilot_fpga_handle_get_fpga_cmd() received reply for module %i with message length of %i", slot, length);

  if (slot < MODULES_COUNT)
    memcpy(m_internals.cmd[slot].cmd_buffer, (void *)cmd->data, length);
  mb();
  m_internals.cmd[slot].is_updated = 1;
}


static void pilot_fpga_handle_get_fpga_state_cmd(pilot_cmd_t *cmd)
{
  module_slot_t slot = target_t_get_module_slot(cmd->target);

  m_internals.modules[slot].done = (uint8_t)cmd->data[2];

  mb();
  m_internals.modules[slot].is_updated = 1;
}

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  int ret;

  switch (cmd.type)
  {
    case pilot_cmd_type_fpga_cmd:
      pilot_fpga_handle_get_fpga_cmd(&cmd);    
      ret = pilot_cmd_handler_status_handled;
      break;
    case pilot_cmd_type_fpga_state:
      pilot_fpga_handle_get_fpga_state_cmd(&cmd);
      ret = pilot_cmd_handler_status_handled;
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
  if (m_internals.is_cmd_handler_registered) 
  {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      m_internals.is_cmd_handler_registered = 0;
  }
}

/* main entry point */
module_init(pilot_fpga_init);

/*  main exit point */
module_exit(pilot_fpga_exit);
