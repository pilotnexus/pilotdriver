#include <asm/delay.h>        /* needed for udelay() function */
#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/xxx files */
#include <linux/seq_file.h>   /* needed for seq_file struct and functions */
#include <linux/slab.h>       /* needed for kmalloc() */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h"
#include "common.h"
#include "bmp.h"

MODULE_LICENSE("GPL");

#define DISPLAY_MAX_WIDTH  400
#define DISPLAY_MAX_HEIGHT 240
#define DISPLAY_BUFFERSIZE ((DISPLAY_MAX_WIDTH * DISPLAY_MAX_HEIGHT) / 8)

typedef struct {
  uint8_t data[DISPLAY_BUFFERSIZE];
} slcd_display_buffer_t;

typedef struct {
  int width;
  int height;
  volatile int is_updated;
} slcd_resolution_t;

#define RESOLUTION_GET_BYTES(r) (r.width * r.height / 8)

typedef struct {
  slcd_display_buffer_t *display_buffer;
  int has_changed;
  slcd_resolution_t resolution;
} slcd_module_t;

typedef struct {
  int driver_id;
  int is_cmd_handler_registered;
  int timeout;
  slcd_module_t modules[MODULES_COUNT];
} internals_t;

static internals_t m_internals ={.timeout=500};

static const char slcd_name[] = "slcd";
#define IS_MODULE_TYPE(m, n) (strncmp(n, m->name, strlen(n)) == 0)

static const char pilot_slcd_proc_buffer_name[] = "buffer";

static void pilot_slcd_send_display_buffer(module_slot_t slot)
{
  pilot_cmd_t cmd;
  int byte_count;
  target_t target = target_t_from_module_slot_and_port(slot, 0);

  LOG_DEBUG("called pilot_slcd_send_display_buffer()");

  /* send slcd update cmd to inform the stm of the following data stream */
  cmd.type = pilot_cmd_type_slcd_udpate;
  cmd.target = target;
  pilot_send_cmd(&cmd);

  byte_count = RESOLUTION_GET_BYTES(m_internals.modules[(int)slot].resolution);

  /* default to 128x128 resolution */
  if (byte_count == 0)
    byte_count = 128 * 128 / 8;

  //LOG_INFO("pilot_slcd_send_display_buffer() byte_count: %i", byte_count);

  /* now send the content of the display buffer */
  pilot_send(target, m_internals.modules[(int)slot].display_buffer->data, byte_count);
}

/* sends the display buffer only to the pilot if it has changed */
static void pilot_slcd_check_send_display_buffer(module_slot_t slot)
{
  LOG_DEBUG("pilot_slcd_check_send_display_buffer(slot=%i)", slot);
  if (m_internals.modules[(int)slot].has_changed)
  {
    pilot_slcd_send_display_buffer(slot);
    m_internals.modules[(int)slot].has_changed = 0;
  }
}

static int pilot_slcd_proc_buffer_show(struct seq_file *file, void *data)
{
  /* get the module index */
  int module_index = (int)file->private;

  /* write the display buffer back to the user */
  seq_write(file, m_internals.modules[module_index].display_buffer->data, DISPLAY_BUFFERSIZE);

  return 0;
}

static int pilot_slcd_proc_buffer_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_slcd_proc_buffer_show, PDE_DATA(inode));
}

static int pilot_slcd_proc_buffer_flush(struct file *file, fl_owner_t id)
{
  /* get the module slot */
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  LOG_DEBUG("called pilot_slcd_proc_buffer_flush()");

  /* check if we need to send an update for it */
  pilot_slcd_check_send_display_buffer(slot);

  return 0;
}

static ssize_t pilot_slcd_proc_buffer_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  module_slot_t slot;
  int i, ret = -EINVAL;

  LOG_DEBUG("called pilot_slcd_proc_buffer_write(count=%i, off=%i)", count, (int)*off);

  slot = (int)PDE_DATA(file->f_inode);

  /* update the internal display buffer with the written data from the user */
  for (i = 0; i < count && i < DISPLAY_BUFFERSIZE; i++)
    m_internals.modules[(int)slot].display_buffer->data[i /*+ *off*/] = buf[i];

  /* mark the display buffer as dirty */
  m_internals.modules[(int)slot].has_changed = i;

  /* return count of written bytes */
  ret = i;

  return ret;
}

static int pilot_slcd_proc_buffer_release(struct inode *inode, struct file *file)
{
  LOG_DEBUG("called pilot_slcd_proc_buffer_release()");

  /* send the display buffer to the pilot before forwarding to single_release */
  pilot_slcd_check_send_display_buffer((module_slot_t)PDE_DATA(inode));

  return single_release(inode, file);
}

/* file operations for /proc/pilot/moduleX/buffer */
static const struct file_operations proc_buffer_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_slcd_proc_buffer_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = pilot_slcd_proc_buffer_release,
  .write   = pilot_slcd_proc_buffer_write,
  .flush   = pilot_slcd_proc_buffer_flush
};

static const char pilot_slcd_proc_resolution_name[] = "resolution";

static void pilot_slcd_send_resolution(module_slot_t slot, slcd_resolution_t *res)
{
  int i;
  pilot_cmd_t cmd;
  target_t target = target_t_from_module_slot_and_port(slot, 0);

  LOG_DEBUG("called pilot_slcd_send_resolution()");
  cmd.type = pilot_cmd_type_slcd_set_resolution;
  cmd.target = target;
  for (i = 0; i < sizeof(int); i++)
    cmd.data[(int)pilot_slcd_resolution_index_width+i] = BYTE_FROM_INT(res->width, i);
  for (i = 0; i < sizeof(int); i++)
    cmd.data[(int)pilot_slcd_resolution_index_height+i] = BYTE_FROM_INT(res->height, i);

  /* send the set resolution command */
  pilot_send_cmd(&cmd);
}

static void pilot_slcd_send_get_resolution(module_slot_t slot)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, 0);
  cmd.type = pilot_cmd_type_slcd_get_resolution;
  pilot_send_cmd(&cmd);
}

static int pilot_slcd_try_get_resolution(module_slot_t slot, int timeout, slcd_resolution_t *resolution)
{
  unsigned long timestamp;
  int timedout = 0;

  /* reset the updated state */
  m_internals.modules[slot].resolution.is_updated = 0;

  /* send a request for the slcd resolution */
  pilot_slcd_send_get_resolution(slot);

  /* choose a point in time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  while (!m_internals.modules[slot].resolution.is_updated)
  {
    if (time_after(jiffies, timestamp))
    {
      timedout=1;
      break;
    }
    udelay(100);
  }

  if (!timedout)
  {
    resolution->width = m_internals.modules[slot].resolution.width;
    resolution->height =m_internals.modules[slot].resolution.height;
  }

  return timedout ? 0 : 1;
}

static int pilot_slcd_proc_resolution_show(struct seq_file *file, void *data)
{
  int count;
  char buffer[8];
  slcd_resolution_t res;
  int ret;

  /* get the module index */
  int module_index = (int)file->private;

  /* send a get resolution command */
  if (!pilot_slcd_try_get_resolution(module_index, m_internals.timeout, &res))
    ret = -EFAULT;
  else
  {
    /* create the string */
    count = snprintf(buffer, sizeof(buffer), "%ix%i", res.width, res.height);

    /* write the string to the user */
    seq_write(file, buffer, count);

    ret = 0;
  }

  return ret;
}

static int pilot_slcd_proc_resolution_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_slcd_proc_resolution_show, PDE_DATA(inode));
}

static ssize_t pilot_slcd_proc_resolution_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  slcd_resolution_t *res;
  int i, width = -1, height = -1;
  int ret = -EINVAL;

  /* get the module index */
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  /* look for the x in the resolution 128x128 */
  for (i = 0; i < count; i++)
  {
    if (buf[i] == 'x')
    {
      /* parse the width & height */
      if (kstrtoint_from_user(buf, i, 10, &width) != 0)
        width = -1;
      if (kstrtoint_from_user(buf +i+1, count-i-1, 10, &height) != 0)
        height = -1;
    }
  }

  // LOG_INFO("width: %i, height: %i", width, height);

  /* if a valid width and height were supplied, */
  if (width != -1 && height != -1)
  {
    /* accept the byte */
    ret = count;

    /* update the resolution */
    res = &m_internals.modules[slot].resolution;
    res->width = width;
    res->height = height;

    /* send a set resolution message to the pilot */
    pilot_slcd_send_resolution(slot, res);
  }

  return ret;
}

static const struct file_operations proc_resolution_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_slcd_proc_resolution_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_slcd_proc_resolution_write
};

static const char pilot_slcd_proc_bitmap_name[] = "bitmap";

static int pilot_slcd_proc_bitmap_show(struct seq_file *file, void *data)
{
  return -EFAULT;
}

static int pilot_slcd_proc_bitmap_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_slcd_proc_bitmap_show, PDE_DATA(inode));
}

static ssize_t pilot_slcd_proc_bitmap_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret, changed;
  pilot_bmp_header_t bmp_header;
  pilot_bmp_dib_header_t dib_header;
  pilot_bmp_color_table_t color_table;

  /* get the module index */
  module_slot_t slot = (int)PDE_DATA(file->f_inode);

  /* retrieve the color palette encoding from the data */

  if (pilot_bmp_try_parse_header(buf, count, &bmp_header) &&
      pilot_bmp_try_parse_dib_header(buf, count, &dib_header) &&
      pilot_bmp_try_parse_color_table(buf, count, &dib_header, &color_table))
  {
    /* update the display buffer from the bitmap */
    changed = pilot_bmp_fill_display_buffer(m_internals.modules[slot].display_buffer->data, DISPLAY_BUFFERSIZE, buf, count, &bmp_header, &dib_header, &color_table);

    /* mark the display buffer as dirty */
    m_internals.modules[(int)slot].has_changed = changed;

    LOG_DEBUG("pilot_slcd_proc_bitmap_write() file_size: %i, pixel_offset: %i, dib size: %i, image_width: %i, changed: %i, count: %i, off: %i, slot: %i",
              bmp_header.file_size, bmp_header.pixel_offset, dib_header.size, dib_header.image_width, changed, count, (int)*off, slot);

    ret = count;
  }
  else
    ret = -EINVAL;

  return ret;
}

static int pilot_slcd_proc_bitmap_release(struct inode *inode, struct file *file)
{
  LOG_DEBUG("called pilot_slcd_proc_bitmap_release()");

  /* send the display bitmap to the pilot before forwarding to single_release */
  pilot_slcd_check_send_display_buffer((module_slot_t)PDE_DATA(inode));

  return single_release(inode, file);
}

static const struct file_operations proc_bitmap_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_slcd_proc_bitmap_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = pilot_slcd_proc_bitmap_release,
  .write   = pilot_slcd_proc_bitmap_write
};


static void pilot_slcd_proc_init(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  proc_create_data(pilot_slcd_proc_buffer_name, 0666, module_dir, &proc_buffer_fops, (void*)slot);
  proc_create_data(pilot_slcd_proc_resolution_name, 0666, module_dir, &proc_resolution_fops, (void*)slot);
  proc_create_data(pilot_slcd_proc_bitmap_name, 0666, module_dir, &proc_bitmap_fops, (void*)slot);
}

static void pilot_slcd_proc_deinit(int slot)
{
  struct proc_dir_entry *module_dir = pilot_get_proc_module_dir(slot);
  remove_proc_entry(pilot_slcd_proc_buffer_name, module_dir);
  remove_proc_entry(pilot_slcd_proc_resolution_name, module_dir);
  remove_proc_entry(pilot_slcd_proc_bitmap_name, module_dir);
}

static int pilot_slcd_alloc_display_buffer(slcd_module_t *module)
{
  slcd_display_buffer_t *display_buffer;
  LOG_DEBUG("called pilot_slcd_alloc_display_buffer()");
  display_buffer = kmalloc(sizeof(slcd_display_buffer_t), GFP_KERNEL);
  if (display_buffer)
    memset(display_buffer, 0, sizeof(slcd_display_buffer_t));
  module->display_buffer = display_buffer;
  return display_buffer ? SUCCESS : -1;
}

static void pilot_slcd_dealloc_display_buffer(slcd_module_t *module)
{
  LOG_DEBUG("called pilot_slcd_dealloc_display_buffer()");
  kfree(module->display_buffer);
  module->display_buffer = NULL;
}

// *******************************************************************
// START pilot interface function implementation

static void pilot_slcd_callback_recv(module_slot_t slot, module_port_t port, spidata_t data)
{
}

static int pilot_slcd_callback_assign_slot(module_slot_t slot, const pilot_module_type_t *module_type)
{
  int slcd_type = 1;
  LOG_DEBUG("pilot_slcd_callback_assign_slot()");

  /* allocate the display buffer */
  pilot_slcd_alloc_display_buffer(&m_internals.modules[slot]);

  /* initialize it's proc files if any */
  pilot_slcd_proc_init(slot);
  
  return slcd_type;
}

static int pilot_slcd_callback_unassign_slot(module_slot_t slot)
{
  LOG_DEBUG("pilot_slcd_callback_unassign_slot()");

  /* destroy the proc devices, if any */
  pilot_slcd_proc_deinit(slot);

  /* free the display buffer */
  pilot_slcd_dealloc_display_buffer(&m_internals.modules[slot]);

  return SUCCESS;
}

static int pilot_slcd_callback_can_assign(const pilot_module_type_t *module_type)
{
  LOG_DEBUG("called pilot_slcd_callback_can_assign()");
  return IS_MODULE_TYPE(module_type, slcd_name) ? SUCCESS : -1;
}

static void pilot_slcd_handle_get_resolution_cmd(pilot_cmd_t *cmd)
{
  slcd_resolution_t *res;
  module_slot_t slot = target_t_get_module_slot(cmd->target);

  res = &m_internals.modules[slot].resolution;

  res->width = INT_FROM_BYTES((cmd->data + pilot_slcd_resolution_index_width));
  res->height = INT_FROM_BYTES((cmd->data + pilot_slcd_resolution_index_height));
  mb();
  res->is_updated = 1;
}

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  int ret;

  switch (cmd.type)
  {
    case pilot_cmd_type_slcd_get_resolution:
      pilot_slcd_handle_get_resolution_cmd(&cmd);
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

/* description of our slcd module driver */
static register_driver_t register_driver = {
  .name                          = "slcd",
  .callback_assign_slot          = pilot_slcd_callback_assign_slot,
  .callback_unassign_slot        = pilot_slcd_callback_unassign_slot,
  .callback_recv                 = pilot_slcd_callback_recv,
  .callback_can_assign           = pilot_slcd_callback_can_assign
};

/* initialization routine, called when the module is loaded */
static int __init pilot_slcd_init(void)
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

/* slcd module cleanup function, called when removing the module */
static void __exit pilot_slcd_exit(void)
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
module_init(pilot_slcd_init);

/*  main exit point */
module_exit(pilot_slcd_exit);
