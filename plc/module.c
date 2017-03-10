#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/pilot/plc files */
#include <linux/seq_file.h>   /* needs for seq_file struct and functions */
#include <asm/uaccess.h>      /* needed for copy_from_user() function */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h" /* needed for rpcp main driver functions */
#include "common.h"
#include <linux/string.h>     /* for memset() */
#include <linux/slab.h>       /* kmalloc() and kfree() */
MODULE_LICENSE("GPL");

// *******************************************************************
// START forward declaration
static int  __init pilot_plc_init(void); /* kernel module entry function */
static void __exit pilot_plc_exit(void); /* kernel module exit function */

/* forward declaration of raspicomm callback functions */
static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd);

// END forward declaration
// *******************************************************************

// *******************************************************************
// START local members

typedef struct {
  u16 min;
  u16 max;
  u16 cur;
  u16 tick;
} pilot_plc_cycletimes_t;

/* the maximum number of variables that can be configured */
#define MAX_VAR_COUNT 256
/* struct that stores the variable configuration */
typedef struct {
  u16 variables[MAX_VAR_COUNT]; /* the variable numbers */
  int count;                    /* the current count of the variables */
} pilot_plc_variables_config_t;

/* the maximum size of the variable values that can be received */
#define MAX_VAR_VALUE_SIZE 1024
typedef struct {
  char data[MAX_VAR_VALUE_SIZE]; /* the received variable values */
  int index;                     /* the current data index */
  int expected;                  /* the expected size of the variable values */
} pilot_plc_variables_values_t;

typedef struct {
  char data[MAX_VAR_VALUE_SIZE]; /* the written variable values */
  int index;                     /* the current data index */
} pilot_plc_variables_write_values_t;

#define MAX_QUALIFIER_STRING_SIZE 25
#define MAX_VAR_STRING_SIZE 50

typedef struct {
  uint32_t number;
  char resource[MAX_QUALIFIER_STRING_SIZE];
  char instance[MAX_QUALIFIER_STRING_SIZE];
  char variable[MAX_VAR_STRING_SIZE];
  char iectype;
} pilot_plc_variable_t;

pilot_plc_variable_t *variables;

typedef struct {
  pilot_plc_variables_config_t read_config;
  pilot_plc_variables_config_t write_config;
  pilot_plc_variables_values_t read_values;
  pilot_plc_variables_write_values_t write_value;
  volatile int is_values_updated;
} pilot_plc_variables_t;

/* struct that groups internal members */
typedef struct {
  int is_cmd_handler_registered; /* set to 1 if the cmd handler is registered with the rpcp */
  int is_read_stream_handler_registered; /* set to 1 if the read stream handler is registered with the rpcp */
  volatile int is_state_updated;
  volatile pilot_plc_state_t state;
  volatile int is_cycletimes_updated;
  volatile pilot_plc_cycletimes_t cycletimes;
  struct proc_dir_entry *proc_pilot_dir;
  struct proc_dir_entry *proc_pilot_plc_dir;
  struct proc_dir_entry *proc_pilot_plc_vars_dir;
  pilot_plc_variables_t variables;
} internals_t;

/* internal variables */
static internals_t _internals = {
  .is_cmd_handler_registered = 0,
  .is_state_updated = 0,
  .is_cycletimes_updated = 0
};

/* rpcp struct with callback functions for the main rpcp driver */
static pilot_cmd_handler_t pilot_cmd_handler = {
  .callback_cmd_received = pilot_callback_cmd_received
};

// END local members
// *******************************************************************

static const char proc_plc_name[] = "plc";                       /* name of the plc directory in /proc/pilot/plc */
static const char proc_plc_state_name[] = "state";               /* name of the state file in /proc/pilot/plc/state */
static const char proc_plc_cycletimes_name[] = "cycletimes";     /* name of the cycletimes file in /proc/pilot/plc/cycletimes */
static const char proc_plc_varconfig_name[] = "varconf";         /* name of the variable config directory in /proc/pilot/plc/varconf */
static const char proc_plc_vars_name[] = "vars";                 /* name of the vars directory in /proc/pilot/plc/vars */
static const char proc_plc_vars_readconfig_name[] = "readcfg";   /* name of the read config file in /proc/pilot/plc/varconf/readcfg */
static const char proc_plc_vars_writeconfig_name[] = "writecfg"; /* name of the write config file in /proc/pilot/plc/varconf/writecfg */
static const char proc_plc_vars_value_name[] = "value";          /* name of the value file in /proc/pilot/plc/varconf/value */
static const char proc_plc_variables_name[] = "variables";       /* name of the variables.csv file in /proc/pilot/plc/varconf/value */


// *******************************************************************
// START state

static void pilot_plc_send_set_state_cmd(pilot_plc_state_t state)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_state_set;
  cmd.data[(int)pilot_plc_state_index] = state;
  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_get_state_cmd(void)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_state_get;
  pilot_send_cmd(&cmd);
}

static int pilot_plc_try_get_state(int timeout, pilot_plc_state_t *state)
{
  int is_timedout = 0;
  unsigned long timestamp;

  /* reset the is state updated flag */
  _internals.is_state_updated = 0;

  /* set the plc state get request */
  pilot_plc_send_get_state_cmd();

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (_internals.is_state_updated == 0) 
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* read the updated state */
  if (!is_timedout)
    *state = _internals.state;
  else
  {
    //LOG_INFO("pilot_plc_try_get_state() timedout while waiting for plc state!");
  }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_plc_proc_state_show(struct seq_file *file, void *data)
{
  int ret;
  pilot_plc_state_t state;

  if (pilot_plc_try_get_state(100, &state) == SUCCESS)
  {
    seq_printf(file, "%i", (int)state);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_plc_proc_state_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_state_show, PDE_DATA(inode));
}

static int pilot_plc_proc_state_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, ret;

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL; /* return an error if the conversion fails */
  else
  {
    /* send a plc state set cmd to the pilot */
    if (new_value == 0){
      pilot_plc_send_set_state_cmd(pilot_plc_state_stop);
    }
    else if (new_value == 1) {
      pilot_plc_send_set_state_cmd(pilot_plc_state_run);
    }

    ret = count; /* we processed the complete input */
  }
  return ret;
}

/* file operations for the /proc/pilot/plc/state */
static const struct file_operations proc_plc_state_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_state_open,
  .read = seq_read,
  .write = pilot_plc_proc_state_write,
  .llseek = seq_lseek,
  .release = single_release
};

static int pilot_plc_proc_variables_state_show(struct seq_file *file, void *data)
{
  int ret;

  seq_printf(file, "blah");
  ret = 0;

  return ret;
}

static int pilot_plc_proc_variables_state_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_variables_state_show, PDE_DATA(inode));
}

static int pilot_plc_proc_variables_state_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int newlines = 0, i=0;

  //estimate maximum needed variable structs
  for (i = 0; i < count; ++i)
    if (buf[i] == '\r')
      newlines++;

  LOG_DEBUG("writing variable list, length: %i, lines %i", count, newlines);

  variables = kmalloc(newlines * sizeof(pilot_plc_variable_t));

  return count;
}



/* file operations for the /proc/pilot/plc/vars/variables */
static const struct file_operations proc_plc_variables_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_variables_state_open,
  .read = seq_read,
  .write = pilot_plc_proc_variables_state_write,
  .llseek = seq_lseek,
  .release = single_release
};

// END state
// *******************************************************************

// *******************************************************************
// START cycletimes

static void pilot_plc_send_get_cycletimes_cmd(void)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_cycletimes_get;
  pilot_send_cmd(&cmd);
}

static int pilot_plc_try_get_cycletimes(int timeout, pilot_plc_cycletimes_t *cycletimes)
{
  int is_timedout = 0;
  unsigned long timestamp;

  /* reset the is cycletimes updated flag */
  _internals.is_cycletimes_updated = 0;

  /* set the plc cycletimes get request */
  pilot_plc_send_get_cycletimes_cmd();

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the cycletime is updated or the timeout occurs */
  while (_internals.is_cycletimes_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* read the updated cycletimes */
  if (!is_timedout)
    *cycletimes = _internals.cycletimes;
  else
  {
    //LOG_INFO("pilot_plc_try_get_cycletimes() timedout while waiting for plc cycletime!");
  }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_plc_proc_cycletimes_show(struct seq_file *file, void *data)
{
  int ret;
  pilot_plc_cycletimes_t cycletimes;

  if (pilot_plc_try_get_cycletimes(100, &cycletimes) == SUCCESS)
  {
    seq_printf(file, "%i\n%i\n%i\n%i\n", cycletimes.min, cycletimes.max, cycletimes.cur, cycletimes.tick);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_plc_proc_cycletimes_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_cycletimes_show, PDE_DATA(inode));
}

/* file operations for /proc/pilot/plc/cycletimes */
static const struct file_operations proc_plc_cycletimes_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_cycletimes_open,
  .read = seq_read,
  .release = single_release
};

// END cycletimes
// *******************************************************************

// *******************************************************************
// START variable read config

static void pilot_plc_send_variables_readconfig_cmd(int count)
{
  int i;
  pilot_cmd_t cmd;

  /* create the variables config cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variables_read_config;
  for (i = 0; i < sizeof(int); i++)
    cmd.data[(int)pilot_plc_variables_config_index_count + i] = BYTE_FROM_INT(count, i);

  /* send the command */
  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_variables_readconfig_numbers(void)
{
  //LOG_INFO("pilot_plc_send_variables_config_numbers() sending %i bytes", sizeof(u16) * _internals.variables.config.count);
  /* send all variable numbers as a blob */
  pilot_send(target_plc_read, (char*) _internals.variables.read_config.variables, sizeof(u16) * _internals.variables.read_config.count);
}

static void pilot_plc_check_send_variables_readconfig(void)
{
  int count = _internals.variables.read_config.count;

  /* check if we need to send the variable config */
  if (count > 0)
  {
    //LOG_INFO("pilot_plc_check_send_variables_config() sends %i vars", count);

    /* send the variables config cmd to the pilot */
    pilot_plc_send_variables_readconfig_cmd(count);

    /* send the variable contents to the pilot */
    pilot_plc_send_variables_readconfig_numbers();

    /* reset the count back to zero */
    _internals.variables.read_config.count = 0;
  }
}

static int pilot_plc_proc_vars_readconfig_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret;
  u16 number;  /* buffer for kstrtoX */
  int i, start, length, var_index;
  const char sep = ' ';
  char buffer[8];
  memset(buffer, 0, sizeof(buffer));
  start = 0;
  var_index = 0;

  //LOG_INFO("count: %i", count);

  /* test the user input */
  for (i = 0; i < count; i++)
  {
    if (buf[i+1] == sep || /* if the next char is a separator */
            i+1  == count)    /* or it's the last char */
    {
      length = i+1 - start;

      /* if it fits into our buffer */
      if (length < sizeof(buffer))
      {
        //LOG_INFO("copying start %i and length %i", start, length);
        if (copy_from_user(buffer, buf + start, length) == SUCCESS)
        {
          /* store the numbers */
          if (kstrtou16(buffer, 10, &number) == SUCCESS)
          {
            //LOG_INFO("configured variable nr: %i", number);
            _internals.variables.read_config.variables[var_index++] = number;
            _internals.variables.read_config.count = var_index;
          }
        }
      }

      /* skip the next char as it is a separator */
      i++;

      start = i+1;
    }
  }

  ret = count;

  return ret;
}

static int pilot_plc_proc_vars_readconfig_show(struct seq_file *file, void *data)
{
  return -EFAULT;
}

static int pilot_plc_proc_vars_readconfig_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_vars_readconfig_show, PDE_DATA(inode));
}

static int pilot_plc_proc_vars_readconfig_flush(struct file *file, fl_owner_t id)
{
  pilot_plc_check_send_variables_readconfig();
  return 0;
}

static int pilot_plc_proc_vars_readconfig_release(struct inode *inode, struct file *file)
{
  pilot_plc_check_send_variables_readconfig();
  return single_release(inode, file);
}

static const struct file_operations proc_plc_vars_readconfig_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_vars_readconfig_open,
  .read = seq_read,
  .write = pilot_plc_proc_vars_readconfig_write,
  .flush = pilot_plc_proc_vars_readconfig_flush,
  .release = pilot_plc_proc_vars_readconfig_release
};

// END variable read config
// *******************************************************************

// *******************************************************************
// START variable write config

static void pilot_plc_send_variables_writeconfig_cmd(int count)
{
  int i;
  pilot_cmd_t cmd;

  /* create the variables config cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variables_write_config;
  for (i = 0; i < sizeof(int); i++)
    cmd.data[(int)pilot_plc_variables_config_index_count + i] = BYTE_FROM_INT(count, i);

  /* send the command */
  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_variables_writeconfig_numbers(void)
{
  //LOG_INFO("pilot_plc_send_variables_config_numbers() sending %i bytes", sizeof(u16) * _internals.variables.config.count);
  /* send all variable numbers as a blob */
  pilot_send(target_plc_write, (char*)_internals.variables.write_config.variables, sizeof(u16) * _internals.variables.write_config.count);
}

static void pilot_plc_check_send_variables_writeconfig(void)
{
  int count = _internals.variables.write_config.count;

  /* check if we need to send the variable config */
  if (count > 0)
  {
    //LOG_INFO("pilot_plc_check_send_variables_config() sends %i vars", count);

    /* send the variables config cmd to the pilot */
    pilot_plc_send_variables_writeconfig_cmd(count);

    /* send the variable contents to the pilot */
    pilot_plc_send_variables_writeconfig_numbers();

    /* reset the count back to zero */
    _internals.variables.write_config.count = 0;
  }
}

static int pilot_plc_proc_vars_writeconfig_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret, i, start, length, var_index, is_forced;
  u16 number;
  const char sep = ' ';
  const char force = '!';
  char buffer[8];
  memset(buffer, 0, sizeof(buffer)); /* buffer for kstrtoX */
  start = 0;
  var_index = 0;

  //LOG_INFO("count: %i", count);

  /* test the user input */
  for (i = 0; i < count; i++)
  {
    if ((is_forced = ((buf[i + 1] == force) && /* is forced if the next char is forced and */
      (i + 2 == count || /* and it's the end of the string or */
      buf[i + 2] == sep || buf[i + 2] == '\n'))) || /* or the char after that is a separator or line feed */
      buf[i + 1] == sep ||   /* if the next char is a separator */
      i + 1 == count)        /* or it's the last char */
    {
      length = i + 1 - start;

      /* if it fits into our buffer */
      if (length < sizeof(buffer))
      {
        //LOG_INFO("copying start %i and length %i", start, length);
        if (copy_from_user(buffer, buf + start, length) == SUCCESS)
        {
          /* store the numbers */
          if (kstrtou16(buffer, 10, &number) == SUCCESS)
          {
            LOG_DEBUG("configured variable nr: %i, is_forced: %i, start: %i, length: %i", number, is_forced, start, length);

            _internals.variables.write_config.variables[var_index++] = VAR_TO_UINT16(number, is_forced);
            _internals.variables.write_config.count = var_index;
          }
          else
          {
            LOG_DEBUG("error parsing variable! is_forced: %i, start: %i, length: %i", is_forced, start, length);
          }
        }
      }

      if (is_forced)
        i++;

      /* skip the next char */
      i++;

      start = i + 1;
    }
  }

  ret = count;

  return ret;
}

static int pilot_plc_proc_vars_writeconfig_show(struct seq_file *file, void *data)
{
  return -EFAULT;
}

static int pilot_plc_proc_vars_writeconfig_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_vars_writeconfig_show, PDE_DATA(inode));
}

static int pilot_plc_proc_vars_writeconfig_flush(struct file *file, fl_owner_t id)
{
  pilot_plc_check_send_variables_writeconfig();
  return 0;
}

static int pilot_plc_proc_vars_writeconfig_release(struct inode *inode, struct file *file)
{
  pilot_plc_check_send_variables_writeconfig();
  return single_release(inode, file);
}

static const struct file_operations proc_plc_vars_writeconfig_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_vars_writeconfig_open,
  .read = seq_read,
  .write = pilot_plc_proc_vars_writeconfig_write,
  .flush = pilot_plc_proc_vars_writeconfig_flush,
  .release = pilot_plc_proc_vars_writeconfig_release
};

// END variable write config
// *******************************************************************

// *******************************************************************
// START variable value

static void pilot_plc_send_get_variable_values_cmd(void)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variables_get;
  pilot_send_cmd(&cmd);
}

static int pilot_plc_try_get_variable_values(int timeout, pilot_plc_variables_values_t **values)
{
  int is_timedout = 0;
  unsigned long timestamp;

  /* reset the variables update */
  _internals.variables.is_values_updated = 0;

  /* send a get variable values request to the pilot */
  pilot_plc_send_get_variable_values_cmd();

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (_internals.variables.is_values_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* set the variables */
  if (!is_timedout)
    *values = &_internals.variables.read_values;

  if (is_timedout)
  {
    //LOG_INFO("pilot_plc_try_get_vars() timedout while waiting for variable values!");
  }

  /* return 0 on success, otherwise 1 */
  return is_timedout;
}

static int pilot_plc_proc_vars_value_show(struct seq_file *file, void *data)
{
  int ret;
  pilot_plc_variables_values_t *values;

  if (pilot_plc_try_get_variable_values(100, &values) == SUCCESS)
  {
    seq_write(file, values->data, values->index);
    ret = 0;
  }
  else
  {
    ret = -EFAULT;
  }

  return ret;
}

static int pilot_plc_proc_vars_value_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_vars_value_show, PDE_DATA(inode));
}

static int pilot_plc_proc_vars_value_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int i;

  for (i = 0; i < count; i++)
  {
    /* store the data in the internal buffer, if there is space left */
    if (_internals.variables.write_value.index < MAX_VAR_VALUE_SIZE)
      _internals.variables.write_value.data[_internals.variables.write_value.index++] = buf[i];
    else
      return -EINVAL; /* return an error if the buffer is full */
  }

  return count;
}

static void pilot_plc_send_vars_value_cmd(void)
{
  int i;
  pilot_cmd_t cmd;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_plc_write;
  cmd.type = pilot_cmd_type_plc_variables_set;
  for (i = 0; i < sizeof(int); i++)
    cmd.data[(int)pilot_plc_variables_value_index_size + i] = BYTE_FROM_INT(_internals.variables.write_value.index, i);

  /* send the cmd */
  pilot_send_cmd(&cmd);
}

static void pilot_plc_check_send_vars_value(void)
{
  /* check if we have data to send */
  if (_internals.variables.write_value.index > 0)
  {
    /* send the set variable values cmd */
    pilot_plc_send_vars_value_cmd();

    /* send the variable values */
    pilot_send(target_plc_write, _internals.variables.write_value.data, _internals.variables.write_value.index);

    /* reset the count */
    _internals.variables.write_value.index = 0;
  }
}

static int pilot_plc_proc_vars_value_flush(struct file *file, fl_owner_t id)
{
  pilot_plc_check_send_vars_value();
  return 0;
}

static int pilot_plc_proc_vars_value_release(struct inode *inode, struct file *file)
{
  pilot_plc_check_send_vars_value();
  return single_release(inode, file);
}

static const struct file_operations proc_plc_vars_value_fops = {
  .owner = THIS_MODULE,
  .open = pilot_plc_proc_vars_value_open,
  .read = seq_read,
  .write = pilot_plc_proc_vars_value_write,
  .flush = pilot_plc_proc_vars_value_flush,
  .release = pilot_plc_proc_vars_value_release
};

// END variable value
// *******************************************************************

/* register the /proc/pilot/plc/... files */
static void pilot_plc_proc_init(void)
{
  struct proc_dir_entry *pilot_dir, *plc_dir, *vars_config_dir, *vars_dir;

  /* get the /proc/pilot base dir */
  _internals.proc_pilot_dir = pilot_dir = pilot_get_proc_pilot_dir();

  /* create the /proc/pilot/plc directory */
  _internals.proc_pilot_plc_dir = plc_dir = proc_mkdir_mode(proc_plc_name, 0, pilot_dir);

  /* create the file /proc/pilot/plc/state (r/w) */
  proc_create_data(proc_plc_state_name, 0666, plc_dir, &proc_plc_state_fops, NULL);

  /* create the file /proc/pilot/plc/cycletimes (read-only) */
  proc_create_data(proc_plc_cycletimes_name, 0, plc_dir, &proc_plc_cycletimes_fops, NULL);

  /* create the /proc/pilot/plc/varconfig directory */
  _internals.proc_pilot_plc_vars_dir = vars_config_dir = proc_mkdir_mode(proc_plc_varconfig_name, 0, plc_dir);

  /* create the file /proc/pilot/plc/varconfig/read_config (w) */
  proc_create_data(proc_plc_vars_readconfig_name, 0222, vars_config_dir, &proc_plc_vars_readconfig_fops, NULL);

  /* create the file /proc/pilot/plc/varconfig/write_config (w) */
  proc_create_data(proc_plc_vars_writeconfig_name, 0222, vars_config_dir, &proc_plc_vars_writeconfig_fops, NULL);

  /* create the file /proc/pilot/plc/varconfig/value (r) */
  proc_create_data(proc_plc_vars_value_name, 0666, vars_config_dir, &proc_plc_vars_value_fops, NULL);

  /* create the file /proc/pilot/plc/varconfig/variables (r/w) */
  proc_create_data(proc_plc_variables_name, 0666, vars_config_dir, &proc_plc_variables_fops, NULL);
}

/* unregister the /proc/pilot/plc/... files */
static void pilot_plc_proc_deinit(void)
{
  /* remove the file /proc/pilot/plc/state */
  remove_proc_entry(proc_plc_state_name, _internals.proc_pilot_plc_dir);

  /* remove the file /proc/pilot/plc/cycletimes */
  remove_proc_entry(proc_plc_cycletimes_name, _internals.proc_pilot_plc_dir);

  /* remove the file /proc/pilot/plc/varconfig/read_config */
  remove_proc_entry(proc_plc_vars_readconfig_name, _internals.proc_pilot_plc_vars_dir);

  /* remove the file /proc/pilot/plc/varconfig/write_config */
  remove_proc_entry(proc_plc_vars_writeconfig_name, _internals.proc_pilot_plc_vars_dir);

  /* remove the file /proc/pilot/plc/varconfig/value */
  remove_proc_entry(proc_plc_vars_value_name, _internals.proc_pilot_plc_vars_dir);

  /* remove the directory /proc/pilot/plc/varconfig */
  remove_proc_entry(proc_plc_varconfig_name, _internals.proc_pilot_plc_dir);

  /* remove the directory /proc/pilot/plc */
  remove_proc_entry(proc_plc_name, _internals.proc_pilot_dir);

  /* create the file /proc/pilot/plc/varconfig/variables (r/w) */
  remove_proc_entry(proc_plc_variables_name, _internals.proc_pilot_plc_vars_dir);
}

// *******************************************************************
// START pilot interface function implementation

/* the command callback handler */
static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  pilot_cmd_handler_status_t ret;
  LOG_DEBUG("pilot_callback_cmd_received() called");

  switch (cmd.type)
  {
    /* we're receiving the answer to the plc_state_get command */
  case pilot_cmd_type_plc_state_get:
    _internals.state = (pilot_plc_state_t)cmd.data[(int)pilot_plc_state_index];
    mb();
    _internals.is_state_updated = 1;

    LOG_DEBUG("pilot_callback_cmd_received() received plc_state_get answer");
    /* mark the command as handled */
    ret = pilot_cmd_handler_status_handled;
    break;

  case pilot_cmd_type_plc_cycletimes_get:
    _internals.cycletimes.min = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_min));
    _internals.cycletimes.max = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_max));
    _internals.cycletimes.cur = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_cur));
    _internals.cycletimes.tick = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_tick));
    mb();
    _internals.is_cycletimes_updated = 1;

    LOG_DEBUG("pilot_callback_cmd_received() received plc_cycletimes_get answer");
    /* mark the command as handled */
    ret = pilot_cmd_handler_status_handled;
    break;

  case pilot_cmd_type_plc_variables_get:
    _internals.variables.read_values.index = 0;
    _internals.variables.read_values.expected = INT_FROM_BYTES((cmd.data + (int)pilot_plc_variables_value_index_size));
    //LOG_INFO("pilot_callback_cmd_received() received plc_variables_get answer");
    ret = pilot_cmd_handler_status_handled;
    break;

  default: ret = pilot_cmd_handler_status_ignored;  break;
  }

  return ret;
}

/* the read stream handler */
static void pilot_plc_read_stream_callback(char data)
{
  //LOG_INFO("pilot_plc_stream_callback() recv: %x", data);

  /* handle the received data */
  if (_internals.variables.read_values.index < MAX_VAR_VALUE_SIZE)
  {
    _internals.variables.read_values.data[_internals.variables.read_values.index++] = data;
    if (_internals.variables.read_values.index >= _internals.variables.read_values.expected)
      _internals.variables.is_values_updated = 1;
  }
}

// END pilot interface function implementation
// *******************************************************************

/* main entry point */
module_init(pilot_plc_init);

/* main exit point */
module_exit(pilot_plc_exit);


/* initialization routine, called when the module is loaded */
static int __init pilot_plc_init()
{
  int ret = -1;

  LOG_DEBUG("pilot_plc_init()");

  variables = NULL; //init double pointer

  /* register the filesystem entries */
  pilot_plc_proc_init();

  /* register with the base driver */
  if (pilot_register_cmd_handler(&pilot_cmd_handler) == SUCCESS)
  {
    LOG_DEBUG("pilot_register_cmd_handler() succeeded");
    _internals.is_cmd_handler_registered = 1;

    pilot_register_stream_handler(target_plc_read, &pilot_plc_read_stream_callback);
    _internals.is_read_stream_handler_registered = 1;

    ret = SUCCESS;
  }
  else
  {
    LOG(KERN_ERR, "pilot_register_cmd_handler() failed!");
  }

  return ret;
}

/* plc module cleanup function, called when removing the module */
static void __exit pilot_plc_exit()
{
  LOG_DEBUG("pilot_plc_exit() called");

  if (variables)
    kfree(variables);

  /* unregister with the base driver */
  if (_internals.is_cmd_handler_registered) {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      _internals.is_cmd_handler_registered = 0;
  }

  if (_internals.is_read_stream_handler_registered) {
    pilot_unregister_stream_handler(target_plc_read);
    _internals.is_read_stream_handler_registered = 0;
  }

  /* unregister the filesystem entries */
  pilot_plc_proc_deinit();
}

