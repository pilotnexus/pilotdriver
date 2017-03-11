#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/pilot/plc files */
#include <linux/seq_file.h>   /* needs for seq_file struct and functions */
#include <asm/uaccess.h>      /* needed for copy_from_user() function */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h" /* needed for rpcp main driver functions */
#include "common.h"
#include <linux/string.h>     /* for memset() */
#include <linux/slab.h>       /* kmalloc() and kfree() */
#include <linux/gfp.h>        /* flags for kmalloc() */
#include <asm/uaccess.h>      /* copy_to_user() */

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

#define MAX_CSV_COLS 6
#define MAX_VAR_COLS 6


enum iectypes {
  IEC_NONE = -1,
  IEC_BOOL,
  IEC_SINT,
  IEC_INT,
  IEC_DINT,
  IEC_LINT,
  IEC_USINT,
  IEC_UINT,
  IEC_UDINT,
  IEC_ULINT,
  IEC_BYTE,
  IEC_WORD,
  IEC_DWORD,
  IEC_LWORD,
  IEC_REAL,
  IEC_LREAL
};

enum iecvarclass {
  NONE = -1,
  VAR,
  EXT
};

typedef struct {
  int number;
  enum iecvarclass varclass;
  enum iectypes iectype;
  volatile int is_variable_updated;
  char value[8];
  char *variable;
} pilot_plc_variable_t;

typedef struct {
  pilot_plc_variables_config_t read_config;
  pilot_plc_variables_config_t write_config;
  pilot_plc_variables_values_t read_values;
  pilot_plc_variables_write_values_t write_value;
  volatile int is_values_updated;
} pilot_plc_variables_t;

/* struct that groups internal members */
typedef struct {
  volatile int is_cmd_handler_registered; /* set to 1 if the cmd handler is registered with the rpcp */
  volatile int is_read_stream_handler_registered; /* set to 1 if the read stream handler is registered with the rpcp */
  volatile int is_state_updated;
  volatile pilot_plc_state_t state;
  volatile int is_cycletimes_updated;
  volatile pilot_plc_cycletimes_t cycletimes;
  struct proc_dir_entry *proc_pilot_dir;
  struct proc_dir_entry *proc_pilot_plc_dir;
  struct proc_dir_entry *proc_pilot_plc_vars_dir;
  struct proc_dir_entry *proc_pilot_plc_varconfig_dir;
  pilot_plc_variables_t variablestream;
  pilot_plc_variable_t **variables;
  int variables_count;
} internals_t;

/* internal variables */
static internals_t _internals = {
  .is_cmd_handler_registered = 0,
  .is_state_updated = 0,
  .is_cycletimes_updated = 0,
  .variables = NULL,
  .variables_count = 0,
  .proc_pilot_plc_vars_dir = NULL //init vars dir with NULL
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
// START VARIABLE.csv funtions

static void pilot_plc_send_set_variable_cmd(uint16_t varnumber)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variable_set;

  cmd.data[8] = varnumber & 0xFF; 
  cmd.data[9] = varnumber >> 8; 

  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_get_variable_cmd(uint16_t varnumber)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variable_get;

  cmd.data[8] = varnumber & 0xFF; 
  cmd.data[9] = varnumber >> 8; 
  
  pilot_send_cmd(&cmd);
}

static int pilot_plc_try_get_variable(int timeout, pilot_plc_variable_t *variable)
{
  int is_timedout = 0;
  unsigned long timestamp;

  /* reset the variable updated flag */
  variable->is_variable_updated = 0;

  /* set the plc state get request */
  pilot_plc_send_get_variable_cmd(variable->number);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (variable->is_variable_updated == 0) 
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* read the updated state */
  if (!is_timedout)
  {
    LOG_DEBUG("variable %i successfully received from pilot\n", variable->number);
  }
  else
  {
    //LOG_INFO("pilot_plc_try_get_state() timedout while waiting for plc state!");
    LOG_DEBUG("variable %i timeout getting variable\n", variable->number);
  }

  return is_timedout ? -1 : SUCCESS;
}

int readlinedelimited(const char *buf, int *index, int count, const char delimiter, int *col_start, int *col_length, int max_cols)
{
  int col = 0;
  char state = 0; //0 = find col start, 1 = find col end
  //int col_start[MAX_CSV_COLS];
  //int col_length[MAX_CSV_COLS];
  
  for (; (*index) < count; (*index)++)
  {
    //first check if this is the end of the line
    if (buf[(*index)] == '\r' || buf[(*index)] == '\n')
      break;

    if (state == 0) //find col start
    {
      if (buf[(*index)] != ' ')
      {
        //found start
        col_start[col] = (*index); //column start
        state = 1;
      }
    }
    else if (state == 1) //find col end
    {
      //end of column
      if (buf[(*index)] == delimiter || buf[(*index)] == ' ')
      {
        col_length[col] = (*index) - col_start[col];

        if (col + 1 < MAX_CSV_COLS)
        {
          col++;
          state = 0; //find col start again
        }
        else
          break; //analyze, maximum columns reached
      }
    }
  }

  //define length if in column (missing delimiter at end)
  if (state == 1)
    col_length[col] =(*index) - col_start[col];

  (*index)++; //advance to the next char so this function can be used in a loop directly
  return col;
}

enum iectypes strToIECVar(const char* str, int len)
{
  enum iectypes var = IEC_NONE;

  if (strncmp(str, "BOOL", len) == 0)
    var = IEC_BOOL;
  else if (strncmp(str, "SINT", len) == 0)
    var = IEC_SINT;
  else if (strncmp(str, "INT", len) == 0)
    var = IEC_INT;
  else if (strncmp(str, "DINT", len) == 0)
    var = IEC_DINT;
  else if (strncmp(str, "LINT", len) == 0)
    var = IEC_LINT;
  else if (strncmp(str, "USINT", len) == 0)
    var = IEC_USINT;
  else if (strncmp(str, "UINT", len) == 0)
    var = IEC_UINT;
  else if (strncmp(str, "UDINT", len) == 0)
    var = IEC_UDINT;
  else if (strncmp(str, "ULINT", len) == 0)
    var = IEC_ULINT;
  else if (strncmp(str, "BYTE", len) == 0)
    var = IEC_BYTE;
  else if (strncmp(str, "WORD", len) == 0)
    var = IEC_WORD;
  else if (strncmp(str, "DWORD", len) == 0)
    var = IEC_DWORD;
  else if (strncmp(str, "LWORD", len) == 0)
    var = IEC_LWORD;
  else if (strncmp(str, "REAL", len) == 0)
    var = IEC_REAL;
  else if (strncmp(str, "LREAL", len) == 0)
    var = IEC_LREAL;

  return var;
}

int toInt(const char *a, int length) {
  int c, n;
  n = 0;

  for (c = 0; c < length; c++) {
    if (a[c] < 0x30 && a[c] > 0x39)
      return -1;

    n = n * 10 + a[c] - '0';
  }
  return n;
}

int analyzecsvbuffer(const char *buf, int fsize, void(*handleline)(int , enum iecvarclass, enum iectypes, const char *, int ))
{
  int index = 0;
  //int lines = 0;
  int maxnumber = 0;
  int col_start[MAX_CSV_COLS];
  int col_length[MAX_CSV_COLS];

  while (index < fsize)
  {
    int cols = readlinedelimited(buf, &index, fsize, ';', col_start, col_length, MAX_CSV_COLS);
    if (cols >= 5)
    {
      int number = 0;
      enum iecvarclass type = NONE; //0 = VAR, 1 = EXT
      enum iectypes iecVar;

      { //ok, analyze

        number = toInt(&buf[col_start[0]], col_length[0]);
        //check if var class is ok
        if (strncmp(&buf[col_start[1]], "VAR", col_length[1]) == 0) //ok
          type = VAR;
        else if (strncmp(&buf[col_start[1]], "EXT", col_length[1]) == 0) //also ok
          type = EXT;

        //check iec var
        iecVar = strToIECVar(&buf[col_start[cols - 1]], col_length[cols - 1]);

        if (number >= 0 && type != -1 && iecVar >= 0)
        {
          //process var
          int var_col_start[MAX_VAR_COLS];
          int var_col_length[MAX_VAR_COLS];
          int varindex = 0;
          if (readlinedelimited(&buf[col_start[2]], &varindex, col_length[2], '.', var_col_start, var_col_length, MAX_VAR_COLS) >= 2)
          {
            if (handleline != NULL)
              handleline(number, type, iecVar, &buf[col_start[2]], col_length[2]);
            if (maxnumber < number)
              maxnumber = number;
          }
        }
      }
    }
  }
  return maxnumber;
}

void debug_print_var_line(int number, enum iecvarclass type, enum iectypes iecvar, const char *variable, int varLength)
{
  //add debug if wanted
  LOG_DEBUG("variable nr: %i:  %.*s\n", number, varLength, variable);

}

/*
int variable_read_proc(struct file *filp,char *buf,size_t count,loff_t *offp ) 
{
  int ret = 0;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file_inode(filp));

  if (variable)
  {
    if (copy_to_user(buf,"OK", 2))
      ret = -EFAULT;

    LOG_DEBUG("variable %i requested. buffer count: %i\n", variable->number, count);

    if (pilot_plc_try_get_variable(100, variable) == SUCCESS)
    {
      LOG_DEBUG("variable %i value received (%x %x %x %x %x %x %x %x)\n", variable->number, variable->value[0], variable->value[1], variable->value[2], variable->value[3], variable->value[4], variable->value[5], variable->value[6], variable->value[7] );
    }
    return 0; //
  }
  else
    ret = -EFAULT; 
  //else
  //{
    //seq_printf(file, "this is variable number %i", variable->number);
  //  ret = 0;
  //}

  return ret;
}
*/

static int pilot_plc_proc_var_show(struct seq_file *file, void *data)
{
  int ret = 0;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)file->private;
  if (variable)
  {
    if (pilot_plc_try_get_variable(100, variable) == SUCCESS)
    {
      seq_printf(file, "%i", *((uint32_t*)variable->value)); 
    }
  }
  else
  {
     ret = -EFAULT;
  }

  return ret;
}

static int pilot_plc_proc_var_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_var_show, PDE_DATA(inode));
}



static int pilot_plc_proc_var_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  return count;
}

/* file operations for the /proc/pilot/plc/vars/variables */
static const struct file_operations proc_plc_variable_fops = {
  .owner = THIS_MODULE,
  //.read = variable_read_proc,
  .open = pilot_plc_proc_var_open,
  .read = seq_read,
  .write = pilot_plc_proc_var_write,
  .llseek = seq_lseek,
  .release = single_release
};

void malloc_var(int number, enum iecvarclass type, enum iectypes iecvar, const char *variable, int varLength)
{
  int i;
  //add debug if wanted
  _internals.variables[number] = (pilot_plc_variable_t *)kzalloc(sizeof(pilot_plc_variable_t), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
  _internals.variables[number]->number = number;
  _internals.variables[number]->varclass = type;
  _internals.variables[number]->iectype = iecvar;
  _internals.variables[number]->is_variable_updated = 0;
  _internals.variables[number]->variable = (char*)kzalloc(varLength+1 * sizeof(char), __GFP_NOFAIL | __GFP_IO | __GFP_FS);

  for (i = 0; i < varLength; i++)
  {
    _internals.variables[number]->variable[i] = (variable[i] > 0x40) && (variable[i] < 0x5B) ? variable[i] + 0x20 : variable[i];
  }

  _internals.variables[number]->variable[i] = 0;


  //now create file
  if (_internals.proc_pilot_plc_vars_dir != NULL)
  {
    /* create the file /proc/pilot/plc/varconfig/variables (r/w) */
    LOG_DEBUG("creating variable %i file %s", _internals.variables[number]->number, _internals.variables[number]->variable);
    proc_create_data(_internals.variables[number]->variable, 0666, _internals.proc_pilot_plc_vars_dir, &proc_plc_variable_fops, _internals.variables[number]);
  }
}

void freevariables(void)
{
  int i = 0;
  LOG_DEBUG("freevariables()");

  //free vars
  for (i = 0; i < _internals.variables_count; i++)
  {
  //remove file
  /* remove the file /proc/pilot/plc/state */
    if (_internals.variables[i] != NULL)
    {
      kfree((void *)_internals.variables[i]->variable);
      kfree((void *)_internals.variables[i]);
    }
  }

  _internals.variables_count = 0;

  if (_internals.variables != NULL)
  {
      kfree((void *)_internals.variables);

      LOG_DEBUG("removing vars directory");
      remove_proc_subtree(proc_plc_vars_name, _internals.proc_pilot_plc_dir);
      _internals.proc_pilot_plc_vars_dir = NULL; //set emnpty
  }

  _internals.variables = NULL;

  LOG_DEBUG("freevariables() done");
}
// END VARIABLE.csv functions
// *******************************************************************

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
  //first remove all variables if existent
  freevariables();

  _internals.variables_count = analyzecsvbuffer(buf, count, debug_print_var_line) + 1; //read variables and store max var number plus one (var numbers are zero based)

  if (_internals.variables_count > 0)
  {
    LOG_DEBUG("creating vars directory");
    _internals.proc_pilot_plc_vars_dir = proc_mkdir_mode(proc_plc_vars_name, 0, _internals.proc_pilot_plc_dir);

    //initialize pointer array
    _internals.variables = (pilot_plc_variable_t **)kzalloc(_internals.variables_count * sizeof(pilot_plc_variable_t *), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
  
    //define vars
    analyzecsvbuffer(buf, count, malloc_var);
  }
 
  LOG_DEBUG("creating variables done, %i bytes read from variable.csv input", count);

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
  //LOG_INFO("pilot_plc_send_variables_config_numbers() sending %i bytes", sizeof(u16) * _internals.variablestream.config.count);
  /* send all variable numbers as a blob */
  pilot_send(target_plc_read, (char*) _internals.variablestream.read_config.variables, sizeof(u16) * _internals.variablestream.read_config.count);
}

static void pilot_plc_check_send_variables_readconfig(void)
{
  int count = _internals.variablestream.read_config.count;

  /* check if we need to send the variable config */
  if (count > 0)
  {
    //LOG_INFO("pilot_plc_check_send_variables_config() sends %i vars", count);

    /* send the variables config cmd to the pilot */
    pilot_plc_send_variables_readconfig_cmd(count);

    /* send the variable contents to the pilot */
    pilot_plc_send_variables_readconfig_numbers();

    /* reset the count back to zero */
    _internals.variablestream.read_config.count = 0;
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
            _internals.variablestream.read_config.variables[var_index++] = number;
            _internals.variablestream.read_config.count = var_index;
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
  //LOG_INFO("pilot_plc_send_variables_config_numbers() sending %i bytes", sizeof(u16) * _internals.variablestream.config.count);
  /* send all variable numbers as a blob */
  pilot_send(target_plc_write, (char*)_internals.variablestream.write_config.variables, sizeof(u16) * _internals.variablestream.write_config.count);
}

static void pilot_plc_check_send_variables_writeconfig(void)
{
  int count = _internals.variablestream.write_config.count;

  /* check if we need to send the variable config */
  if (count > 0)
  {
    //LOG_INFO("pilot_plc_check_send_variables_config() sends %i vars", count);

    /* send the variables config cmd to the pilot */
    pilot_plc_send_variables_writeconfig_cmd(count);

    /* send the variable contents to the pilot */
    pilot_plc_send_variables_writeconfig_numbers();

    /* reset the count back to zero */
    _internals.variablestream.write_config.count = 0;
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

            _internals.variablestream.write_config.variables[var_index++] = VAR_TO_UINT16(number, is_forced);
            _internals.variablestream.write_config.count = var_index;
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
  _internals.variablestream.is_values_updated = 0;

  /* send a get variable values request to the pilot */
  pilot_plc_send_get_variable_values_cmd();

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (_internals.variablestream.is_values_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* set the variables */
  if (!is_timedout)
    *values = &_internals.variablestream.read_values;

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
    if (_internals.variablestream.write_value.index < MAX_VAR_VALUE_SIZE)
      _internals.variablestream.write_value.data[_internals.variablestream.write_value.index++] = buf[i];
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
    cmd.data[(int)pilot_plc_variables_value_index_size + i] = BYTE_FROM_INT(_internals.variablestream.write_value.index, i);

  /* send the cmd */
  pilot_send_cmd(&cmd);
}

static void pilot_plc_check_send_vars_value(void)
{
  /* check if we have data to send */
  if (_internals.variablestream.write_value.index > 0)
  {
    /* send the set variable values cmd */
    pilot_plc_send_vars_value_cmd();

    /* send the variable values */
    pilot_send(target_plc_write, _internals.variablestream.write_value.data, _internals.variablestream.write_value.index);

    /* reset the count */
    _internals.variablestream.write_value.index = 0;
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
  struct proc_dir_entry *pilot_dir, *plc_dir, *vars_config_dir;

  /* get the /proc/pilot base dir */
  _internals.proc_pilot_dir = pilot_dir = pilot_get_proc_pilot_dir();

  /* create the /proc/pilot/plc directory */
  _internals.proc_pilot_plc_dir = plc_dir = proc_mkdir_mode(proc_plc_name, 0, pilot_dir);

  /* create the file /proc/pilot/plc/state (r/w) */
  proc_create_data(proc_plc_state_name, 0666, plc_dir, &proc_plc_state_fops, NULL);

  /* create the file /proc/pilot/plc/cycletimes (read-only) */
  proc_create_data(proc_plc_cycletimes_name, 0, plc_dir, &proc_plc_cycletimes_fops, NULL);

  /* create the /proc/pilot/plc/varconfig directory */
  _internals.proc_pilot_plc_varconfig_dir = vars_config_dir = proc_mkdir_mode(proc_plc_varconfig_name, 0, plc_dir);

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
  remove_proc_subtree(proc_plc_name, _internals.proc_pilot_dir);

  /* remove the file /proc/pilot/plc/state */
  //remove_proc_entry(proc_plc_state_name, _internals.proc_pilot_plc_dir);

  /* remove the file /proc/pilot/plc/cycletimes */
  //remove_proc_entry(proc_plc_cycletimes_name, _internals.proc_pilot_plc_dir);

  /* remove the file /proc/pilot/plc/varconfig/read_config */
  //remove_proc_entry(proc_plc_vars_readconfig_name, _internals.proc_pilot_plc_varconfig_dir);

  /* remove the file /proc/pilot/plc/varconfig/write_config */
  //remove_proc_entry(proc_plc_vars_writeconfig_name, _internals.proc_pilot_plc_varconfig_dir);

  /* remove the file /proc/pilot/plc/varconfig/value */
  //remove_proc_entry(proc_plc_vars_value_name, _internals.proc_pilot_plc_varconfig_dir);

  /* remove the directory /proc/pilot/plc/varconfig */
  //remove_proc_entry(proc_plc_varconfig_name, _internals.proc_pilot_plc_dir);

  /* remove the directory /proc/pilot/plc */
  //remove_proc_entry(proc_plc_name, _internals.proc_pilot_dir);

  /* create the file /proc/pilot/plc/varconfig/variables (r/w) */
  //remove_proc_entry(proc_plc_variables_name, _internals.proc_pilot_plc_varconfig_dir);
}

// *******************************************************************
// START pilot interface function implementation

/* the command callback handler */
static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  pilot_cmd_handler_status_t ret;
  uint16_t number;
  LOG_DEBUG("pilot_callback_cmd_received() called");

  switch (cmd.type)
  {
    case pilot_cmd_type_plc_variable_get:
    number = *((uint16_t *)&cmd.data[8]) & 0xFFF;
    if (number < _internals.variables_count)
    {
    LOG_DEBUG("pilot_callback_cmd_received() received pilot_cmd_type_plc_variable_get answer, variable number %i", _internals.variables[number]->number);
      memcpy(_internals.variables[number]->value, cmd.data, 8);
      _internals.variables[number]->is_variable_updated = 1;
    }

    //if (_internals.variables[])

    ret = pilot_cmd_handler_status_handled;
    break;
    case pilot_cmd_type_plc_variable_set:
    number = *((uint16_t *)&cmd.data[8]) & 0xFFF;
    LOG_DEBUG("pilot_callback_cmd_received() received pilot_cmd_type_plc_variable_set answer, variable number %i", number);

    ret = pilot_cmd_handler_status_handled;
    break;

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
    _internals.variablestream.read_values.index = 0;
    _internals.variablestream.read_values.expected = INT_FROM_BYTES((cmd.data + (int)pilot_plc_variables_value_index_size));
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
  if (_internals.variablestream.read_values.index < MAX_VAR_VALUE_SIZE)
  {
    _internals.variablestream.read_values.data[_internals.variablestream.read_values.index++] = data;
    if (_internals.variablestream.read_values.index >= _internals.variablestream.read_values.expected)
      _internals.variablestream.is_values_updated = 1;
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

  /* unregister with the base driver */
  if (_internals.is_cmd_handler_registered) {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      _internals.is_cmd_handler_registered = 0;
  }

  if (_internals.is_read_stream_handler_registered) {
    pilot_unregister_stream_handler(target_plc_read);
    _internals.is_read_stream_handler_registered = 0;
  }

  /* free plc variables related memory */
  freevariables();

  /* unregister the filesystem entries */
  pilot_plc_proc_deinit();
}

