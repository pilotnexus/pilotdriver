#include <linux/kernel.h>     /* needed for KERN_INFO */
#include <linux/proc_fs.h>    /* needed for functions to manage /proc/pilot/plc files */
#include <linux/seq_file.h>   /* needed for seq_file struct and functions */
#include <linux/btree.h>      /* needed for B+ tree used for var dirs */
#include <asm/uaccess.h>      /* needed for copy_from_user() function */
#include "module.h"           /* include defines that describe the module */
#include "../driver/export.h" /* needed for rpcp main driver functions */
#include <linux/string.h>     /* for memset() */
#include <linux/slab.h>       /* kmalloc() and kfree() */
#include <linux/gfp.h>        /* flags for kmalloc() */
#include <asm/uaccess.h>      /* copy_to_user() */
#include <linux/wait.h>       /* waitqueue */
#include <linux/poll.h>       /* file polling */
#include <linux/sched.h>      /* TASK_INTERRUPTIBLE */
#include <linux/mutex.h>   /* mutex */
#include <linux/kfifo.h>

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
DECLARE_WAIT_QUEUE_HEAD(msg_recv_wait); 



/* internal variables */
static internals_t _internals = {
  .is_cmd_handler_registered = false,
  .is_state_updated = false,
  .is_info_updated = false,
  .is_cycletimes_updated = false,
  .variables = NULL,
  .variables_count = 0
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
static const char proc_plc_vars_name[] = "variables";            /* name of the vars directory in /proc/pilot/plc/vars */
static const char proc_plc_vars_stream_name[] = "stream";        /* name of the read config file in /proc/pilot/plc/read*/
static const char proc_plc_vars_value_name[] = "value";          /* name of the value file in /proc/pilot/plc/varconf/value */
static const char proc_plc_varconfig_name[] = "varconfig";       /* name of the variables.csv file in /proc/pilot/plc/varconfig */
static const char proc_plc_fwinfo_name[] = "fwinfo";                  /* name of the info file in /proc/pilot/plc/info */


static const char proc_plc_var_value[] = "value";
static const char proc_plc_var_type[] = "type";
static const char proc_plc_var_subscribed[] = "subscribe";
static const char proc_plc_var_forced[] = "force";
static const char proc_plc_var_force_value[] = "force_value";

// *******************************************************************
// START VARIABLE.csv funtions

/*********************/
/*  IEC Types defs   */
/*********************/
/*
typedef uint8_t  IEC_BOOL;

typedef int8_t    IEC_SINT;
typedef int16_t   IEC_INT;
typedef int32_t   IEC_DINT;
typedef int64_t   IEC_LINT;

typedef uint8_t    IEC_USINT;
typedef uint16_t   IEC_UINT;
typedef uint32_t   IEC_UDINT;
typedef uint64_t   IEC_ULINT;

typedef uint8_t    IEC_BYTE;
typedef uint16_t   IEC_WORD;
typedef uint32_t   IEC_DWORD;
typedef uint64_t   IEC_LWORD;

typedef float    IEC_REAL;
typedef double   IEC_LREAL;
*/

static int get_IEC_size(enum iectypes type)
{
  switch(type)
  {
    case IEC_BOOL: return sizeof(uint8_t);

    case IEC_SINT:return sizeof(int8_t);
    case IEC_INT:return sizeof(int16_t);
    case IEC_DINT:return sizeof(int32_t);
    case IEC_LINT:return sizeof(int64_t);

    case IEC_USINT:return sizeof(uint8_t);
    case IEC_UINT:return sizeof(uint16_t);
    case IEC_UDINT:return sizeof(uint32_t);
    case IEC_ULINT: return sizeof(uint64_t);

    case IEC_BYTE:return sizeof(uint8_t);
    case IEC_WORD:return sizeof(uint16_t);
    case IEC_DWORD:return sizeof(uint32_t);
    case IEC_LWORD:return sizeof(uint64_t);

    //case IEC_REAL:return sizeof(float);
    //case IEC_LREAL:return sizeof(double);
    default: return 0;
  }
}

static int raw_IEC_to_string(enum iectypes type, char *buffer_in, int buffer_in_length, char *buffer_out, int buffer_out_length)
{
  switch(type)
  {
    case IEC_BOOL: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint8_t *)buffer_in));

    case IEC_SINT: return snprintf(buffer_out, buffer_out_length, "%i\n", *((int8_t *)buffer_in));
    case IEC_INT: return snprintf(buffer_out, buffer_out_length, "%i\n", *((int16_t *)buffer_in));
    case IEC_DINT: return snprintf(buffer_out, buffer_out_length, "%i\n", *((int32_t *)buffer_in));
    case IEC_LINT: return snprintf(buffer_out, buffer_out_length, "%lli\n", *((int64_t *)buffer_in));

    case IEC_USINT: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint8_t *)buffer_in));
    case IEC_UINT: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint16_t *)buffer_in));
    case IEC_UDINT: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint32_t *)buffer_in));
    case IEC_ULINT: return snprintf(buffer_out, buffer_out_length, "%llu\n", *((uint64_t *)buffer_in));

    case IEC_BYTE: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint8_t *)buffer_in));
    case IEC_WORD: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint16_t *)buffer_in));
    case IEC_DWORD: return snprintf(buffer_out, buffer_out_length, "%u\n", *((uint32_t *)buffer_in));
    case IEC_LWORD: return snprintf(buffer_out, buffer_out_length, "%llu\n", *((uint64_t *)buffer_in));

    //case IEC_REAL: return snprintf(buffer_out, buffer_out_length, "%f", *((float *)buffer_in));
    //case IEC_LREAL: return snprintf(buffer_out, buffer_out_length, "%lf", *((double *)buffer_in));
    default: return 0;
  }
  return 0;
}

static int string_to_IEC_from_user(enum iectypes type, const char *buffer_in, int buffer_in_length, char *buffer_out, int buffer_out_length)
{
  unsigned int base = 10;
  int start_index = 0;
  if (buffer_in_length > 2 && buffer_in[0] == '0' && buffer_in[1] == 'x')
  {
    base = 16;
    start_index = 2;
  }

  memset(buffer_out, 0, buffer_out_length);

  switch(type)
  {
    case IEC_BOOL: return kstrtobool_from_user(&buffer_in[start_index], buffer_in_length, (bool *)buffer_out);

    case IEC_SINT: return kstrtos8_from_user(&buffer_in[start_index], buffer_in_length, base, (s8 *)buffer_out);
    case IEC_INT: return kstrtos16_from_user(&buffer_in[start_index], buffer_in_length, base, (s16 *)buffer_out);
    case IEC_DINT: return kstrtoint_from_user(&buffer_in[start_index], buffer_in_length, base, (int *)buffer_out);
    case IEC_LINT: return kstrtoll_from_user(&buffer_in[start_index], buffer_in_length, base, (long long *)buffer_out);

    case IEC_BYTE:
    case IEC_USINT: return kstrtou8_from_user(&buffer_in[start_index], buffer_in_length, base, (u8 *)buffer_out);
    case IEC_WORD:
    case IEC_UINT: return kstrtou16_from_user(&buffer_in[start_index], buffer_in_length, base, (u16 *)buffer_out);
    case IEC_DWORD:
    case IEC_UDINT: return kstrtouint_from_user(&buffer_in[start_index], buffer_in_length, base, (unsigned int *)buffer_out);
    case IEC_LWORD:
    case IEC_ULINT: return kstrtoull_from_user(&buffer_in[start_index], buffer_in_length, base, (unsigned long long *)buffer_out);
    default: return -EINVAL;
  }

}


static void pilot_plc_send_set_variable_cmd(uint16_t varnumber, uint8_t subvalue, char *value, uint8_t valuelen)
{
  pilot_cmd_t cmd;
  msg_plc_var_t *v = (msg_plc_var_t *)cmd.data;

  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variable_set;

  v->opt = SET_VAR_LEN(valuelen);
  v->opt |= SET_VAR_SUB(subvalue);
  v->number = varnumber;

  memset(v->value, 0, MSG_PLC_VAR_MAX_LEN);
  memcpy(v->value, value, valuelen);

  cmd.length = MSG_LEN(MSG_PLC_VAR_HEADER_LEN+valuelen);
  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_get_variable_cmd(pilot_plc_variable_t *variable, uint8_t subvalue)
{
  pilot_cmd_t cmd;
  msg_plc_var_t *v = (msg_plc_var_t *)cmd.data;

  memset(&cmd, 0, sizeof(msg_plc_var_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_variable_get;

  v->opt |= SET_VAR_LEN(get_IEC_size(variable->iectype));
  v->opt |= SET_VAR_SUB(subvalue);
  v->number = variable->number;

  cmd.length = MSG_LEN(MSG_PLC_VAR_HEADER_LEN);
  pilot_send_cmd(&cmd);
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


const char * getVarClassStr(enum iecvarclass varclass)
{
  switch(varclass)
  {
    case IN: return "IN"; break;
    case OUT: return "OUT"; break;
    case MEM: return "MEM"; break;
    case VAR: return "VAR"; break;
    case EXT: return "EXT"; break;
    default: return "NONE"; break;
  }
}

enum iecvarclass strToIECVarClass(const char *str, int len)
{
  enum iecvarclass var = NONE;

  if (strncmp(str, "IN", len) == 0)
    var = IN;
  else if (strncmp(str, "OUT", len) == 0)
    var = OUT;
  else if (strncmp(str, "MEM", len) == 0)
    var = MEM;
  else if (strncmp(str, "VAR", len) == 0)
    var = VAR;
  else if (strncmp(str, "EXT", len) == 0)
    var = EXT;

  return var;
}

const char * getIecTypeStr(enum iectypes type)
{  
  switch(type)
  { 
    case IEC_BOOL: return  "BOOL"; break;
    case IEC_SINT: return  "SINT"; break;
    case IEC_INT: return   "INT"; break;
    case IEC_DINT: return  "DINT"; break;
    case IEC_LINT: return  "LINT"; break;
    case IEC_USINT: return "USINT"; break;
    case IEC_UINT: return  "UINT"; break;
    case IEC_UDINT: return "UDINT"; break;
    case IEC_ULINT: return "ULINT"; break;
    case IEC_BYTE: return  "BYTE"; break;
    case IEC_WORD: return  "WORD"; break;
    case IEC_DWORD: return "DWORD"; break;
    case IEC_LWORD: return "LWORD"; break;
    case IEC_REAL: return  "REAL"; break;
    case IEC_LREAL: return "LREAL"; break;
    default: return "NONE"; break;
  }
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

uint32_t analyzecsvbuffer(const char *buf, int fsize, bool(*handleline)(int , enum iecvarclass, enum iectypes, const char *, int ))
{
  int index = 0;
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
        type = strToIECVarClass(&buf[col_start[1]], col_length[1]);
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
            {
              //remove first variable segment CONFIG, so add the start index of second var segment
              //and remove the length of the first variable segment and one extra char for the separator
              if (!(handleline(number, type, iecVar, &buf[col_start[2]+var_col_start[1]], col_length[2]-var_col_length[0]-1)))
		return 0;
              if (number != maxnumber)
	        return 0; //numbers need to be continous.
              maxnumber++;
            }
          }
        }
      }
    }
  }
  return maxnumber;
}

bool debug_print_var_line(int number, enum iecvarclass type, enum iectypes iecvar, const char *variable, int varLength)
{
  //add debug if wanted
  LOG_DEBUGALL("variable nr: %i:  '%.*s'\n", number, varLength, variable);
  return true;
}

static ssize_t pilot_plc_proc_var_read(struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
  unsigned int copied=0, not_copied;
  int ret = 0;
  char varbuf[MSG_PLC_VAR_MAX_LEN];
  char conv_buf[CONV_BUF_LEN]; 

  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)filep->private_data;


  LOG_DEBUG("pilot_plc_proc_var_read() called\n");
  if (variable && (!variable->is_variable_updated || variable->is_poll))
  {
    if (!variable->is_poll)
      pilot_plc_send_get_variable_cmd(variable, 0);

    do 
    {
      if (kfifo_is_empty(&variable->fifo)) 
      {
        if (filep->f_flags & O_NONBLOCK)
                return -EAGAIN;

        if (variable->is_poll)
        {
          LOG_DEBUG("wait for item in kfifo queue (polling)");
          ret = wait_event_interruptible(variable->in_queue, !kfifo_is_empty(&variable->fifo));
        }
        else
        {
          LOG_DEBUG("wait for item in kfifo queue (with timeout)");
          ret = wait_event_interruptible_timeout(variable->in_queue, !kfifo_is_empty(&variable->fifo), (200 * HZ / 1000));
          if (ret == 0) //timeout
            return -EFAULT;
        }
        LOG_DEBUG("wait done, ret=%i", ret);
        if (ret == -ERESTARTSYS)
                return ret;
      }

    if (mutex_lock_interruptible(&variable->access_lock))
      return -ERESTARTSYS;
    //ret = kfifo_to_user(&variable->fifo, buf, count, &copied);
    ret = kfifo_out(&variable->fifo, varbuf, MSG_PLC_VAR_MAX_LEN);
    copied = raw_IEC_to_string(variable->iectype, varbuf, get_IEC_size(variable->iectype), conv_buf, CONV_BUF_LEN);
    not_copied = copy_to_user(buf, conv_buf, copied);
    //TODO - handle if not_copied > 0

    LOG_DEBUG("raw_IEC_to_string (raw: %x%x%x%x%x%x%x%x '%.*s'), copied bytes: %i", varbuf[0],varbuf[1],varbuf[2],varbuf[3],varbuf[4],varbuf[5],varbuf[6],varbuf[7],copied, buf, copied);

    variable->is_variable_updated = true;
    mutex_unlock(&variable->access_lock);

    /*
     * If we couldn't read anything from the fifo (a different
     * thread might have been faster) we either return -EAGAIN if
     * the file descriptor is non-blocking, otherwise we go back to
     * sleep and wait for more data to arrive.
     */
    if (copied == 0 && (filep->f_flags & O_NONBLOCK))
            return -EAGAIN;

    } while (copied == 0);
  }   

  LOG_DEBUG("Returning from pilot_plc_proc_var_read() with %i copied bytes", copied);
  return copied;
}

static int pilot_plc_proc_var_open(struct inode *inode, struct file *file)
{

  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(inode);

  if (variable->is_open)
    return -EBUSY; /* already open */

  if (mutex_lock_interruptible(&variable->access_lock))
    return -ERESTARTSYS;

  kfifo_reset(&variable->fifo);
  variable->is_open = true;
  variable->is_variable_updated = false;
  variable->is_poll = false;
  file->private_data = variable;

  mutex_unlock(&variable->access_lock);
  
  LOG_DEBUG("plc variable file %s open, flags=%X\n", variable->variable, file->f_flags);
  return 0;
}

static ssize_t pilot_plc_proc_var_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret = -EINVAL;
  char value[MSG_PLC_VAR_MAX_LEN];
  int index = 0;
  int waitret;

  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file->f_inode);

  if (!variable)
    return -EINVAL;

  LOG_DEBUG("pilot_plc_proc_var_write() called with count=%i", count);

    /* try to get an int value from the user */
      //change to copy_from_user(msg,buf,count) for more generic use

    if (string_to_IEC_from_user(variable->iectype, &buf[index], count-index, value, MSG_PLC_VAR_MAX_LEN) != -EINVAL)
    {
      /* send a plc state set cmd to the pilot */ 
      //LOG_DEBUG("writing variable %i to plc. new value %i\n", variable->number, new_value);
    
      pilot_plc_send_set_variable_cmd(variable->number, 0, value, get_IEC_size(variable->iectype));
      LOG_DEBUG("wait_event_interruptible_timeout() after set_variable");
      waitret = wait_event_interruptible_timeout(variable->in_queue, !kfifo_is_empty(&variable->fifo), (200 * HZ / 1000));
      LOG_DEBUG("wait_event_interruptible_timeout() after set_variable returned with returnvalue %i\n", waitret);      
      
      if (waitret == 0 || waitret == -ERESTARTSYS)
        ret = -EINVAL;
      else
        ret = count;
    }
    else
    {
      LOG_DEBUG("error parsing input\n");      
    }
  return ret;
}

static unsigned int pilot_plc_proc_var_poll(struct file *filp, poll_table *wait)
{
  unsigned int events = 0;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(filp->f_inode);

  poll_wait(filp, &variable->in_queue, wait);
  variable->is_poll = true;

  if (!kfifo_is_empty(&variable->fifo))
  {
    LOG_DEBUG("pilot_plc_proc_var_poll() called, return POLLIN | EPOLLPRI | POLLRDNORM (items in queue)\n");
    events = POLLIN | EPOLLPRI | POLLRDNORM; 
  }
  else
    LOG_DEBUG("pilot_plc_proc_var_poll() called, return 0 (queue empty)\n");

  return events;
}

static int pilot_plc_proc_var_release(struct inode * inode, struct file * file)
{
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file->f_inode);

  if (mutex_lock_interruptible(&variable->access_lock))
    LOG_DEBUG("mutex lock cancelled");

  variable->is_open = false;
  variable->is_variable_updated = false; //return from waiting handlers

  mutex_unlock(&variable->access_lock);
  //pilot_plc_proc_var_fasync(-1, file, 0);

  LOG_DEBUG("plc variable file %s release\n", variable->variable);
  return 0;
}

loff_t pilot_plc_proc_var_llseek(struct file *filp, loff_t off, int whence)
{
  //pilot_plc_variable_t *variable = (pilot_plc_variable_t *)filp->private_data;
  loff_t newpos;

  //LOG_DEBUG("llseek(), whence=%i, loff=%i %s\n", whence, (int)off, variable->variable);

  switch(whence) {
   case 0: /* SEEK_SET */
    newpos = off;
    break;

   case 1: /* SEEK_CUR */
    newpos = filp->f_pos + off;
    break;

   case 2: /* SEEK_END */
    //newpos = dev->size + off;
    //currently we cannot seek to the end
    return -ESPIPE; /* unseekable */
    break;

   default: /* can't happen */
    return -EINVAL;
  }
  if (newpos<0) return -EINVAL;
  filp->f_pos = newpos;
  return newpos;
}

static int pilot_plc_proc_var_type_show(struct seq_file *file, void *data)
{
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)file->private;

  seq_printf(file, "%s\n", getIecTypeStr(variable->iectype));

  return 0;
}

static int pilot_plc_proc_var_type_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_var_type_show, PDE_DATA(inode));
}

static int pilot_plc_try_get_variable_config(pilot_plc_variable_t * variable, uint8_t config, int timeout)
{
  pilot_cmd_t cmd;
  int is_timedout = 0;
  bool updated = false;
  unsigned long timestamp;
  msg_plc_var_config_t *data = (msg_plc_var_config_t *)cmd.data;

  /* reset the is state updated flag */
  switch(config) {
    case 2: variable->is_subscribed_updated = false; break;
    case 3: variable->is_forced_updated = false; break;
    default: return -1; break;
  }

  /* set the plc state get request */
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_read_var_config;
  data->number = variable->number;
  data->config = config;
  data->value = 0;

  cmd.length = MSG_LEN(4);
  pilot_send_cmd(&cmd);
  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (!updated)
  {
    switch(config) {
      case 2: if (variable->is_subscribed_updated) updated = true; break;
      case 3: if (variable->is_forced_updated) updated = true; break;
      default: return -1; break;
    }

    if (time_after(jiffies, timestamp))
    { 
      LOG_DEBUG("TIMEOUT pilot_plc_try_get_variable_config (config %d, varnum %d)", config, variable->number);
      is_timedout = 1;
      break;
    } 
  }

  return is_timedout ? -1 : SUCCESS;
}

static void pilot_plc_set_variable_config(pilot_plc_variable_t * variable, uint8_t config, uint8_t value)
{
  pilot_cmd_t cmd;
  msg_plc_var_config_t *data = (msg_plc_var_config_t *)cmd.data;

  /* set the plc state set request */
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_write_var_config;
  data->number = variable->number;
  data->config = config;
  data->value = value;

  cmd.length = MSG_LEN(4);
  pilot_send_cmd(&cmd);
}

static int pilot_plc_try_set_variable_config(pilot_plc_variable_t * variable, uint8_t config, uint8_t value, int timeout)
{
  int is_timedout = 0;
  bool updated = false;
  unsigned long timestamp;

  /* reset the is state updated flag */
  switch(config) {
    case 2: variable->is_subscribed_updated = false; break;
    case 3: variable->is_forced_updated = false; break;
    default: return -1; break;
  }

  /* set the plc state set request */
  pilot_plc_set_variable_config(variable, config, value);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (!updated) 
  {
    switch(config) {
      case 2: if (variable->is_subscribed_updated) updated = true; break;
      case 3: if (variable->is_forced_updated) updated = true; break;
      default: return -1; break;
    }

    if (time_after(jiffies, timestamp))
    {
      LOG_DEBUG("TIMEOUT pilot_plc_try_set_variable_config (config %d, varnum %d)", config, variable->number);
      is_timedout = 1;
      break;
    }
  }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_plc_proc_var_subscribed_show(struct seq_file *file, void *data)
{
  int ret;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)file->private;

  if (pilot_plc_try_get_variable_config(variable, 0x2, 200) == SUCCESS)
  {
    seq_printf(file, "%i\n", variable->subscribed ? 1 : 0);
    ret=0;
  }
  else
  ret = -EFAULT;

  return ret;
}

static ssize_t pilot_plc_proc_var_subscribed_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret = -EINVAL;
  int new_value;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file->f_inode);

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL; /* return an error if the conversion fails */
  else if (new_value >= 0 && new_value < 3)
  {
    /* send a plc state set cmd to the pilot */
    if (pilot_plc_try_set_variable_config(variable, 0x2, new_value, 200) == SUCCESS)
      ret = count;
    else
      ret = -EINVAL;
  }
  return ret;
}

static int pilot_plc_proc_var_subscribed_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_var_subscribed_show, PDE_DATA(inode));
}

static int pilot_plc_proc_var_forced_show(struct seq_file *file, void *data)
{
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)file->private;

  pilot_plc_try_get_variable_config(variable, 0x3, 200);
  seq_printf(file, "%i\n", variable->forced ? 1 : 0);

  return 0;
}

static ssize_t pilot_plc_proc_var_forced_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret = -EINVAL;
  bool new_value;
  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file->f_inode);

  /* try to get an int value from the user */
  if (kstrtobool_from_user(buf, count, &new_value) != SUCCESS)
    ret = -EINVAL; /* return an error if the conversion fails */
  else
  {
    /* send a plc state set cmd to the pilot */
    if (pilot_plc_try_set_variable_config(variable, 0x3, new_value ? 1 : 0, 200) == SUCCESS)
      ret = count;
    else
      ret = -EINVAL;
  }
  return ret;
}

static int pilot_plc_proc_var_forced_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_var_forced_show, PDE_DATA(inode));
}


// var_force_value

static int pilot_plc_proc_var_force_value_show(struct seq_file *file, void *data)
{
  int is_timedout = 0;
  int timeout = 100;
  unsigned long timestamp;
  char conv_buf[CONV_BUF_LEN]; 

  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)file->private;
 
  variable->is_force_value_updated = false;
  pilot_plc_send_get_variable_cmd(variable, 0x3); //get forced value

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);
 
  /* wait until the state is updated or the timeout occurs */
  while (variable->is_force_value_updated == false)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  /* read the updated state */
  if (!is_timedout)
  {

    raw_IEC_to_string(variable->iectype, variable->force_value, get_IEC_size(variable->iectype), conv_buf, CONV_BUF_LEN);
    seq_printf(file, "%s", conv_buf);
  }
  else
  {
    //LOG_INFO("pilot_plc_proc_var_force_value_show() timedout while waiting for forced value!");
  }

  return is_timedout ? -1 : SUCCESS;
}

static ssize_t pilot_plc_proc_var_force_value_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  int ret = -EINVAL;
  char value[MSG_PLC_VAR_MAX_LEN];
  int index = 0;
  int waitret;

  pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(file->f_inode);

  if (!variable)
    return -EINVAL;

  LOG_DEBUG("pilot_plc_proc_var_force_value_write() called with count=%i", count);

    /* try to get an int value from the user */
      //change to copy_from_user(msg,buf,count) for more generic use

    if (string_to_IEC_from_user(variable->iectype, &buf[index], count-index, value, MSG_PLC_VAR_MAX_LEN) != -EINVAL)
    {
      /* send a plc state set cmd to the pilot */ 
      //LOG_DEBUG("writing variable %i to plc. new value %i\n", variable->number, new_value);
    
      pilot_plc_send_set_variable_cmd(variable->number, 0x3, value, get_IEC_size(variable->iectype));
      LOG_DEBUG("wait_event_interruptible_timeout() after set_variable");
      waitret = wait_event_interruptible_timeout(variable->in_queue, !kfifo_is_empty(&variable->fifo), (200 * HZ / 1000));
      LOG_DEBUG("wait_event_interruptible_timeout() after set_variable returned with returnvalue %i\n", waitret);      
      
      if (waitret == 0 || waitret == -ERESTARTSYS)
        ret = -EINVAL;
      else
        ret = count;
    }
    else
    {
      LOG_DEBUG("error parsing input\n");      
    }
  return ret;
}

static int pilot_plc_proc_var_force_value_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_var_force_value_show, PDE_DATA(inode));
}

/* file operations for the /proc/pilot/plc/vars/..../value */
static const struct proc_ops proc_plc_variable_fops = {
  
  .proc_open = pilot_plc_proc_var_open,
  .proc_read = pilot_plc_proc_var_read,
  .proc_write = pilot_plc_proc_var_write,
  .proc_poll = pilot_plc_proc_var_poll,
  .proc_lseek = pilot_plc_proc_var_llseek,
  .proc_release = pilot_plc_proc_var_release
};

static const struct proc_ops proc_plc_variable_type_fops = {
  
  .proc_open = pilot_plc_proc_var_type_open,
  .proc_read = seq_read,
  .proc_release = single_release
};

static const struct proc_ops proc_plc_variable_subscribed_fops = {
  
  .proc_open = pilot_plc_proc_var_subscribed_open,
  .proc_read = seq_read,
  .proc_write = pilot_plc_proc_var_subscribed_write,
  .proc_lseek = seq_lseek,
  .proc_release = single_release
};

static const struct proc_ops proc_plc_variable_forced_fops = {
  
  .proc_open = pilot_plc_proc_var_forced_open,
  .proc_read = seq_read,
  .proc_write = pilot_plc_proc_var_forced_write,
  .proc_lseek = seq_lseek,
  .proc_release = single_release
};

static const struct proc_ops proc_plc_variable_force_value_fops = {
  
  .proc_open = pilot_plc_proc_var_force_value_open,
  .proc_read = seq_read,
  .proc_write = pilot_plc_proc_var_force_value_write,
  .proc_lseek = seq_lseek,
  .proc_release = single_release
};

void remove_path_recursive(pilot_plc_vardir_t *start) {
  int i;
  //remove children
  for (i=0; i<start->childCount;i++) {
    remove_path_recursive(start->children[i]);
  }

  //free children array
  if (start->children != NULL) {
    kfree((void *)start->children);
  }

  kfree((void *)start);
}

void remove_path(void) {
  int i;
  //remove children
  for (i=0; i< _internals.proc_pilot_plc_vars_dir_root.childCount;i++) {
    remove_path_recursive(_internals.proc_pilot_plc_vars_dir_root.children[i]);
  }

  if (_internals.proc_pilot_plc_vars_dir_root.children != NULL) {
    kfree((void *)_internals.proc_pilot_plc_vars_dir_root.children);
  }

  _internals.proc_pilot_plc_vars_dir_root.children = NULL;
}

pilot_plc_vardir_t * create_path(pilot_plc_vardir_t *parent, char* name) {
  int i;
  struct pilot_plc_vardir_t **children;

  pilot_plc_vardir_t *path = NULL;
  for (i=0; i<parent->childCount;i++) {
    LOG_DEBUG("comparing existing child %s to %s", parent->children[i]->name, name);

    if(strcmp(parent->children[i]->name, name) == 0) {
      LOG_DEBUG("Matching path name, using path");
      path = parent->children[i];
      break;
    }
  }

  if (path == NULL) { //create
      LOG_DEBUG("No matching path name, creating new path %s", name);

    children = (struct pilot_plc_vardir_t **)kzalloc(sizeof(struct pilot_plc_vardir_t **) * (parent->childCount+1), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
    if (parent->childCount > 0) {
      memcpy(children, parent->children, sizeof(struct pilot_plc_vardir_t **) * (parent->childCount));
      kfree((void *)parent->children);
    }
    parent->children = (pilot_plc_vardir_t **)children;
    path = (pilot_plc_vardir_t *)kzalloc(sizeof(pilot_plc_vardir_t), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
    parent->children[parent->childCount++] = path;

    //initialize path
    path->parent = parent;
    path->name = name;
    path->self = proc_mkdir_mode(name, 0, path->parent->self);
    path->children = NULL;
    path->childCount = 0;
  }

  return path;
}

bool malloc_var(int number, enum iecvarclass type, enum iectypes iecvar, const char *variable, int varLength)
{
  int i;
  int ret;
  int pathCount = 0;
  char *paths[MAX_VAR_DIR_DEPTH];
  pilot_plc_vardir_t *varpath = &_internals.proc_pilot_plc_vars_dir_root;
  struct proc_dir_entry *vardir;

  //add debug if wanted
  _internals.variables[number] = (pilot_plc_variable_t *)kzalloc(sizeof(pilot_plc_variable_t), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
  _internals.variables[number]->number = number;
  _internals.variables[number]->varclass = type;
  _internals.variables[number]->iectype = iecvar;
  mutex_init(&_internals.variables[number]->access_lock);
  init_waitqueue_head(&_internals.variables[number]->in_queue);
  _internals.variables[number]->variable = (char*)kzalloc(varLength+1 * sizeof(char), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
  _internals.variables[number]->variablename = _internals.variables[number]->variable;
  
  ret = kfifo_alloc(&_internals.variables[number]->fifo, FIFO_SIZE, GFP_KERNEL);
  if (ret) {
    LOG_DEBUG("ERROR kfifo_alloc for variable %i", number);
    return false;
  }

  for (i = 0; i < varLength; i++)
  {
    //lowercase characters
    _internals.variables[number]->variable[i] = (variable[i] > 0x40) && (variable[i] < 0x5B) ? variable[i] + 0x20 : variable[i];
    //make dots into 0 (string terminations)
    if (variable[i] == 0x2e) {
      _internals.variables[number]->variable[i] = 0;

      if (pathCount < MAX_VAR_DIR_DEPTH) {
        paths[pathCount++] = _internals.variables[number]->variablename;
        LOG_DEBUG("varpath depth=%i, pathname=%s", pathCount, paths[pathCount-1]);
        varpath = create_path(varpath, paths[pathCount-1]);

        _internals.variables[number]->variablename = &_internals.variables[number]->variable[i+1];
      }
      else {
        //ERROR, path depth too high
        LOG_DEBUG("ERROR path depth to high for plc variable");
        return false;
      }
    }
  }
  //zero terminate string
  _internals.variables[number]->variable[i] = 0;

  //now create file
  /* create the file /proc/pilot/plc/varconf (r/w) */
  LOG_DEBUG("creating variable %i file %s", _internals.variables[number]->number, _internals.variables[number]->variablename);
  vardir = proc_mkdir_mode(_internals.variables[number]->variablename, 0, varpath->self);

  //create actual files
  proc_create_data(proc_plc_var_value, 0666, vardir, &proc_plc_variable_fops, _internals.variables[number]);
  proc_create_data(proc_plc_var_type, 0666, vardir, &proc_plc_variable_type_fops, _internals.variables[number]);
  proc_create_data(proc_plc_var_subscribed, 0666, vardir, &proc_plc_variable_subscribed_fops, _internals.variables[number]);
  proc_create_data(proc_plc_var_forced, 0666, vardir, &proc_plc_variable_forced_fops, _internals.variables[number]);
  proc_create_data(proc_plc_var_force_value, 0666, vardir, &proc_plc_variable_force_value_fops, _internals.variables[number]);
  return true;
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
      kfifo_free(&_internals.variables[i]->fifo);
    }
  }

  _internals.variables_count = 0;

  remove_path(); //remove all pilot_plc_vardir_t structs

  if (_internals.variables != NULL)
  {
      kfree((void *)_internals.variables);

      LOG_DEBUG("removing vars directory");
      remove_proc_subtree(proc_plc_vars_name, _internals.proc_pilot_plc_dir);
      //_internals.proc_pilot_plc_vars_dir = NULL; //set emnpty
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
  memcpy(cmd.data, (void *)&state, sizeof(state));
  cmd.length = MSG_LEN(sizeof(state));
  pilot_send_cmd(&cmd);
}

static void pilot_plc_send_get_state_cmd(void)
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_plc_state_get;
  cmd.length = 0; //no payload

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

static ssize_t pilot_plc_proc_state_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
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
static const struct proc_ops proc_plc_state_fops = {
  
  .proc_open = pilot_plc_proc_state_open,
  .proc_read = seq_read,
  .proc_write = pilot_plc_proc_state_write,
  .proc_lseek = seq_lseek,
  .proc_release = single_release
};

static int pilot_plc_proc_variables_state_show(struct seq_file *file, void *data)
{
  int ret, i;

  if (_internals.variables_count > 0)
  {
    for (i = 0; i < _internals.variables_count; ++i)
    {
        if (_internals.variables[i] != NULL)
        {
            seq_printf(file, "%i;%s;%s;%s;%s;\n", _internals.variables[i]->number,
            getVarClassStr(_internals.variables[i]->varclass),
            _internals.variables[i]->variable,
            _internals.variables[i]->variable,
            getIecTypeStr( _internals.variables[i]->iectype));
        }
    }
  }
  ret = 0;

  return ret;
}

static int pilot_plc_proc_variables_state_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_plc_proc_variables_state_show, PDE_DATA(inode));
}

static ssize_t pilot_plc_proc_variables_state_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  uint32_t varcount = 0;
  //first remove all variables if existent
  freevariables();

  varcount = analyzecsvbuffer(buf, count, debug_print_var_line) + 1; //read variables and store max var number plus one (var numbers are zero based)

  if (varcount > 0)
  {
    LOG_DEBUG("creating vars directory");
    _internals.proc_pilot_plc_vars_dir_root.parent = NULL;
    _internals.proc_pilot_plc_vars_dir_root.children = NULL;
    _internals.proc_pilot_plc_vars_dir_root.name = NULL;
    _internals.proc_pilot_plc_vars_dir_root.childCount = 0;
    _internals.proc_pilot_plc_vars_dir_root.self = proc_mkdir_mode(proc_plc_vars_name, 0, _internals.proc_pilot_plc_dir);

    //initialize pointer array
    _internals.variables = (pilot_plc_variable_t **)kzalloc(varcount * sizeof(pilot_plc_variable_t *), __GFP_NOFAIL | __GFP_IO | __GFP_FS);
  
    //define vars
    _internals.variables_count = analyzecsvbuffer(buf, count, malloc_var);
  }
 
  LOG_DEBUG("creating variables done, %i bytes read from variable.csv input", count);

  return count;
}

/* file operations for the /proc/pilot/plc/vars/variables */
static const struct proc_ops proc_plc_variables_fops = {
  
  .proc_open = pilot_plc_proc_variables_state_open,
  .proc_read = seq_read,
  .proc_write = pilot_plc_proc_variables_state_write,
  .proc_lseek = seq_lseek,
  .proc_release = single_release
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
  cmd.length = 0; //no payload
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
    seq_printf(file, "min:%ius\nmax:%ius\ncurrent:%ius\n  comm:%ius\n  read:%ius\n  program:%ius\n  write:%ius\n", 
      cycletimes.min, cycletimes.max, cycletimes.cur, cycletimes.comm,
      cycletimes.read,
      cycletimes.program,
      cycletimes.write
      );
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
static const struct proc_ops proc_plc_cycletimes_fops = {
  
  .proc_open = pilot_plc_proc_cycletimes_open,
  .proc_read = seq_read,
  .proc_release = single_release
};

// END cycletimes
// *******************************************************************


// *******************************************************************
// START variable stream config

//static int pilot_plc_proc_stream_open(struct inode *inode, struct file *file)
//{
//  kfifo_reset(&_internals.event_state.events);
//  mutex_unlock(&access_lock);
//  return 0;
//}

static ssize_t pilot_plc_proc_stream_read(struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
  //struct pilotevent_state *pe = filep->private_data;
  unsigned int copied;
	int ret;
  
	if (count < sizeof(struct pilotevent_state))
		return -EINVAL;

	do {
		if (kfifo_is_empty(&_internals.event_state.events)) {
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ret = wait_event_interruptible(_internals.event_state.wait,
					!kfifo_is_empty(&_internals.event_state.events));
			if (ret)
				return ret;
		}

		if (mutex_lock_interruptible(&_internals.event_state.read_lock))
			return -ERESTARTSYS;
		ret = kfifo_to_user(&_internals.event_state.events, buf, count, &copied);
		mutex_unlock(&_internals.event_state.read_lock);

		if (ret)
			return ret;

		/*
		 * If we couldn't read anything from the fifo (a different
		 * thread might have been faster) we either return -EAGAIN if
		 * the file descriptor is non-blocking, otherwise we go back to
		 * sleep and wait for more data to arrive.
		 */
		if (copied == 0 && (filep->f_flags & O_NONBLOCK))
			return -EAGAIN;

	} while (copied == 0);


  return copied;
}


static void pilot_set_module_status(module_slot_t slot, int module_status)
{
  pilot_cmd_t cmd;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(slot, module_port_1);
  cmd.type = pilot_cmd_type_module_status_set;

  cmd.data[0] = (uint8_t)slot;
  memcpy(cmd.data, (void *)&module_status, sizeof(module_status));
  cmd.length = MSG_LEN(4); 

  /* send the Cmd */
  pilot_send_cmd(&cmd);
}

static ssize_t pilot_plc_proc_stream_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  pilot_plc_state_t value;
  int module_status;
  struct pilotevent_data pe;

  if (count != sizeof(struct pilotevent_data)) 
  {
    LOG_DEBUG("stream write with wrong length. expected: %d, got %d", sizeof(struct pilotevent_data), count);
    return -EINVAL;
  }

  if (copy_from_user(&pe, buf, count) != 0)
    return -EFAULT;

  switch (pe.cmd)
  {
    case 0x1: //write variable
      if (pe.sub < _internals.variables_count)
      {
        LOG_DEBUG("stream write to variable %d received", pe.sub);
        pilot_plc_send_set_variable_cmd(pe.sub, 0, pe.data, get_IEC_size(_internals.variables[pe.sub]->iectype));
      }
    break;
    case 0x2: //write variable config
      if (pe.sub < _internals.variables_count)
      {
        LOG_DEBUG("stream write to variable %d config received", pe.sub);
        pilot_plc_set_variable_config(_internals.variables[pe.sub], pe.reserved, pe.data[0]);
      }
    break;
    case 0x10: //write plc state
      memcpy(&value, pe.data, sizeof(pilot_plc_state_t));
      pilot_plc_send_set_state_cmd(value);
    break;
    case 0x11: //write module status
      memcpy(&module_status, pe.data, sizeof(module_status));
      pilot_set_module_status(pe.reserved, module_status);
    break;
  }
  
  
  return sizeof(struct pilotevent_data); 
}

static unsigned int pilot_plc_proc_stream_poll(struct file *filep, poll_table *wait)
{
  //unsigned int events = 0;
  //pilot_plc_variable_t *variable = (pilot_plc_variable_t *)PDE_DATA(filp->f_inode);

  //struct pilotevent_state *pe = filep->private_data;
	__poll_t events = 0;

	poll_wait(filep, &_internals.event_state.wait, wait);

	if (!kfifo_is_empty(&_internals.event_state.events))
  {
    LOG_DEBUG("pilot_stream poll Events set: EPOLLPRI | EPOLLIN | EPOLLRDNORM");
		events = EPOLLPRI | EPOLLIN | EPOLLRDNORM;
  }

	return events;
}

//static int pilot_plc_proc_stream_release(struct inode * inode, struct file * file)
//{
//  return 0;
//}

loff_t pilot_plc_proc_stream_llseek(struct file *filp, loff_t off, int whence)
{
  loff_t newpos;

  LOG_DEBUG("stream llseek(), whence=%i, loff=%i\n", whence, (int)off);

  switch(whence) {
   case 0: /* SEEK_SET */
    newpos = off;
    break;

   case 1: /* SEEK_CUR */
    newpos = filp->f_pos + off;
    break;

   case 2: /* SEEK_END */
    //newpos = dev->size + off;
    //currently we cannot seek to the end
    return -ESPIPE; /* unseekable */
    break;

   default: /* can't happen */
    return -EINVAL;
  }
  if (newpos<0) return -EINVAL;
  filp->f_pos = newpos;
  return newpos;
}

static const struct proc_ops proc_plc_stream_fops = {
  
  //.proc_open = pilot_plc_proc_stream_open,
  .proc_read = pilot_plc_proc_stream_read,
  .proc_write = pilot_plc_proc_stream_write,
  .proc_poll = pilot_plc_proc_stream_poll,
  .proc_lseek = pilot_plc_proc_stream_llseek,
  //.proc_release = pilot_plc_proc_stream_release
};

// END variable stream config
// *******************************************************************

/*  */
static int pilot_try_get_fwinfo(uint8_t n, int timeout)
{
  pilot_cmd_t cmd;
  int is_timedout = 0;
  unsigned long timestamp;

  /* reset the uid_is_updated flag */
  _internals.is_fwinfo_updated = false;

  /* send the request */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.data[0] = n;
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_fwinfo;
  cmd.length = MSG_LEN(1);

  pilot_send_cmd(&cmd);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the state is updated or the timeout occurs */
  while (_internals.is_fwinfo_updated == false) 
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_proc_plc_fwinfo_show(struct seq_file *file, void *data)
{
  int ret;
  if (pilot_try_get_fwinfo(1, 500) == SUCCESS)
  {
    _internals.fwinfo[pilot_cmd_t_data_size-1] = 0; //safety terminating char
    seq_printf(file, "%s\n", _internals.fwinfo);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_proc_plc_fwinfo_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_plc_fwinfo_show, NULL);
}

static const struct proc_ops proc_plc_fwinfo_fops = {
  
  .proc_open =pilot_proc_plc_fwinfo_open,
  .proc_read = seq_read,
  .proc_lseek  =seq_lseek,
  .proc_release = single_release
};

/* register the /proc/pilot/plc/... files */
static void pilot_plc_proc_init(void)
{
  struct proc_dir_entry *pilot_dir, *plc_dir;

  /* get the /proc/pilot base dir */
  _internals.proc_pilot_dir = pilot_dir = pilot_get_proc_pilot_dir();

  /* create the /proc/pilot/plc directory */
  _internals.proc_pilot_plc_dir = plc_dir = proc_mkdir_mode(proc_plc_name, 0, pilot_dir);

  /* create the file /proc/pilot/plc/fwinfo (r) */
  proc_create_data(proc_plc_fwinfo_name, 0666, plc_dir, &proc_plc_fwinfo_fops, NULL);

  /* create the file /proc/pilot/plc/state (r/w) */
  proc_create_data(proc_plc_state_name, 0666, plc_dir, &proc_plc_state_fops, NULL);

  /* create the file /proc/pilot/plc/cycletimes (read-only) */
  proc_create_data(proc_plc_cycletimes_name, 0, plc_dir, &proc_plc_cycletimes_fops, NULL);

  /* create the file /proc/pilot/plc/stream (r/w) */
  proc_create_data(proc_plc_vars_stream_name, 0666, plc_dir, &proc_plc_stream_fops, NULL);

  /* create the file /proc/pilot/plc/varconfig (r/w) */
  proc_create_data(proc_plc_varconfig_name, 0666, plc_dir, &proc_plc_variables_fops, NULL);
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

static uint8_t *get_plc_var(uint8_t *data, plc_var_t *out) {
  msg_plc_var_t *v = (msg_plc_var_t *)data;
  uint8_t len = v->opt & 0xF;
  out->number = v->number;
  out->subvalue = (v->opt >> 4) & 0x3;

  memset(out->value, 0, sizeof(out->value));

  if (len > sizeof(out->value))
    return NULL; //not allowed
  
  memcpy(out->value, v->value, len);

  if (v->opt & 0x80)
    return data + 3 + len; //move pointer to next element
    
  return NULL; //no further variables
}

/* the command callback handler */
static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  pilot_cmd_handler_status_t ret = pilot_cmd_handler_status_ignored;
  struct pilotevent_data pe;
  char dummy[MSG_PLC_VAR_MAX_LEN];
  plc_var_t v;
  msg_plc_var_config_t *c;
  uint8_t *p;
  int i;

  //LOG_DEBUG("pilot_callback_cmd_received() called");

  switch (cmd.type)
  {
    case pilot_cmd_type_plc_variable_set:
    case pilot_cmd_type_plc_variable_get:
      p = cmd.data;
      for (i=0; i<(sizeof(cmd.data)/16) && p != NULL; i++) //dont use while(p) to safeguard against malformed data. Max size of var packet is 11 (we use 16, that makes a maximum of 12 vars in one msg )
      {
        p = get_plc_var(p, &v);
        if (v.number < _internals.variables_count) //we don't need a p != NULL check here, p is only used in the line above and it is checked before by  the for loop condition
        {
          //queue event
          pe.cmd = 0x1; //get var event
          pe.sub = v.number;
          memcpy(&pe.data, v.value, sizeof(pe.data));

          LOG_DEBUG("received variable %d, subvalue %d: %02X %02X %02X %02X %02X %02X %02X %02X", v.number, v.subvalue,
            v.value[0], v.value[1], v.value[2], v.value[3], v.value[4], v.value[5], v.value[6], v.value[7]   );

          if (!mutex_lock_interruptible(&_internals.variables[v.number]->access_lock)) 
          {
            if (v.subvalue == 0x3) //force value
            {
              memcpy(_internals.variables[v.number]->force_value, v.value, MSG_PLC_VAR_MAX_LEN);
              _internals.variables[v.number]->is_force_value_updated = true;
            }
            else 
            {
              if (kfifo_put(&_internals.event_state.events, pe) != 0)
              {
		            wake_up_poll(&_internals.event_state.wait, EPOLLIN | EPOLLPRI);
              }
              else
              {
                  LOG_ERROR("FIFO is full, discarding new value for stream buffer");
              }

              if (kfifo_avail(&_internals.variables[v.number]->fifo) < MSG_PLC_VAR_MAX_LEN)
              {
                  // Try to remove an old value to make room
                  ret = kfifo_out(&_internals.variables[v.number]->fifo, dummy, MSG_PLC_VAR_MAX_LEN);
              
                  // Check if there's enough room now
                  if (kfifo_avail(&_internals.variables[v.number]->fifo) < MSG_PLC_VAR_MAX_LEN)
                  {
                      // Still not enough room, discard the new value
                      LOG_ERROR("FIFO is full, discarding new value for variable number %i", v.number);
                  }
                  else
                  {
                      // There's enough room now, add the new value
                      kfifo_in(&_internals.variables[v.number]->fifo,v.value, MSG_PLC_VAR_MAX_LEN);
                      wake_up_poll(&_internals.variables[v.number]->in_queue, POLLIN | EPOLLPRI);
                  }
              }
              else
              {
                  // There's enough room, add the new value
                  kfifo_in(&_internals.variables[v.number]->fifo,v.value, MSG_PLC_VAR_MAX_LEN);
                  wake_up_poll(&_internals.variables[v.number]->in_queue, POLLIN | EPOLLPRI);
              }
            }

            mutex_unlock(&_internals.variables[v.number]->access_lock);
          }

        }
      }
      ret = pilot_cmd_handler_status_handled;
    break;

    //case pilot_cmd_type_plc_variable_set:
    //  // TODO - response when setting var
    //  ret = pilot_cmd_handler_status_handled;
    //break;
  
  case pilot_cmd_type_plc_read_var_config:
    c = (msg_plc_var_config_t *)&cmd.data;
    if (c->number < _internals.variables_count) 
    {
      switch(c->config)
      {
        case 2: 
          _internals.variables[c->number]->subscribed = c->value > 0 ? true : false;
          _internals.variables[c->number]->is_subscribed_updated = true;
          LOG_DEBUG("pilot_cmd_type_plc_read_var_config subscribed read = %d", _internals.variables[c->number]->subscribed);
          break;
        case 3: 
          _internals.variables[c->number]->forced = c->value > 0 ? true : false;
          _internals.variables[c->number]->is_forced_updated = true;
          LOG_DEBUG("pilot_cmd_type_plc_read_var_config forced read = %d", _internals.variables[c->number]->forced);
          break;
        default:
          LOG_DEBUG("pilot_cmd_type_plc_read_var_config UNKNOWN config value: %d", c->config);
        break;
      }

      //send stream event
      pe.cmd = 0x2; //get var config event
      pe.reserved = c->config;
      pe.sub = c->number;
      pe.data[0] = c->value;
      if (kfifo_put(&_internals.event_state.events, pe) != 0)
      {
		    wake_up_poll(&_internals.event_state.wait, EPOLLIN | EPOLLPRI);
      }

    }
    ret = pilot_cmd_handler_status_handled;
  break;

  case pilot_cmd_type_plc_write_var_config:
    c = (msg_plc_var_config_t *)&cmd.data;
    if (c->number < _internals.variables_count) 
    {
      switch(c->config)
      {
        case 2: 
          _internals.variables[c->number]->is_subscribed_updated = true;
          LOG_DEBUG("pilot_cmd_type_plc_write_var_config (varnum %d) subscribed", c->number);
          break;
        case 3: 
          _internals.variables[c->number]->is_forced_updated = true;
          LOG_DEBUG("pilot_cmd_type_plc_write_var_config (varnum %d) forced", c->number);
          break;
        default:
          c->value = 0;
          LOG_DEBUG("pilot_cmd_type_plc_write_var_config UNKNOWN config value: %d", c->config);
        break;
      }

      //send stream event
      pe.cmd = 0x2; //get var config event
      pe.reserved = c->config;
      pe.sub = c->number;
      pe.data[0] = c->value;
      if (kfifo_put(&_internals.event_state.events, pe) != 0)
      {
		    wake_up_poll(&_internals.event_state.wait, EPOLLIN | EPOLLPRI);
      }
    }
    ret = pilot_cmd_handler_status_handled;
  break;

    /* we're receiving the answer to the plc_state_get command */
  case pilot_cmd_type_plc_state_set:
  case pilot_cmd_type_plc_state_get:
    _internals.state = (pilot_plc_state_t)cmd.data[(int)pilot_plc_state_index];
    mb();
    _internals.is_state_updated = 1;

    pe.cmd = 0x10;
    memcpy(&pe.data, (void*)&_internals.state, sizeof(int));
    if (kfifo_put(&_internals.event_state.events, pe) != 0)
    {
		  wake_up_poll(&_internals.event_state.wait, EPOLLIN | EPOLLPRI);
    }

    LOG_DEBUG("pilot_callback_cmd_received() received plc_state_get answer");
    /* mark the command as handled */
    ret = pilot_cmd_handler_status_handled;
    break;
  case pilot_cmd_type_module_status_set:
  case pilot_cmd_type_module_status_get:
    pe.cmd = 0x11;
    pe.reserved = target_t_get_module_slot(cmd.target);
    memcpy(&pe.data, (void*)cmd.data, sizeof(int));
    if (kfifo_put(&_internals.event_state.events, pe) != 0)
    {
		  wake_up_poll(&_internals.event_state.wait, EPOLLIN | EPOLLPRI);
    }
    /* mark the command as handled */
    ret = pilot_cmd_handler_status_handled;
    break;

  case pilot_cmd_type_plc_cycletimes_get:
    _internals.cycletimes.min = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_min));
    _internals.cycletimes.max = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_max));
    _internals.cycletimes.cur = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_cur));
    _internals.cycletimes.tick = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_tick));
    _internals.cycletimes.comm = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_comm));
    _internals.cycletimes.read = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_read));
    _internals.cycletimes.program = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_program));
    _internals.cycletimes.write = UINT16_FROM_BYTES((cmd.data + (int)pilot_plc_cycletimes_index_write));

    mb();
    _internals.is_cycletimes_updated = true;

    LOG_DEBUG("pilot_callback_cmd_received() received plc_cycletimes_get answer");
    /* mark the command as handled */
    ret = pilot_cmd_handler_status_handled;
    break;
  case pilot_cmd_type_fwinfo:
    if (cmd.data[0] > 0)
    {
      strncpy((uint8_t *)_internals.fwinfo, &cmd.data[1], pilot_cmd_t_data_size-1);
      mb();
      _internals.is_fwinfo_updated = true;

      ret = pilot_cmd_handler_status_handled;
    }
  break;

    default: ret = pilot_cmd_handler_status_ignored;  break;
  }

  return ret;
}

/* the read stream handler */
static void pilot_plc_read_stream_callback(char data)
{
  /* handle the received data */
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

  //stream data init 
  mutex_init(&_internals.event_state.read_lock);
  INIT_KFIFO(_internals.event_state.events);
  init_waitqueue_head(&_internals.event_state.wait);

  /* register the filesystem entries */
  pilot_plc_proc_init();

  /* register with the base driver */
  if (pilot_register_cmd_handler(&pilot_cmd_handler) == SUCCESS)
  {
    LOG_DEBUG("pilot_register_cmd_handler() succeeded");
    _internals.is_cmd_handler_registered = true;

    pilot_register_stream_handler(target_plc_read, &pilot_plc_read_stream_callback);
    _internals.is_read_stream_handler_registered = true;

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
      _internals.is_cmd_handler_registered = false;
  }

  if (_internals.is_read_stream_handler_registered) {
    pilot_unregister_stream_handler(target_plc_read);
    _internals.is_read_stream_handler_registered = false;
  }

  /* free plc variables related memory */
  freevariables();

  kfifo_free(&_internals.event_state.events);
  /* unregister the filesystem entries */
  pilot_plc_proc_deinit();
}

