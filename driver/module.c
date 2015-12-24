// defines
#define MODULE_NAME "pilot"
#define IRQ_DEV_NAME MODULE_NAME
/* reserve a driver slot for every module, as we could have 4 different modules loaded that need 4 different drivers */
#define DRIVERS_COUNT MODULES_COUNT


// includes
#include "platform.h"
#include "module.h"          // include defines that describe the module
#include "common.h"          // common defines for logging
#include "types.h"
#include "queue.h"           // fifo queue, circle buffered
#include "pilotstm.h"         // pilot <-> stm communication defines
#include "commands.h"        // pilot_cmd_type_to_name

#include <linux/module.h>    // needed by all modules
#include <linux/kernel.h>    // needed for KERN_INFO
#include <linux/interrupt.h> // needed for request_interrupt()
#include <linux/gpio.h>      // needed for gpio_XXX() functions
#include <asm/io.h>          // needed for ioremap & iounmap
#include <asm/delay.h>       // needed for udelay()

#include <linux/proc_fs.h>   // needed for functions to manage /proc/xxx files
#include <linux/seq_file.h>  /* sequential file handles the read/write calls to /proc/files */
#include <linux/workqueue.h> // needed for DECLARE_WORKQUEUE() macro
#include <asm/uaccess.h>     // needed for copy_from_user() function
#include <asm/atomic.h>      // needed for atomic_cmpxchg() function
#include <linux/spinlock.h>  // needed for spinlock_t and it's functions

// forward declaration of pilot private functions
static int  __init rpc_init(void);
static void __exit rpc_exit(void);

static void pilot_internals_init(void);

static int                    rpc_gpio_init    (gpio_config_t* gpios, int count);
static void                   rpc_gpio_deinit  (gpio_config_t* gpios, int count);
static void                   rpc_gpio_init_alt(int gpio, int alt);
volatile static unsigned int* rpc_gpio_mmap_get(void);
volatile static unsigned int* rpc_gpio_mmap_set(void);
volatile static unsigned int* rpc_gpio_mmap_clr(void);
static void                   rpc_gpio_mmap_free(volatile unsigned int* p);
static void                   rpc_gpio_log_inputs(gpio_config_t* gpios, int count);

volatile static unsigned int* rpc_spi0_init_mem(void);
static void                   rpc_spi0_deinit_mem(volatile unsigned int* spi);
static void                   rpc_spi0_enable(volatile unsigned int* spi);
static void                   rpc_spi0_reset(volatile unsigned int* spi, int spiclk);
static spidata_t              rpc_spi0_dataexchange(volatile unsigned int* spi, spidata_t mosi);

static void                   rpc_spi0_transmit(volatile unsigned int* spi);
static void                   rpc_spi0_handle_received_data(spidata_t miso);

static int         rpc_irq_data_m2r_init (int gpio);
static void        rpc_irq_data_m2r_deinit(int irq);
static irqreturn_t rpc_irq_data_m2r_handler(int irq, void* dev_id);
static void        rpc_irq_data_m2r_work_queue_handler(struct work_struct *work);

static void rpc_proc_init(void);
static void rpc_proc_deinit(void);

static int pilot_try_get_module_uid(int module_index, int timeout, pilot_eeprom_uid_t **uid);
static int pilot_try_get_module_hid(int module_index, int timeout, pilot_eeprom_hid_t **hid);

static int pilot_handle_module_assignment(module_slot_t slot, const pilot_module_type_t *module_type);
static void rpc_unregister_driver(driver_t* driver);
static void rpc_unassign_slot(module_slot_t slot);

// static variables
static internals_t _internals; /* holds all private fields */

static gpio_config_t _internals_gpio[] = {
  {9, GPIO_MODE_ALT},  // SPI_MISO
  {10, GPIO_MODE_ALT}, // SPI_MOSI
  {11, GPIO_MODE_ALT}, // SPI_SCLK
  {DATA_M2R, GPIO_MODE_INPUT}
};

/* workqueue for bottom half of DATA_M2R irq handler */
static DECLARE_WORK(_internals_irq_data_m2r_work, rpc_irq_data_m2r_work_queue_handler);

/* spinlock to secure the spi data transfer */
//static spinlock_t QueueLock;

/* count of gpio configurations */
static int _internals_gpio_count = sizeof(_internals_gpio) / sizeof(gpio_config_t);

/* declare the file operations for the /proc/pilot/ files - the initialization is done after after the necessary functions are defined */
static const struct file_operations proc_pilot_spiclk_fops,
                                    proc_pilot_stats_fops,
                                    proc_pilot_last_recv_cmd_fops,
                                    proc_pilot_module_type_fops,
                                    proc_pilot_module_firmware_type_fops,
                                    proc_pilot_module_uid_fops,
                                    proc_pilot_module_hid_fops,
                                    proc_pilot_module_fid_fops,
                                    proc_pilot_module_eeprom_user_fops,
                                    proc_pilot_test_fops,
                                    proc_pilot_uid_fops;

/* module entry point function, gets called when loading the module */
module_init(rpc_init);

/* module exit point function, gets called when unloading the module */
module_exit(rpc_exit);

/* initialize the internal members */
static void pilot_internals_init()
{
  /* initialize the cmd handler list */
  INIT_LIST_HEAD(&_internals.list_cmd_handler);

  _internals.spiclk = 250;
}

/* initialization function, called from module_init() */
static int __init rpc_init(void)
{
  int i, retValue = SUCCESS;
  LOG_DEBUG("rpc_init()");

  pilot_internals_init();

  /* driver ids are 1 based */
  for (i = 0; i < DRIVERS_COUNT; i++)
    _internals.drivers[i].id = i+1;

  /* initialize the gpios */
  if (rpc_gpio_init( _internals_gpio, _internals_gpio_count ) != SUCCESS) {

    /* if allocation fails, release all acquired gpios again */
    rpc_gpio_deinit(_internals_gpio, _internals_gpio_count );

    LOG(KERN_ERR, "rpc_gpio_init() failed");
    retValue = -1;
  }
  else {
    LOG_DEBUG("rpc_gpio_init() succeeded");

    // memory map the Spi0  
    _internals.Spi0 = rpc_spi0_init_mem();

    // init the /proc/XXX files
    rpc_proc_init();

    // memory map the gpios for faster access
    _internals.GpioGet = rpc_gpio_mmap_get();
    _internals.GpioSet = rpc_gpio_mmap_set();
    _internals.GpioClr = rpc_gpio_mmap_clr();

    // log gpio input state
    rpc_gpio_log_inputs( _internals_gpio, _internals_gpio_count );

    // reset the spi
    rpc_spi0_reset(_internals.Spi0, _internals.spiclk);

    // enable CS
    rpc_spi0_enable(_internals.Spi0);

    /* install the irq handler */
    _internals.irq_data_m2r = rpc_irq_data_m2r_init(DATA_M2R);

    LOG_DEBUG("initialization complete");
  }

  return retValue;
}

/* cleanup function, called from module_exit() */
static void __exit rpc_exit(void)
{
  LOG_DEBUG("rpc_exit() called.");

  // free the Spi0 memory mapping
  rpc_spi0_deinit_mem(_internals.Spi0);

  // free the irq again
  rpc_irq_data_m2r_deinit(_internals.irq_data_m2r);

  // free the gpios
  rpc_gpio_deinit(_internals_gpio, _internals_gpio_count);

  // free /proc/XXX files
  rpc_proc_deinit();

  LOG_DEBUG("Goodbye!");
}

// ************* START GPIO specific functions ******************

/* request the gpios from the supplied configuration and calls rpc_gpio_init_alt() for them */
static int rpc_gpio_init(gpio_config_t* gpios, int count)
{
  int i, ret = SUCCESS;

  LOG_DEBUG ( "rpc_gpio_init() called with %i gpios", count );

  for (i = 0; i < count; i++)
  {
    if ( gpio_request_one( gpios[i].gpio, GPIOF_IN, "SPI" ) == 0 )
    {
      gpios[i].gpio_is_requested = 1; // mark the gpio as successfully requested
      LOG_DEBUG( "gpio_request_one(%i) succeeded", gpios[i].gpio);

      switch (gpios[i].gpio_mode) {

        case GPIO_MODE_INPUT:
          LOG_DEBUG( "setting gpio %i as input", gpios[i].gpio);
          if (gpio_direction_input(gpios[i].gpio) != SUCCESS)
            LOG(KERN_ERR, "gpio_direction_input(%i) failed", gpios[i].gpio);
          else
            LOG_DEBUG("gpio_direction_input(%i) succeeded", gpios[i].gpio);
          break;

        case GPIO_MODE_OUTPUT:
          LOG_DEBUG( "setting gpio %i as output", gpios[i].gpio);

          if (gpio_direction_output(gpios[i].gpio, 0) != SUCCESS)
            LOG(KERN_ERR, "gpio_direction_output(%i) failed", gpios[i].gpio);
          else {
            LOG_DEBUG("gpio_direction_output(%i) succeeded", gpios[i].gpio);
          }
          break;

        case GPIO_MODE_ALT:
            // set the alternative function according
          LOG_DEBUG( "setting gpio alt for gpio=%i and alt=%i", gpios[i].gpio, gpios[i].gpio_alternative);
          rpc_gpio_init_alt( gpios[i].gpio, gpios[i].gpio_alternative );        
          break;
      }
      
      
    }
    else {
      LOG( KERN_ERR, "gpio_request_one(%i) failed", gpios[i].gpio );
      ret--;
    }
  }

  if ( ret == SUCCESS )
    LOG_DEBUG( "rpc_gpio_init() returned successfully" );

  return ret;
}

/* frees the allocated gpios specified in the supplied configuration */
static void rpc_gpio_deinit(gpio_config_t* gpios, int count)
{
  int i;
  
  LOG_DEBUG( "rpc_gpio_deinit() called" );

  // reset the gpio
  //GPIO_CLR(INT_R2M);

  // free the memory mapped gpio again
  //rpc_gpio_mmap_free(_internals.p_gpioirq);
  rpc_gpio_mmap_free(_internals.GpioGet);
  rpc_gpio_mmap_free(_internals.GpioSet);
  rpc_gpio_mmap_free(_internals.GpioClr);

  // frees all gpios that we successfully requested  
  for (i = 0; i < count; i++)
  {
    if ( gpios[i].gpio_is_requested ) 
    {
      LOG_DEBUG( "Freeing gpio %i", gpios[i].gpio );
      gpio_free( gpios[i].gpio );
      gpios[i].gpio_is_requested = 0;
    }
  }
}

/* sets the supplied alternative configuration for the gpio */
static void rpc_gpio_init_alt(int gpio, int alt)
{
  volatile unsigned int* p;
  int address;
  
  LOG_DEBUG( "rpc_gpio_init_alt(gpio=%i, alt=%i) called", gpio, alt);
  
  // calc the memory address for manipulating the gpio
  //address = GPIO_BASE + (4 * (gpio / 10) );
  address = GPIO_BASE + (4 * (gpio / 10) );

  // map the gpio into kernel memory
  p = ioremap(address, 4);

  // if the mapping was successful
  if (p != NULL) {
    
    LOG_DEBUG ( "ioremap returned %X", (int)p );

    // set the gpio to the alternative mapping
    (*p) |= (((alt) <= 3 ? (alt) + 4 : (alt) == 4 ? 3 : 2) << (((gpio)%10)*3));

    // free the gpio mapping again
    iounmap(p);
  }
}

/* logs the current state of the inputs if in debug mode */
static void rpc_gpio_log_inputs(gpio_config_t* gpios, int count)
{
  int i;
  for (i = 0; i < count; i++)
  {
    if (gpios[i].gpio_mode == GPIO_MODE_INPUT) {
      LOG_DEBUG("gpio(%i): %i", gpios[i].gpio, GPIO_GET(gpios[i].gpio));
    }
  }
}

/* memory maps the gpio for faster access  */
volatile static unsigned int* rpc_gpio_mmap(int address)
{
  volatile unsigned int* p;

  LOG_DEBUG("rpc_gpio_mmap() called");

  // map the gpio address into kernel memory
  p = ioremap(address, 4);

  if (p == NULL ){
    LOG(KERN_ERR, "ioremap(%X) failed", address);
  }
  LOG_DEBUG("ioremap(%X) returned %X", address, (int)p);

  return p;
}

/* address for gpio inputs */
volatile static unsigned int* rpc_gpio_mmap_get()
{  
  return rpc_gpio_mmap(GPIO_BASE + (4 * 13));
}

volatile static unsigned int* rpc_gpio_mmap_set()
{
  return rpc_gpio_mmap(GPIO_BASE + (4 * 7));
}

volatile static unsigned int* rpc_gpio_mmap_clr()
{
  return rpc_gpio_mmap(GPIO_BASE + (4 * 10));
}

static void rpc_gpio_mmap_free(volatile unsigned int* p) 
{
  LOG_DEBUG("rpc_gpio_mmap_free() called with %X", (int)p);
  if (p != NULL) {
    iounmap(p);
  }
}
// *************** END GPIO specific functions *****************

static int rpc_irq_data_m2r_init(int gpio)
{
    int err, irq;

  // map the gpio to the irq
  irq = gpio_to_irq(gpio);

  if (irq < 0) {
    LOG(KERN_ERR, "gpio_to_irq(%i) failed with returncode %i", gpio, irq);
    return irq;
  }
  else
  {
    err = request_irq( irq,                               // the irq we want to receive
                       rpc_irq_data_m2r_handler,          // our irq handler function
                       IRQF_SHARED | IRQF_TRIGGER_RISING, // irq is shared and triggered on the rising edge
                       IRQ_DEV_NAME,                      // device name that is displayed in /proc/interrupts
                       (void*)(rpc_irq_data_m2r_handler)  // a unique id, needed to free the irq
                       );

    LOG_DEBUG( "request_irq(%i..) returned %i", irq, err);
    return (err == SUCCESS) ? irq : -1;
  }
}

static void rpc_irq_data_m2r_deinit(int irq)
{
  LOG_DEBUG("rpc_irq_data_m2r_deinit() called with irq=%i", irq);
  if (irq != 0) {
    free_irq(irq, (void*)(rpc_irq_data_m2r_handler));
  }
}

/* description: gets invoked when the stm changes the DATA pin */
static irqreturn_t rpc_irq_data_m2r_handler(int irq, void* dev_id)
{
  //if (GPIO_GET(DATA_M2R))
  //{
    schedule_work( &_internals_irq_data_m2r_work );
  //}

  return IRQ_HANDLED;
}

/* description: work queue handler is scheduled by the bottom half of the DATA_M2R interrupt */
static void rpc_irq_data_m2r_work_queue_handler(struct work_struct* args)
{
  /* start the spi transmission */
  rpc_spi0_transmit(_internals.Spi0);
}

// ****************** START SPI specific functions ************************

/* map the physical memory that we need for spi0 access */
volatile static unsigned int* rpc_spi0_init_mem(void)
{
  // in user space we would do mmap() call, in kernel space we do ioremap
  // call ioremap to map the physical address to something we can use
  unsigned int* p = ioremap(SPI0_BASE, 12);

  LOG_DEBUG("ioremap(%X) returned %X", SPI0_BASE, (int)p);
  LOG_DEBUG("spi0: %X spi0+1 %X spi0+2 %X", *p, *(p+1), *(p+2) );

  return p;
}

/* frees the memory mapping for spi0 access */
static void rpc_spi0_deinit_mem(volatile unsigned int* spi0)
{
  if (spi0 != NULL)
    iounmap(spi0);
}

/* sets the clock speed and clears all FIFOs and Status bits */
static void rpc_spi0_reset(volatile unsigned int* spi, int spiclk)
{
  /* SPI clock is calculated 250 MHz / divisor = clock, eg. 250 / 50 = 5 MHz */
  //SPI0_CLKSPEED(spi) = 250;  /* 250 / 125 = 2 MHz, works for stm32f405 */
  //SPI0_CLKSPEED(spi) = 64; 
  //SPI0_CLKSPEED(spi) = 125;
  SPI0_CLKSPEED(spi) = spiclk;

  // clear FIFOs and all status bits
  SPI0_CNTLSTAT(spi) = SPI0_CS_CLRALL;

  // clear done bit
  SPI0_CNTLSTAT(spi) = SPI0_CS_DONE;
}

static void rpc_spi0_enable(volatile unsigned int* spi0)
{
  // enable SPI interface
  SPI0_CNTLSTAT(spi0) = SPI0_CS_CHIPSEL0 | SPI0_CS_ACTIVATE;
}

static void rpc_spi0_transmit(volatile unsigned int* spi0)
{
  spidata_t recv, send;
  int data_available;
  int max_data_per_irq = 1000, data_count = 0; /* guard against stuck DATA_M2R high pin! */

  //spin_lock( &QueueLock );

  while (1)
  {
    /* get the next data to send */
    if ((data_available = queue_dequeue(&_internals.TxQueue, &send)) == 0)
      send = (target_invalid << 8);

    if ( data_available || (GPIO_GET(DATA_M2R) && data_count++ <= max_data_per_irq))
    {
      recv = rpc_spi0_dataexchange(_internals.Spi0, send);

      rpc_spi0_handle_received_data(recv);

      if (data_available)
        _internals.stats.sent_byte_count[(send >> 8)]++;
    }
    else
      break;

  }

  //spin_unlock(&QueueLock);
}

static void pilot_spi0_handle_received_base_cmd(pilot_cmd_t *cmd)
{
  int i, data_index;

  /* handle the cmd */
  switch (cmd->type)
  {
    /* if it's a bufferstate cmd, store the bufferstate */
  case pilot_cmd_type_bufferstate:
    _internals.stm_bufferstate = cmd->data[(int)pilot_bufferstate_index_value];
    break;

  case pilot_cmd_type_eeprom_uid_get:
    /* update the base uid */
    if (cmd->target == target_base)
    {
      _internals.uid = INT_FROM_BYTES((cmd->data));
      _internals.uid_is_updated = 1; /* mark the base uid as updated */
    }
    else /* update the module uids */
    {
      for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_UID_LENGTH; i++)
        _internals.modules[target_t_get_module_slot(cmd->target)].uid.uid[i] = cmd->data[i];
      _internals.modules[target_t_get_module_slot(cmd->target)].uid_is_updated = 1; /* mark the uid as updated */
    }

    break;

  case pilot_cmd_type_eeprom_hid_get:
    /* update the hid */
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_HID_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].hid.data[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].hid_is_updated = 1; /* mark the hid as updated */
    break;

  case pilot_cmd_type_eeprom_fid_get:
    /* update the fid */
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_FID_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].fid.data[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].fid_is_updated = 1; /* mark the fid as updated */
    break;

  case pilot_cmd_type_eeprom_userdata_get:
    /* update the user data */
    data_index = eeprom_decode_data_index(cmd->target);
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_DATA_LENGTH; i++)
      _internals.modules[eeprom_decode_module_slot(cmd->target)].user[data_index].data[i] = cmd->data[i];
    _internals.modules[eeprom_decode_module_slot(cmd->target)].user_is_updated[data_index] = 1;
    break;

  case pilot_cmd_type_module_type_get:
    /* update the module_type */
    for (i = 0; i < pilot_cmd_t_data_size && i < MODULE_TYPE_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].type.name[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].type_is_updated = 1; /* mark the type as updated */
    break;

  case pilot_cmd_type_test_run:
    /* update the test results */
    _internals.test_result.index_failed = (int)cmd->data[(int)pilot_test_run_index_failed_index];
    _internals.test_result.count_failed_low = (int)cmd->data[(int)pilot_test_run_index_failed_low_count];
    _internals.test_result.count_failed_high = (int)cmd->data[(int)pilot_test_run_index_failed_high_count];
    _internals.test_result.count_failed = (int)cmd->data[(int)pilot_test_run_index_failed_count];
    _internals.test_result.count_success = (int)cmd->data[(int)pilot_test_run_index_success_count];
    _internals.test_result.count_total = (int)cmd->data[(int)pilot_test_run_index_total_count];
    _internals.test_result.result = (pilot_test_run_result_t)cmd->data[(int)pilot_test_run_index_result];
    _internals.test_result_is_updated = 1; /* mark the test results as updated */
     break;
  }
}

/* description: handles the specified command
                by looping through all registered command handlers */
static void pilot_spi0_handle_received_cmd(pilot_cmd_t cmd)
{
  struct list_head *ptr;
  pilot_cmd_handler_t *handler;

  LOG_DEBUG("pilot_spi0_handle_received_cmd() called");

  /* handle the cmd */
  pilot_spi0_handle_received_base_cmd(&cmd);

  /* give each registered cmd handler the chance to handle the cmd */
  spin_lock(&_internals.list_cmd_handler_lock);

  /* call each registered cmd_handler with the command */
  list_for_each(ptr, &_internals.list_cmd_handler) {
    handler = list_entry(ptr, pilot_cmd_handler_t, list);
    handler->callback_cmd_received(cmd);
  }

  spin_unlock(&_internals.list_cmd_handler_lock);
}

/* description: this function handles data received from the stm that is targetted at the base driver */
static void pilot_spi0_handle_received_cmd_byte(char data)
{
  int i;
  const unsigned long timeout = HZ; /* timeout in 1 sec */

  /* if it's a new cmd, then start the timeout */
  if (_internals.current_cmd.index == pilot_current_cmd_index_target)
    _internals.current_cmd_timeout = jiffies + timeout;

  /* if the timeout elapsed... */
  if (time_after(jiffies, _internals.current_cmd_timeout))
  {
    LOG_INFO("cmd timed out! was: index=%X, target=%X, type=%X", _internals.current_cmd.index, _internals.current_cmd.cmd.target, _internals.current_cmd.cmd.type);

    /* ...reset the cmd */
    _internals.current_cmd.index = pilot_current_cmd_index_target;
    /* ...reset the timeout */
    _internals.current_cmd_timeout = jiffies + timeout;
  }

  LOG_DEBUG("pilot_spi0_handle_received_cmd_byte(data=%i), current_cmd.index=%i", data, _internals.current_cmd.index);

  /* add the data to the current rpcp command */
  switch (_internals.current_cmd.index)
  {
    /* the first byte of a new command is the target_t */
    case pilot_current_cmd_index_target:
      _internals.current_cmd.cmd.target = (target_t)data;
      break;

    /* the second byte of a command is the cmd_type */
    case pilot_current_cmd_index_type:
      _internals.current_cmd.cmd.type = (pilot_cmd_type_t)data;
      break;

    /* the following bytes of a command are the data bytes */
    default:
      i = _internals.current_cmd.index - pilot_current_cmd_index_data_begin;
      if (i >= 0 && i < pilot_cmd_t_data_size)
        _internals.current_cmd.cmd.data[i] = data;
      break;
  }

  /* increment the index */
  _internals.current_cmd.index++;

  /* is the received command completed? */
  if (_internals.current_cmd.index >= sizeof(pilot_cmd_t))
  {
    LOG_DEBUG("cmd completed - target: %i, type: %i", _internals.current_cmd.cmd.target, _internals.current_cmd.cmd.type);

    /* reset the current cmd index */
    _internals.current_cmd.index = pilot_current_cmd_index_target;

    /* update the stats */
    _internals.stats.recv_cmd_count++;
    if ((int)_internals.current_cmd.cmd.type < MAX_CMD_TYPE) /* sanity check */
      _internals.stats.recv_cmd_type_count[(int)_internals.current_cmd.cmd.type]++;
    memcpy(&_internals.last_recv_cmd, &_internals.current_cmd.cmd, sizeof(pilot_cmd_t));

    /* handle the command by copy */
    pilot_spi0_handle_received_cmd(_internals.current_cmd.cmd);
  }
}

/* description: handles a received word from the spi */
/* performance critical - enable inlining */
static void rpc_spi0_handle_received_data(spidata_t miso)
{
  target_t target;
  module_slot_t slot; module_port_t port;
  char data;

  //LOG_DEBUG("rpc_spi0_handle_received_data(miso=%x)", miso);

  target = (miso >> 8);
  data   = (miso & 0xFF);

  if (target == target_base)
  {
    pilot_spi0_handle_received_cmd_byte(data);
    _internals.stats.recv_byte_count[target]++; /* update the stats */
  }
  else if (target <= target_module4_port2)
  {
    slot = target_t_get_module_slot(target);
    port = target_t_get_module_port(target);

    if (slot >= 0 && slot < MODULES_COUNT
     && port >= 0 && port < MODULE_PORT_COUNT)
    {
      if (_internals.modules[slot].driver != NULL)
         _internals.modules[slot].driver->driver->callback_recv(slot, port, data);
      _internals.stats.recv_byte_count[target]++; /* update the stats */
    }
  }
  else if (_internals.stream_callback.callbacks[target] != NULL) /* look for a registered stream target */
    _internals.stream_callback.callbacks[target](data);
}

/* description: spi data exchange sends / receives 8-bits over spi */
/* performance critical - enable inlining */
inline static spidata_t rpc_spi0_dataexchange(volatile unsigned int* spi0, 
                                              spidata_t mosi)
{
  spidata_t miso;
  u8 data1, data2;
  int status;

  data1 = mosi >> 8;
  data2 = mosi & 0xFF;

  // enable SPI interface
  SPI0_CNTLSTAT(spi0) = SPI0_CS_CHIPSEL0 | SPI0_CS_ACTIVATE;

  // send the data
  SPI0_FIFO(spi0) = data1;
  SPI0_FIFO(spi0) = data2;

  // wait for the SPI to be ready
  do {
    status = SPI0_CNTLSTAT(spi0);
  } while ((status & SPI0_CS_DONE) == 0);

  SPI0_CNTLSTAT(spi0) = SPI0_CS_DONE;

  // data should now be in the receiver
  data1 = SPI0_FIFO(spi0);
  data2 = SPI0_FIFO(spi0);

  //LOG_INFO("d1: %X, d2: %X", data1, data2);

  miso = ( (data1 << 8) | data2 );

  LOG_DEBUG("send: %X, recv: %X", mosi, miso);

  return miso;
}

// ******************* END SPI specific funtions ***************************

// ******************* START proc file system ******************************

#define pilot_proc_directory_name "pilot"
#define pilot_proc_drivers_name   "drivers" /* filename: /proc/pilot/drivers */
#define pilot_proc_modules_name   "modules" /* filename: /proc/pilot/modules */
static char* pilot_proc_module_dir_names[] = { "module1", "module2", "module3", "module4" };
#define pilot_proc_module_type_name "type"  /* filename: /proc/pilot/moduleX/type */
#define pilot_proc_module_firmware_type_name "firmware_type" /* /proc/pilot/moduleX/firmware_type */
#define pilot_proc_module_eeprom_dir_name "eeprom" /* directory name: /proc/pilot/moduleX/eeprom */
#define pilot_proc_module_uid "uid" /* filename: /proc/pilot/moduleX/uid */
#define pilot_proc_module_hid "hid" /* filename: /proc/pilot/moduleX/hid */
#define pilot_proc_module_fid "fid" /* filename: /proc/pilot/moduleX/fid */
static char* pilot_proc_module_eeprom_user_names[] = { "user00", "user01", "user02", "user03", "user04", "user05", "user06", "user07", "user08", "user09", "user10", "user11" };
#define pilot_proc_spiclk_name "spiclk" /* filename: /proc/pilot/spiclk */
#define pilot_proc_stats_name "stats"/* /proc/pilot/stats */
#define pilot_proc_last_recv_cmd_name "last_recv_cmd"  /* /proc/pilot/last_recv_cmd */
#define pilot_proc_test_name "test" /* /proc/pilot/test */
#define pilot_proc_uid_name "uid" /* filename: /proc/pilot/uid */

#define RPC_PROC_BUFFER_SIZE 255

#define PROC_USER_DATA_GET_INT(module_index, data_index) ( (module_index << 8) | (data_index) )
#define PROC_USER_DATA_GET_MODULE_INDEX(value) ( value >> 8 )
#define PROC_USER_DATA_GET_DATA_INDEX(value) ( value & 0xFF )


/* description: registers the files used in /proc/... */
static void rpc_proc_init(void)
{
  int i, j;
  struct proc_dir_entry *base_dir, *module_dir, *eeprom_dir;

  /* register & store the /proc/pilot directory */
  _internals.proc_pilot_dir = base_dir = proc_mkdir_mode(pilot_proc_directory_name /* name */, 0 /* default */, NULL /* parent dir */);

  /* register /proc/pilot/spiclk file */
  proc_create_data(pilot_proc_spiclk_name, 0666 /* r+w */, base_dir, &proc_pilot_spiclk_fops, NULL);
  
  /* register the /proc/pilot/stats file */
  proc_create_data(pilot_proc_stats_name, 0666 /* r+w */, base_dir, &proc_pilot_stats_fops, NULL);

  /* register the /proc/pilot/last_recv_cmd */
  proc_create_data(pilot_proc_last_recv_cmd_name, 0 /* r */, base_dir, &proc_pilot_last_recv_cmd_fops, NULL);

  /* register the /proc/pilot/test file */
  proc_create_data(pilot_proc_test_name, 0 /* r */, base_dir, &proc_pilot_test_fops, NULL);

  /* register the /proc/pilot/eeprom file */
  proc_create_data(pilot_proc_uid_name, 0 /* r */, base_dir, &proc_pilot_uid_fops, NULL);

  /* register a directory foreach module (/proc/pilot/module1-4) */
  for (i = 0; i < MODULES_COUNT; i++)
  {
    /* create and store a directory proc entry for the module */
    _internals.proc_pilot_modules_dir[i] = module_dir = proc_mkdir_mode(pilot_proc_module_dir_names[i], 0, base_dir);

    /* create a read/writable proc entry for the modules type (/proc/pilot/moduleX/type)*/
    proc_create_data(pilot_proc_module_type_name, 0666, module_dir, &proc_pilot_module_type_fops, (void*)i);

    /* create a readable proc entry for the module firmware type (/proc/pilot/moduleX/firmware_type) */
    proc_create_data(pilot_proc_module_firmware_type_name, 0, module_dir, &proc_pilot_module_firmware_type_fops, (void*)i);

    /* register a directory for all eeprom entries */
    _internals.proc_pilot_modules_eeprom_dir[i] = eeprom_dir = proc_mkdir_mode(pilot_proc_module_eeprom_dir_name, 0,  module_dir);

    /* create a readable proc entry for the eeprom uid */
    proc_create_data(pilot_proc_module_uid, 0, eeprom_dir, &proc_pilot_module_uid_fops, (void*)i);

    /* create a read- and writeable proc entry for the eeprom hid */
    proc_create_data(pilot_proc_module_hid, 0666 /* r+w */, eeprom_dir, &proc_pilot_module_hid_fops, (void*)i);

    /* create a read- and writeable proc entry for the eeprom fid */
    proc_create_data(pilot_proc_module_fid, 0666 /* r+w */, eeprom_dir, &proc_pilot_module_fid_fops, (void*)i);

    /* create read- and writeable proc entries for user content */
    for (j = 0; j < EEPROM_USER_DATA_COUNT; j++)
      proc_create_data(pilot_proc_module_eeprom_user_names[j], 0666, eeprom_dir, &proc_pilot_module_eeprom_user_fops, (void*)PROC_USER_DATA_GET_INT(i,j));
  }
}

/* description: removes the 'proc' filesystem entries ('/proc/pilotdrivers', '/proc/pilotmodules', '/proc/pilotmodule0', '/proc/pilotmodule1', '/proc/pilotmodule2' and '/proc/pilotmodule3') */
static void rpc_proc_deinit(void)
{
  int i, j;

  /* remove /proc/pilot/spiclk */
  remove_proc_entry(pilot_proc_spiclk_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/stats */
  remove_proc_entry(pilot_proc_stats_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/last_recv_cmd */
  remove_proc_entry(pilot_proc_last_recv_cmd_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/test */
  remove_proc_entry(pilot_proc_test_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/uid */
  remove_proc_entry(pilot_proc_uid_name, _internals.proc_pilot_dir);

  /* remove the /proc/pilot/moduleX entries */
  for (i = 0; i < MODULES_COUNT; i++)
  {
    /* remove the /proc/pilot/moduleX/type entry */
    remove_proc_entry(pilot_proc_module_type_name, _internals.proc_pilot_modules_dir[i]);

    /* remove the /proc/pilot/moduleX/firmware_type entry */
    remove_proc_entry(pilot_proc_module_firmware_type_name, _internals.proc_pilot_modules_dir[i]);

    /* remove the /proc/pilot/moduleX/eeprom/uid entry */
    remove_proc_entry(pilot_proc_module_uid, _internals.proc_pilot_modules_eeprom_dir[i]);

    /* remove the /proc/pilot/moduleX/eeprom/hid entry */
    remove_proc_entry(pilot_proc_module_hid, _internals.proc_pilot_modules_eeprom_dir[i]);

    /* remove the /proc/pilot/moduleX/eeprom/fid entry */
    remove_proc_entry(pilot_proc_module_fid, _internals.proc_pilot_modules_eeprom_dir[i]);

    /* remove the /proc/pilot/moduleX/eeprom/user0-11 entries */
    for (j = 0; j < EEPROM_USER_DATA_COUNT; j++)
      remove_proc_entry(pilot_proc_module_eeprom_user_names[j], _internals.proc_pilot_modules_eeprom_dir[i]);

    /* remove the /proc/pilot/moduleX/eeprom directory */
    remove_proc_entry(pilot_proc_module_eeprom_dir_name, _internals.proc_pilot_modules_dir[i]);

    /* remove the /proc/pilot/moduleX directory */
    remove_proc_entry(pilot_proc_module_dir_names[i], _internals.proc_pilot_dir);
  }

  /* remove the base /proc/pilot directory */
  remove_proc_entry(pilot_proc_directory_name, NULL);
}

static int pilot_proc_pilot_stats_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
  int new_value, ret;

  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL;
  else if (new_value == 0)
  {
    memset(&_internals.stats, 0, sizeof(stats_t)); /* clear the stats */
    ret = count;
  }
  else
    ret = -EINVAL;

  return ret;
}

static void pilot_set_module_hid(module_slot_t module, const pilot_eeprom_hid_t *hid)
{
  int i;
  pilot_cmd_t cmd;

  /* create the set module hid command */
  cmd.target = target_t_from_module_slot_and_port(module, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_hid_set;
  for (i = 0; i < EEPROM_HID_LENGTH; i++)
    cmd.data[i] = hid->data[i];

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

/* timeout for eeprom requests get uid and get hid */
#define EEPROM_TIMEOUT 1000

/* description: callback function that gets called by the kernel, when /proc/pilot/moduleX/hid is written to */
static int pilot_proc_pilot_module_hid_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  pilot_eeprom_hid_t hid; int not_copied;
  
  /* get the slot */
  module_slot_t module = (module_slot_t)PDE_DATA(file->f_inode);

  LOG_DEBUG("pilot_proc_pilot_module_hid_write() called for module=%i", module);

  /* copy the input from the cmdline */
  not_copied = copy_from_user(hid.data, buf, EEPROM_HID_LENGTH);

  /* try to set the hid of the module */
  pilot_set_module_hid(module, &hid);

  return count;
}

static void pilot_set_module_fid(module_slot_t module, const pilot_eeprom_fid_t *fid)
{
  int i;
  pilot_cmd_t cmd;

  /* create the set module hid command */
  cmd.target = target_t_from_module_slot_and_port(module, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_fid_set;
  for (i = 0; i < EEPROM_FID_LENGTH; i++)
    cmd.data[i] = fid->data[i];

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

static int pilot_proc_pilot_module_fid_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  pilot_eeprom_fid_t fid; int not_copied;

  /* get the slot */
  module_slot_t module = (module_slot_t)PDE_DATA(file->f_inode);

  LOG_DEBUG("pilot_proc_pilot_module_fid_write() called for module=%i", module);

  /* copy the input from userspace */
  not_copied = copy_from_user(fid.data, buf, EEPROM_FID_LENGTH);

  /* try to set the fid of the module */
  pilot_set_module_fid(module, &fid);

  return count;
}

/* description: callback function that gets called, when the /proc/pilotmoduleX is written to */
static int pilot_proc_pilot_module_type_write(struct file *file, const char *buf, size_t count, loff_t *off)
{
  int notCopied, slot;
  pilot_module_type_t module_type;

  slot = (int)PDE_DATA(file->f_inode);
  memset(&module_type, 0, sizeof(pilot_module_type_t));

  LOG_DEBUG("pilot_proc_pilot_module_type_write() called for slot=%i with count=%lu", slot , (unsigned long)count);

  // make sure we're not out of bounds
  if (count > sizeof(pilot_module_type_t))
    count = sizeof(pilot_module_type_t);

  // copy the data from userspace to kernelspace
  notCopied = copy_from_user(module_type.name, buf, count);

  LOG_DEBUG("copy_from_user() returned %i", notCopied);

  if (pilot_handle_module_assignment(slot, &module_type) == SUCCESS)
    return count;
  else
    return -EFAULT;
}

static void pilot_set_module_eeprom_data(int module_index, int data_index, pilot_eeprom_data_t *data)
{
  int i;
  pilot_cmd_t cmd;

  /* create the set module eeprom user command */
  cmd.target = eeprom_encode_module_slot_and_data_index(module_index, data_index);
  cmd.type = pilot_cmd_type_eeprom_userdata_set;
  for (i = 0; i < EEPROM_DATA_LENGTH; i++)
    cmd.data[i] = data->data[i];

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

// **************** END proc file system functions *****************************************

static int pilot_try_get_module_type(int module_index, int timeout, pilot_module_type_t **type)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int is_timedout = 0;

  /* get the module type */
  pilot_module_type_t *module_type = &_internals.modules[module_index].type;

  LOG_DEBUG("pilot_try_get_module_type(module_index=%i, timeout=%i) called", module_index, timeout);

  /* clear the update flag of the module type */
  _internals.modules[module_index].type_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_module_type_get;
  pilot_send_cmd(&cmd);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the pilot_module_type is updated or the timeout occurs */
  while (_internals.modules[module_index].type_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  if (!is_timedout)
    *type = module_type;
  else
  {
    LOG_INFO("pilot_try_get_module_type() timedout while waiting for module_type!");
    *type = NULL;
  }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_uid(int module_index, int timeout, pilot_eeprom_uid_t **uid)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int timedout = 0;
  LOG_DEBUG("pilot_try_update_module_uid(module_index = %i, timeout = %i) called", module_index, timeout);

  /* clear the uid_is_updated flag for the module */
  _internals.modules[module_index].uid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_uid_get;

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* pick a time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the uid is updated or the timeout occurs */
  while (_internals.modules[module_index].uid_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }

  if (!timedout)
    *uid = &_internals.modules[module_index].uid;
  else
    LOG_INFO("pilot_try_get_module_uid() timeout reached while waiting for uid!");

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_hid(int module_index, int timeout, pilot_eeprom_hid_t **hid)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int timedout = 0;

  /* clear the hid_is_updated flag for the module */
  _internals.modules[module_index].hid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_hid_get;

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* pick a  time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the uid is updated or the timeout occurs */
  while (_internals.modules[module_index].hid_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }

  if (!timedout)
    *hid = &_internals.modules[module_index].hid;
  else
    LOG_INFO("pilot_try_get_module_hid() timeout reached while waiting for hid!");

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_fid(int module_index, int timeout, pilot_eeprom_fid_t **fid)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int timedout = 0;

  /* clear the fid_is_updated flag for the module */
  _internals.modules[module_index].fid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_fid_get;

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* pick a  time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the uid is updated or the timeout occurs */
  while (_internals.modules[module_index].fid_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }

  if (!timedout)
    *fid = &_internals.modules[module_index].fid;
  else
    LOG_INFO("pilot_try_get_module_fid() timeout reached while waiting for fid!");

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_eeprom_data(int module_index, int user_data_index, int timeout, pilot_eeprom_data_t **data)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int timedout = 0;

  /* clear the is_updated flag for the module & data */
  _internals.modules[module_index].user_is_updated[user_data_index] = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_userdata_get;
  cmd.data[pilot_eeprom_userdata_index_number] = (char)user_data_index;

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* pick a  time thats timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the data is updated of the timeout occurs */
  while (_internals.modules[module_index].user_is_updated[user_data_index] ==  0)
    if (time_after(jiffies, timestamp))
    {
      timedout = 1;
      break;
    }

  if (!timedout)
    *data = &_internals.modules[module_index].user[user_data_index];
  else
    LOG_INFO("pilot_try_get_module_eeprom_data() timeout reached while waiting for data!");

  return timedout ? -1 : SUCCESS;
}

/* description: assign a driver to the specified slot */
static int pilot_assign_slot(int driverId, module_slot_t slot, const pilot_module_type_t *module_type)
{
  module_t* m;
  driver_t* d;
  LOG_DEBUG("assigning driverId=%i to slot=%i", driverId, slot);

  rpc_unassign_slot(slot); // first unassign it

  if (driverId > 0 && driverId <= DRIVERS_COUNT)
  {
    d = &_internals.drivers[driverId - 1]; // 1-based
    m = &_internals.modules[slot];

    // is there a registered driver yet?
    if (d->driver != NULL)
    {
      if (d->driver->callback_assign_slot(slot, module_type))
      {
        m->driver = d;
        return SUCCESS;
      }
    }
  }

  return -1;
}

static int pilot_handle_module_assignment(module_slot_t slot, const pilot_module_type_t *module_type)
{
  int i, ret = -1;

  /* check if it starts with '0' or 'none' then unassign it */
  if (module_type->name[0] == '0' ||
     (module_type->name[0] == 'n' && module_type->name[1] == 'o' && module_type->name[2] == 'n' && module_type->name[3] == 'e'))
  {
    rpc_unassign_slot(slot);
    ret = SUCCESS;
  }
  else
  {
    /* look for a driver that can handle it */
    for (i = 0; i < DRIVERS_COUNT; i++)
      if (_internals.drivers[i].driver != NULL)
        if (_internals.drivers[i].driver->callback_can_assign != NULL)
          if (_internals.drivers[i].driver->callback_can_assign(module_type) == SUCCESS)
            if (pilot_assign_slot(_internals.drivers[i].id, slot, module_type) == SUCCESS)
            {
              ret = SUCCESS;
              break;
            }
  }


  return ret;
}

static void pilot_auto_configure_module(module_slot_t slot)
{
  pilot_module_type_t *module_type;
  LOG_DEBUG("pilot_auto_configuration(slot=%i) called", slot);

  if (pilot_try_get_module_type((int)slot, 100, &module_type) == SUCCESS)
    pilot_handle_module_assignment(slot, module_type);
}

void pilot_auto_configure(void)
{
  int i;
  for (i = 0; i < MODULES_COUNT; i++)
    pilot_auto_configure_module((module_slot_t)i);
}

/* description: unregisters the supplied driver by first unassigning the modules to the driver, then removing the driver description */
static void rpc_unregister_driver(driver_t* driver)
{
  int i;
  if (driver != NULL)
  {
    // unassign the module slots that the driver occupies
    for (i = 0; i < MODULES_COUNT; i++)
      if (_internals.modules[i].driver == driver) {
        rpc_unassign_slot(i);
      }

    // unregister the driver description
    driver->driver = NULL;
    driver->totalBytesRecv = 0;
    driver->totalBytesSent = 0;
  }
}

static void rpc_unassign_slot(module_slot_t slot)
{
   module_t* m = &_internals.modules[slot];
   
   if (m->driver != NULL)
   {
     m->driver->driver->callback_unassign_slot(slot); // inform the driver of the unassignment
     m->driver = NULL;
   }
}

static int pilot_proc_pilot_spiclk_show(struct seq_file *file, void *data)
{
  seq_printf(file, "%i\n", _internals.spiclk);
  return 0;
}

static int pilot_proc_pilot_spiclk_write(struct file* file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, ret;

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL; /* return an error if the conversion fails */
  else
  {
    /* sanity check the value before setting the spiclk */
    if (new_value < 10 || new_value > 1000)
      ret = -EINVAL;
    else
    {
      /* store the new spi clk value and reset the spi */
      _internals.spiclk = new_value;
      rpc_spi0_reset(_internals.Spi0, _internals.spiclk);
      ret = count;
    }
  }
  return ret;
}

static int pilot_proc_pilot_spiclk_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_spiclk_show, NULL);
}

/* file operations for /proc/pilot/spiclk */
static const struct file_operations proc_pilot_spiclk_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_spiclk_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_spiclk_write
};

static int pilot_proc_pilot_stats_show(struct seq_file *file, void *data)
{
  int i;
  seq_printf(file, "\t\tbase\tm0\tm1\tm2\tm3\n");

  seq_printf(file, "recv bytes\t%u\t%u\t%u\t%u\t%u\n",
    _internals.stats.recv_byte_count[target_base],
    _internals.stats.recv_byte_count[target_module1_port1] + _internals.stats.recv_byte_count[target_module1_port2],
    _internals.stats.recv_byte_count[target_module2_port1] + _internals.stats.recv_byte_count[target_module2_port2],
    _internals.stats.recv_byte_count[target_module3_port1] + _internals.stats.recv_byte_count[target_module3_port2],
    _internals.stats.recv_byte_count[target_module4_port1] + _internals.stats.recv_byte_count[target_module4_port2]);

  seq_printf(file, "sent bytes\t%u\t%u\t%u\t%u\t%u\n",
    _internals.stats.sent_byte_count[target_base],
    _internals.stats.sent_byte_count[target_module1_port1] + _internals.stats.sent_byte_count[target_module1_port2],
    _internals.stats.sent_byte_count[target_module2_port1] + _internals.stats.sent_byte_count[target_module2_port2],
    _internals.stats.sent_byte_count[target_module3_port1] + _internals.stats.sent_byte_count[target_module3_port2],
    _internals.stats.sent_byte_count[target_module4_port1] + _internals.stats.sent_byte_count[target_module4_port2]);

  seq_printf(file, "recv cmds\t%u\n", _internals.stats.recv_cmd_count);
  for (i = 0; i < MAX_CMD_TYPE; i++)
    if (_internals.stats.recv_cmd_type_count[i] > 0)
      seq_printf(file, "%s\t%i\n", pilot_cmd_type_to_name(i), _internals.stats.recv_cmd_type_count[i]);

  seq_printf(file, "sent cmds\t%u\n", _internals.stats.sent_cmd_count);
  for (i = 0; i < MAX_CMD_TYPE; i++)
    if (_internals.stats.sent_cmd_type_count[i] > 0)
      seq_printf(file, "%s\t%i\n", pilot_cmd_type_to_name(i), _internals.stats.sent_cmd_type_count[i]);

  return 0;
}

static int pilot_proc_pilot_stats_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_stats_show, NULL);
}

/* file operations for /proc/pilot/stats */
static const struct file_operations proc_pilot_stats_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_stats_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_stats_write
};

static int pilot_proc_pilot_last_recv_cmd_show(struct seq_file *file, void *data)
{
  int i;

  seq_printf(file, "module: %i / port: %i\n", target_t_get_module_slot(_internals.last_recv_cmd.target), target_t_get_module_port(_internals.last_recv_cmd.target));
  seq_printf(file, "type: %s\n", pilot_cmd_type_to_name(_internals.last_recv_cmd.type));

  for (i = 0; i < sizeof(pilot_cmd_t); i++)
    seq_printf(file, "%i\t", i);
  seq_printf(file, "\n");

  for (i = 0; i < sizeof(pilot_cmd_t); i++)
    seq_printf(file, "%#04x\t", *(((char*)&_internals.last_recv_cmd) + i));
  seq_printf(file, "\n");

  /* todo */
  return 0;
}

static int pilot_proc_pilot_last_recv_cmd_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_last_recv_cmd_show, NULL);
}

/* file operations for /proc/pilot/last_recv_cmd */
static const struct file_operations proc_pilot_last_recv_cmd_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_last_recv_cmd_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release
};


static int pilot_try_get_test_result(int timeout, test_result_t *result)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int is_timedout = 0;

  LOG_DEBUG("pilot_try_get_test_result() called");

  /* reset the flag */
  _internals.test_result_is_updated = 0;

  /* send the test run command to the pilot */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_test_run;
  pilot_send_cmd(&cmd);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the test_result is updated or the timeout occurs */
  while (_internals.test_result_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  if (!is_timedout)
    *result = _internals.test_result;
  else
  {
    LOG_INFO("pilot_try_get_test_result() timedout while waiting for test result!");
  }
  return is_timedout ? -1 : SUCCESS;
}

static int pilot_proc_pilot_test_show(struct seq_file *file, void *data)
{
  int ret;
  test_result_t result;

  if (pilot_try_get_test_result(1000, &result) == SUCCESS)
  {
    /* print the result */
    seq_printf(file, "failed_index: %i\n", result.index_failed);
    seq_printf(file, "failed low:   %i\n", result.count_failed_low);
    seq_printf(file, "failed high:  %i\n", result.count_failed_high);
    seq_printf(file, "failed:       %i\n", result.count_failed);
    seq_printf(file, "succeeded:    %i\n", result.count_success);
    seq_printf(file, "total:        %i\n", result.count_total);
    seq_printf(file, "status:       ");
    switch (result.result)
    {
      case pilot_test_run_result_not_supported:
        seq_printf(file, "test run is not supported on this image\n");
        break;
      case pilot_test_run_result_success:
        seq_printf(file, "test run completed successfully\n");
        break;
      case pilot_test_run_result_failed:
        seq_printf(file, "test run failed\n");
        break;
      default:
        seq_printf(file, "returned unknown test result\n");
        break;
    }

    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_proc_pilot_test_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_test_show, NULL);
}

/* file operations for /proc/pilot/test */
static const struct file_operations proc_pilot_test_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_test_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release
};

/*  */
static int pilot_try_get_uid(int timeout, uint32_t *uid)
{
  pilot_cmd_t cmd;
  unsigned long timestamp;
  int is_timedout = 0;

  /* reset the uid_is_updated flag */
  _internals.uid_is_updated = 0;

  /* send the request */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_eeprom_uid_get;
  pilot_send_cmd(&cmd);

  /* pick a time that is timeout ms in the future */
  timestamp = jiffies + (timeout * HZ / 1000);

  /* wait until the test_result is updated or the timeout occurs */
  while (_internals.uid_is_updated == 0)
    if (time_after(jiffies, timestamp))
    {
      is_timedout = 1;
      break;
    }

  if (!is_timedout)
    *uid = _internals.uid_is_updated;
  else
  {
    LOG_INFO("pilot_try_get_uid() timedout while waiting for uid!");
  }
  return is_timedout ? -1 : SUCCESS;
}

static int pilot_proc_pilot_uid_show(struct seq_file *file, void *data)
{
  int ret;
  uint32_t uid;

  if (pilot_try_get_uid(1000, &uid) == SUCCESS)
  {
    seq_printf(file, "%u\n", uid);
    ret = 0;
  }
  else
    ret = -EFAULT;

  return ret;
}

static int pilot_proc_pilot_uid_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_uid_show, NULL);
}

static const struct file_operations proc_pilot_uid_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_uid_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release
};


static int pilot_proc_pilot_module_type_show(struct seq_file *file, void *data)
{
  module_t *m = &_internals.modules[(int)file->private];

  seq_printf(file, "%i %s\n",
    m->driver == NULL ? 0 : m->driver->id,
    m->driver == NULL ? "NULL" : m->driver->driver->name);

  return 0;
}

static int pilot_proc_pilot_module_type_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_module_type_show, PDE_DATA(inode));
}

/* file operations for /proc/pilot/moduleX/type */
static const struct file_operations proc_pilot_module_type_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_module_type_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_module_type_write
};

static int pilot_proc_pilot_module_firmware_type_show(struct seq_file *file, void *data)
{
  int ret;
  pilot_module_type_t *module_type;
  int module_index = (int)file->private;

  if (pilot_try_get_module_type(module_index, 100, &module_type) == SUCCESS)
  {
    seq_write(file, module_type->name, MODULE_TYPE_LENGTH);
    ret = 0;
  }
  else 
    ret = -EFAULT;

  return ret;
}

static int pilot_proc_pilot_module_firmware_type_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_module_firmware_type_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_firmware_type_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_module_firmware_type_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release
};

static int pilot_proc_pilot_module_uid_show(struct seq_file *file, void *data)
{
  int ret;
  int module_index = (int)file->private;
  pilot_eeprom_uid_t *uid;

  if (pilot_try_get_module_uid(module_index, EEPROM_TIMEOUT, &uid) != SUCCESS) /* try to read get the uid from pilot */
    ret = -EFAULT; /* return an error */
  else /* format the uid as an uint64 */
  {
    seq_printf(file, "%llu", *((uint64_t*)uid->uid));
    ret = 0;
  }

  return ret;
}

static int pilot_proc_pilot_module_uid_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_module_uid_show, PDE_DATA(inode));
}

/* file operations for /proc/pilot/moduleX/uid */
static const struct file_operations proc_pilot_module_uid_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_module_uid_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release
};

static int pilot_proc_module_hid_show(struct seq_file *file, void *data)
{
  pilot_eeprom_hid_t *hid;
  int ret, module_index = (int)file->private;

  if (pilot_try_get_module_hid(module_index, EEPROM_TIMEOUT, &hid) != SUCCESS)
    ret = -EFAULT;
  else
  {
    seq_write(file, hid->data, EEPROM_HID_LENGTH);
    ret = 0;
  }

  return ret;
}

static int pilot_proc_module_hid_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_module_hid_show, PDE_DATA(inode));
}

/* file operations for /proc/pilot/moduleX/hid */
static const struct file_operations proc_pilot_module_hid_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_module_hid_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_module_hid_write
};

static int pilot_proc_module_fid_show(struct seq_file *file, void *data)
{
  pilot_eeprom_fid_t *fid;
  int ret, module_index = (int)file->private;

  if (pilot_try_get_module_fid(module_index, EEPROM_TIMEOUT, &fid) != SUCCESS)
    ret = -EFAULT;
  else
  {
    seq_write(file, fid->data, EEPROM_HID_LENGTH);
    ret = 0;
  }

  return ret;
}

static int pilot_proc_module_fid_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_module_fid_show, PDE_DATA(inode));
}

static const struct file_operations proc_pilot_module_fid_fops = {
  .owner = THIS_MODULE,
  .open = pilot_proc_module_fid_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
  .write = pilot_proc_pilot_module_fid_write
};

static int pilot_proc_module_eeprom_user_show(struct seq_file *file, void *data)
{
  pilot_eeprom_data_t *eeprom_data;
  int ret, module_index, data_index;
  module_index = PROC_USER_DATA_GET_MODULE_INDEX((int)file->private);
  data_index = PROC_USER_DATA_GET_DATA_INDEX((int)file->private);

  LOG_INFO("pilot_proc_module_eeprom_user_show() module_index=%i, data_index=%i", module_index, data_index);

  if (pilot_try_get_module_eeprom_data(module_index, data_index, EEPROM_TIMEOUT, &eeprom_data) != SUCCESS)
    ret = -EFAULT;
  else
  {
    seq_write(file, eeprom_data->data, EEPROM_DATA_LENGTH);
    ret = 0;
  }

  return ret;
}

static int pilot_proc_module_eeprom_user_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_module_eeprom_user_show, PDE_DATA(inode));
}

static int pilot_proc_pilot_module_eeprom_user_write (struct file *file, const char __user *buf, size_t count, loff_t *off)
{
  pilot_eeprom_data_t data; int not_copied;
  
  /* get the slot */
  module_slot_t module = (module_slot_t)PROC_USER_DATA_GET_MODULE_INDEX(((int)PDE_DATA(file->f_inode)));
  int data_index = PROC_USER_DATA_GET_DATA_INDEX(((int)PDE_DATA(file->f_inode)));

  LOG_DEBUG("pilot_proc_pilot_module_eeprom_user_write() called for module=%i and data_index=%i", module, data_index);

  /* copy the input from the cmdline */
  not_copied = copy_from_user(data.data, buf, EEPROM_DATA_LENGTH);

  /* try to set the data of the module */
  pilot_set_module_eeprom_data(module, data_index, &data);

  return count;
}

static const struct file_operations proc_pilot_module_eeprom_user_fops = {
  .owner = THIS_MODULE,
  .open = pilot_proc_module_eeprom_user_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
  .write = pilot_proc_pilot_module_eeprom_user_write
};

// **************** START exported functions for the extension modules *********************

/* description: registers the supplied extension module driver */
int pilot_register_driver(register_driver_t* driver)
{
  int i;
  LOG_DEBUG("pilot_register_driver() called with name='%s'", driver->name);

  // find the first free driver
  for (i = 0; i < DRIVERS_COUNT; i++)
  {    
    if (_internals.drivers[i].driver == NULL)
    {
      // we found a free driver
      _internals.drivers[i].driver = driver;
      return _internals.drivers[i].id;
    }
  }

  LOG(KERN_ERR, "pilot_register_driver() couldn't allocate a free driver slot");

  // we didn't have a slot free for the driver
  return -1;
}

/* description: unregisters the supplied extension module driver */
void pilot_unregister_driver(int driverId)
{ 
  LOG_DEBUG("pilot_unregister_driver() called with driverId=%i", driverId);
  
  // driver ids are 1 based
  rpc_unregister_driver(&_internals.drivers[driverId-1]); 
}

/* description: sends the supplied data to the specified module */
int pilot_try_send(target_t target, const char* data, int count)
{ 
  int ret = 0, i;
  LOG_DEBUG("pilot_send(target=%i, count=%i) called", target, count);

  //spin_lock( &QueueLock );

  for (i = 0; i < count; i++)
    if (queue_enqueue( &_internals.TxQueue, ( (int)target << 8 | data[i] ) ))
      ret++;
    else
      break;

  //spin_unlock( &QueueLock );

  /* start the spi transmission */
  while(!schedule_work( &_internals_irq_data_m2r_work ));

  return ret;
}

void pilot_send(target_t target, const char* data, int count)
{
  int sent_bytes = 0;
  while ((sent_bytes += pilot_try_send(target, data + sent_bytes, count - sent_bytes)) < count)
    cpu_relax();
}

/* sends the supplied command to the stm by calling pilot_send() internally */
void pilot_send_cmd(pilot_cmd_t* cmd)
{
  LOG_DEBUG("pilot_send_cmd: target: %i, type: %i, data: %x %x %x %x %x %x %x %x",(int) cmd->target,(int) cmd->type, cmd->data[0], cmd->data[1], cmd->data[2], cmd->data[3], cmd->data[4], cmd->data[5], cmd->data[6], cmd->data[7]);
  pilot_send(target_base, (char*)cmd, sizeof(pilot_cmd_t));

  /* update the stats */
  _internals.stats.sent_cmd_count++;
  if (cmd->type < MAX_CMD_TYPE)
    _internals.stats.sent_cmd_type_count[(int)cmd->type]++;
}

/* description: gets the number of remaining free bytes of the send buffer for the specified target */
int pilot_get_free_send_buffer_size(target_t target)
{
  LOG_DEBUG("pilot_get_free_send_buffer_size(target=%i) called", target);
  //if (target == target_t_invalid || target > target_t_module4_port2)
  //{
  //  if (printk_ratelimit())
  //    LOG(KERN_ERR, "pilot_get_free_send_buffer_size() called with illegal target=%i", target);
  //  return -1;
  //}
  //else
  return queue_get_room(&_internals.TxQueue);
}

stm_bufferstate_t pilot_get_stm_bufferstate()
{
  return _internals.stm_bufferstate;
}

/* description: registers a command handler with the main driver */
int pilot_register_cmd_handler(pilot_cmd_handler_t* cmd_handler)
{
  int result = SUCCESS;

  LOG_DEBUG("pilot_register_cmd_handler() called");

  spin_lock(&_internals.list_cmd_handler_lock);

  /* add the supplied cmd_handler to the end of the list */
  list_add_tail(&cmd_handler->list, &_internals.list_cmd_handler);

  spin_unlock(&_internals.list_cmd_handler_lock);

  return result;
}

/* description: unregisters a command handler from the main driver */
int pilot_unregister_cmd_handler(pilot_cmd_handler_t* cmd_handler)
{
  int result = SUCCESS;

  LOG_DEBUG("pilot_unregister_cmd_handler() called");

  spin_lock(&_internals.list_cmd_handler_lock);

  list_del(&cmd_handler->list);

  spin_unlock(&_internals.list_cmd_handler_lock);

  return result;
}

/* description: gets the proc_dir_entry of the specified module directory ('/proc/pilot/moduleX') */
struct proc_dir_entry* pilot_get_proc_module_dir(module_slot_t slot)
{
  LOG_DEBUG("pilot_get_proc_module_dir(slot=%i) called", slot);

  return _internals.proc_pilot_modules_dir[(int)slot];
}

/* description: gets the proc_dir_entry of the base pilot directory ('/proc/pilot/') */
struct proc_dir_entry* pilot_get_proc_pilot_dir()
{
  LOG_DEBUG("pilot_get_proc_pilot_dir() called");

  return _internals.proc_pilot_dir;
}

void pilot_register_stream_handler(target_t target, void(*stream_callback)(char data))
{
  _internals.stream_callback.callbacks[target] = stream_callback;
}

void pilot_unregister_stream_handler(target_t target)
{
  _internals.stream_callback.callbacks[target] = NULL;
}

// END exported functions for extension modules ***************************
