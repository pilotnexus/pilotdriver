// defines
#define MODULE_NAME "pilot"
#define IRQ_DEV_NAME MODULE_NAME
/* reserve a driver slot for every module, as we could have 4 different modules loaded that need 4 different drivers */
#define DRIVERS_COUNT MODULES_COUNT

// includes
#include "module.h"          // include defines that describe the module
#include "common.h"          // common defines for logging
#include "types.h"
#include "queue.h"           // fifo queue, circle buffered
#include "pilotstm.h"         // pilot <-> stm communication defines
#include "commands.h"        // pilot_cmd_type_to_name
#include "stm32flash.h"        // pilot_cmd_type_to_name

#include <linux/module.h>    // needed by all modules
#include <linux/kernel.h>    // needed for KERN_INFO
#include <linux/interrupt.h> // needed for request_interrupt()
#include <linux/gpio.h>      // needed for gpio_XXX() functions
#include <linux/smp.h>		 // needed for get_cpu()
#include <asm/io.h>          // needed for ioremap & iounmap
#include <linux/delay.h>       // needed for udelay()
#include <linux/proc_fs.h>   // needed for functions to manage /proc/xxx files
#include <linux/seq_file.h>  /* sequential file handles the read/write calls to /proc/files */
#include <linux/uaccess.h>     // needed for copy_from_user() function
#include <asm/atomic.h>      // needed for atomic_cmpxchg() function
#include <linux/spinlock.h>  // needed for spinlock_t and it's functions
#include <linux/kthread.h>   // kthread_run()
#include <linux/ktime.h>     //ktime support
#include <linux/spi/spi.h>   //  needed for SPI interface
#include <linux/workqueue.h>  //needed for workqueue

#ifdef USE_SWAIT_QUEUE
#include <linux/swait.h>
#else
#include <linux/wait.h>
#endif

#include <linux/gpio.h> 
#include <linux/of.h> 
#include <linux/of_gpio.h> 

// forward declaration of pilot private functions
static int  __init rpc_init(void);
static void __exit rpc_exit(void);

static void pilot_internals_init(void);

static void                   rpc_spi0_handle_received_data(spidata_t miso);

static int         rpc_irq_data_m2r_init (int gpio);
static void        rpc_irq_data_m2r_deinit(int irq);
static irqreturn_t rpc_irq_data_m2r_handler(int irq, void* dev_id);

//static void       spi_complete(void *context);
//static void        rpc_irq_data_m2r_work_queue_handler(struct work_struct *work); /* workqueue definition */
//static void        rpc_irq_data_m2r_work_queue_handler(unsigned long data); /* tasklet definition */

static void rpc_proc_init(void);
static void rpc_proc_deinit(void);

static int pilot_try_get_module_uid(int module_index, int timeout, pilot_eeprom_uid_t **uid);
static int pilot_try_get_module_hid(int module_index, int timeout, pilot_eeprom_hid_t **hid);

static int pilot_handle_module_assignment(module_slot_t slot, const pilot_module_type_t *module_type);
static void rpc_unregister_driver(driver_t* driver);
static void rpc_unassign_slot(module_slot_t slot);

static int pilot_comm_task(void *vp);

#define MAX_SPI_DATAEXCHANGE_BYTES_IN_IRQ 200
#define MAX_SPI_DATAEXCHANGE_BYTES_IN_THREAD 1000

// static variables
u64 start_spi_us, end_spi_us, average_us = 10000000;
uint32_t average_spi_load;

struct task_struct *pilot_receive_thread;
struct task_struct *pilot_comm_thread;

static int stop_threads = 0;

DEF_WQ_HEAD(data_received_wait_queue);
DEF_WQ_HEAD(comm_wait_queue);

//static DEFINE_MUTEX(spi_transmit_lock);
//static DEFINE_SPINLOCK(spi_txrx_lock);
//unsigned long spi_txrx_lock_flags;

static internals_t _internals; /* holds all private fields */

//static struct workqueue_struct *comm_wq;

/* workqueue for bottom half of DATA_M2R irq handler */
//static DECLARE_WORK(_internals_irq_data_m2r_work, rpc_irq_data_m2r_work_queue_handler);

/* tasklet */
//DECLARE_TASKLET( _internals_irq_data_m2r_tasklet, rpc_irq_data_m2r_work_queue_handler, (unsigned long) 0 );

/* spinlock to secure the spi data transfer */
//static spinlock_t QueueLock;

const int POLYNOME = 0x04C11DB7;
const uint32_t INITIAL_CRC_VALUE = 0xFFFFFFFF;

static uint32_t CrcSoftwareFunc(uint32_t Initial_Crc, uint32_t Input_Data)
{
  int bindex = 0;
  uint32_t Crc = 0;

  Crc = Initial_Crc ^ Input_Data;

  for (bindex = 0; bindex < sizeof(int) * 8; bindex = bindex + 1)
  {
    if ((Crc & 0x80000000) > 0)
    {
      Crc = (Crc << 1) ^ POLYNOME;
    }
    else
    {
      Crc = (Crc << 1);
    }
  }
  return Crc;
}

static uint crc(const char *data, int len)
{
  int i;
  uint32_t crc = INITIAL_CRC_VALUE;

  if (len % 4 != 0)
  {
    LOG_DEBUG("CRC length mismatch. Needs to be a multiple of 4 but is %i", len);
    return 0;
  }
  for (i = 0; i < len; i+=4)
    crc = CrcSoftwareFunc(crc, (uint32_t)data[i + 3] << 24 | (uint32_t)data[i + 2] << 16 | (uint32_t)data [i+1] << 8 | (uint32_t)data[i]);

  return crc;
}

/* declare the file operations for the /proc/pilot/ files - the initialization is done after after the necessary functions are defined */
static const struct file_operations proc_pilot_spiclk_fops,
                                    proc_pilot_reset_fops,
                                    // proc_pilot_firmware_fops,
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

static int assign_gpio(struct device *dev, int* pin, int direction, const char* pin_name)
{
  uint32_t ret;
  char gpio_name[100];
  strcpy(gpio_name, pin_name);
  strcat(gpio_name, "-gpios");
  *pin = of_get_named_gpio(dev->of_node, gpio_name, 0);
  if (gpio_is_valid(*pin))
  {
    LOG_DEBUG("gpio=%d valid", *pin);
    ret = devm_gpio_request_one(dev, *pin, direction, pin_name);
    if (ret)
    {
      LOG(KERN_ERR, "Failed to request GPIO (%d): error %d", *pin, ret);
    }
    else
    {
      LOG_DEBUG("Request GPIO (%d) successful", *pin);

      return 1;
    }
  }
  else
  {
    LOG(KERN_ERR, "devm_gpio_request_one() failed for GPIO (%d)", *pin);
  }

  return 0;
}

static int32_t pilot_spi_probe(struct spi_device * spi)
{
  struct device_node *np = spi->dev.of_node;
  uint32_t ret;

  LOG_DEBUG("pilot_spi_probe called with cs=%d", spi->chip_select);

  if (spi->chip_select == 0)
  {
     LOG_DEBUG("pilot_spi_probe() for CS0 called");
    //spi->modalias = "pilot-device-driver";
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 25000000; //todo - get from devicetree
    spi->bits_per_word = 8;
    spi->chip_select = 0;

   ret=spi_setup(spi);
    dev_info(&spi->dev,
          "default setup (%d): cs %d (cs_gpio=%d): %d Hz: bpw %u, mode 0x%x\n",
    ret, spi->chip_select, spi->cs_gpio, spi->max_speed_hz, spi->bits_per_word,
    spi->mode);

    _internals.spi0 = spi;

    _internals.spi_xfer.tx_buf = &_internals.send;
    _internals.spi_xfer.rx_buf = &_internals.recv,
    _internals.spi_xfer.len = 2;
    _internals.spi_xfer.speed_hz = 4000000; //todo - get from devicetree

    if (np)
    {
      if (assign_gpio(&spi->dev, &_internals.data_m2r_gpio, GPIOF_IN, "data_m2r")) 
      {
            // install the irq handler
          _internals.irq_data_m2r = rpc_irq_data_m2r_init(_internals.data_m2r_gpio);
      }

      //assign_gpio(&spi->dev, &_internals.reset_gpio, GPIOF_OUT_INIT_LOW, "reset");
      //assign_gpio(&spi->dev, &_internals.boot_gpio, GPIOF_OUT_INIT_LOW, "boot");
    }
    else 
    {
      LOG(KERN_ERR, "Error getting SPI device!");
    }
    //spi setup completed, start communication thread
    pilot_comm_thread = kthread_run(pilot_comm_task, NULL, "K_PILOT_COMM_TASK");

  }


  return 0;
}

static int32_t pilot_spi_remove(struct spi_device * spi)
{
  if (spi->chip_select == 0)
  {
    LOG_DEBUG("pilot_spi_remove() for CS0 called");

    // free the irq again
    rpc_irq_data_m2r_deinit(_internals.irq_data_m2r);

    // free the gpios
    devm_gpio_free(&spi->dev, _internals.data_m2r_gpio);
    //devm_gpio_free(&spi->dev, _internals.reset_gpio);
    //devm_gpio_free(&spi->dev, _internals.boot_gpio);
  }

  return 0;
}

static struct spi_driver pilot_spi_driver = {
  .driver = 
  {
    .name = "pilot",
    .owner = THIS_MODULE
  },
  .probe = pilot_spi_probe,
  .remove = pilot_spi_remove
};

/* initialize the internal members */
static void pilot_internals_init()
{
  /* initialize the cmd handler list */
  INIT_LIST_HEAD(&_internals.list_cmd_handler);

  INIT_WQ_HEAD(_internals.test_result_is_updated_wq);
  INIT_WQ_HEAD(_internals.uid_is_updated_wq);

  start_spi_us = end_spi_us = ktime_to_us(ktime_get());
  _internals.spiclk = 4000000;
}

#define TARGET_INVALID_BLOCK_SIZE 12
static int pilot_comm_task(void *vp)
{
  static spidata_t target_invalid_block[TARGET_INVALID_BLOCK_SIZE] = { [ 0 ... TARGET_INVALID_BLOCK_SIZE-1 ] = target_invalid};
  int max_rx_len = 0;
  int is_invalid_block_transmit = 0;

  LOG_DEBUG("communication thread started");

  while (1)
  {
    //todo - checks are currently duplicated in rpc_spi0_transmit()
    if (WAIT_EVENT_INTERRUPTIBLE(comm_wait_queue, !queue_is_empty(&_internals.TxQueue) || gpio_get_value(_internals.data_m2r_gpio) || stop_threads == 1))
    {
      LOG_DEBUG("communication thread interrupted");
      return 1; //interrupted
    }

    if (stop_threads == 1)
      return 0; 


    LOG_DEBUGALL("spi_sync (m2r=%d, tx-queue-empty=%d, rx-queue-room=%d, pilot-recv-buffer-full=%d)", 
    gpio_get_value(_internals.data_m2r_gpio), 
    queue_is_empty(&_internals.TxQueue),
    queue_get_room(&_internals.RxQueue),
     _internals.pilot_recv_buffer_full);

    while ((!queue_is_empty(&_internals.TxQueue) || _internals.pilot_recv_buffer_full || (gpio_get_value(_internals.data_m2r_gpio))))
    {    
      if (queue_get_room(&_internals.RxQueue) > 0)
      {
        /* get the next data to send */
        /*
        if (_internals.pilot_recv_buffer_full)
          _internals.send = (target_invalid << 8);
        else if ((data_available = queue_dequeue(&_internals.TxQueue, &_internals.send)) == 0)
          _internals.send = (target_invalid << 8);
        */

        spi_message_init(&_internals.spi_message);

        if (queue_is_empty(&_internals.TxQueue))
        {  //transmit invalid target blocks, we just need the received data
          _internals.spi_xfer.tx_buf = target_invalid_block;
          _internals.spi_xfer.len = TARGET_INVALID_BLOCK_SIZE;
          is_invalid_block_transmit = 1;
          LOG_DEBUGALL("transmitting %d invalid blocks to receive data", TARGET_INVALID_BLOCK_SIZE);
        }
        else
        {
          is_invalid_block_transmit = 0;
          _internals.spi_xfer.len = queue_read_seq_block(&_internals.TxQueue, (void**)&_internals.spi_xfer.tx_buf);
          if (_internals.spi_xfer.len > 32)
            _internals.spi_xfer.len = 32; //chunk limit, much more does not work for some reason with spi_sync
          LOG_DEBUGALL("transmitting %d bytes in TxQueue starting from %d (write is %d)", _internals.spi_xfer.len, _internals.TxQueue.read,  _internals.TxQueue.write);
        }

        max_rx_len = queue_write_seq_block(&_internals.RxQueue, (void**)&_internals.spi_xfer.rx_buf);

        if (max_rx_len < _internals.spi_xfer.len)
        {
          _internals.spi_xfer.len = max_rx_len;
        }
        LOG_DEBUGALL("adjusting txrx bytes to %d bytes in RxQueue starting from %d", _internals.spi_xfer.len, _internals.RxQueue.write);          

        //_internals.spi_xfer.len = 2;
        //_internals.spi_xfer.tx_buf = &_internals.send;
        //_internals.spi_xfer.rx_buf = &_internals.recv,

        if (_internals.spi_xfer.len > 0)
        {
          //TODO - check if spi_sync fails and act how?
          spi_message_add_tail(&_internals.spi_xfer, &_internals.spi_message);
          spi_sync(_internals.spi0, &_internals.spi_message);

          if (is_invalid_block_transmit == 0)
          { //we did not use the TxQueue, to not advance pointer!
            queue_skip_read_block(&_internals.TxQueue, _internals.spi_xfer.len);
          }

          queue_skip_write_block(&_internals.RxQueue, _internals.spi_xfer.len);
          LOG_DEBUGALL("Done transmitting, adjusted TxQueue to %d and RxQueue to %d", _internals.TxQueue.read, _internals.RxQueue.write);          

          //signal data received thread to process data
          WAIT_WAKEUP(data_received_wait_queue);

          /*
          if ((_internals.recv & 0x0080) || _internals.recv == 0xFFFF) //pilot receive buffer full bit set (and the target byte is not 0xFF which occurs when MISO is always high)
            _internals.pilot_recv_buffer_full = true;
          else
            _internals.pilot_recv_buffer_full = false;
          */
          //queue_enqueue(&_internals.RxQueue, _internals.recv);

          //LOG_DEBUGALL("spi_sync done (tx=%x rx=%x)", _internals.send, _internals.recv); 

          //_internals.stats.sent_byte_count[(_internals.send >> 8)]++;
        }
      }
    }
  }
}

static int pilot_receive_task(void *vp)
{
  spidata_t recv;

  LOG_DEBUG("receive thread started");

  while (1)
  {
    if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(data_received_wait_queue, !queue_is_empty(&_internals.RxQueue) || stop_threads == 1,  (200 * HZ / 1000)) == -ERESTARTSYS)
    {
      LOG_DEBUG("receive thread interrupted");
      return 1; //interrupted
    }

    if (stop_threads == 1)
      return 0;

    while (queue_dequeue(&_internals.RxQueue, &recv) == 1)
    {
      rpc_spi0_handle_received_data(recv);
    }
  }
}

/* initialization function, called from module_init() */
static int __init rpc_init(void)
{
  int i, retValue = SUCCESS;
  LOG_DEBUG("pilot_init()");
  
  INIT_WQ_HEAD(data_received_wait_queue);
  INIT_WQ_HEAD(comm_wait_queue);

  pilot_internals_init();

  /* driver ids are 1 based */
  for (i = 0; i < DRIVERS_COUNT; i++)
    _internals.drivers[i].id = i+1;

  /* SPI init */  
  spi_register_driver(&pilot_spi_driver);

  pilot_receive_thread = kthread_run(pilot_receive_task, NULL, "K_PILOT_RECV_TASK");

    // init the /proc/XXX files
  LOG_DEBUG("rpc_proc_init()");
  rpc_proc_init();


    LOG_DEBUG("initialization complete");


  return retValue;
}

/* cleanup function, called from module_exit() */
static void __exit rpc_exit(void)
{
  LOG_DEBUG("rpc_exit() called.");
  
  if (pilot_receive_thread) 
  {
    LOG_DEBUG("Trying to stop threads...");
    //try to end thread
    stop_threads = 1;
    WAIT_WAKEUP(data_received_wait_queue);
    WAIT_WAKEUP(comm_wait_queue);
    kthread_stop(pilot_receive_thread);
    kthread_stop(pilot_comm_thread);

    pilot_receive_thread = NULL;
    LOG_DEBUG("Threads stopped.");
  }

  // free /proc/XXX files
  rpc_proc_deinit();
  
  spi_unregister_driver(&pilot_spi_driver);

  LOG_DEBUG("Goodbye!");
}

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
  //tasklet_schedule(&_internals_irq_data_m2r_tasklet); 
  //schedule_work(&_internals_irq_data_m2r_work);
  LOG_DEBUGALL("M2R Interrupt called, waking up comm thread");
  WAIT_WAKEUP(comm_wait_queue);
  return IRQ_HANDLED;
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
      WAIT_WAKEUP(_internals.uid_is_updated_wq);
    }
    else /* update the module uids */
    {
      for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_UID_LENGTH; i++)
        _internals.modules[target_t_get_module_slot(cmd->target)].uid.uid[i] = cmd->data[i];
      _internals.modules[target_t_get_module_slot(cmd->target)].uid_is_updated = 1; /* mark the uid as updated */
      WAIT_WAKEUP(_internals.modules[target_t_get_module_slot(cmd->target)].uid_is_updated_wq);
    }

    break;

  case pilot_cmd_type_eeprom_hid_get:
    /* update the hid */
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_HID_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].hid.data[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].hid_is_updated = 1; /* mark the hid as updated */
    WAIT_WAKEUP(_internals.modules[target_t_get_module_slot(cmd->target)].hid_is_updated_wq);
    break;

  case pilot_cmd_type_eeprom_fid_get:
    /* update the fid */
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_FID_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].fid.data[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].fid_is_updated = 1; /* mark the fid as updated */
    WAIT_WAKEUP(_internals.modules[target_t_get_module_slot(cmd->target)].fid_is_updated_wq);
    break;

  case pilot_cmd_type_eeprom_userdata_get:
    /* update the user data */
    data_index = eeprom_decode_data_index(cmd->target);
    for (i = 0; i < pilot_cmd_t_data_size && i < EEPROM_DATA_LENGTH; i++)
      _internals.modules[eeprom_decode_module_slot(cmd->target)].user[data_index].data[i] = cmd->data[i];
    _internals.modules[eeprom_decode_module_slot(cmd->target)].user_is_updated[data_index] = 1;
    WAIT_WAKEUP(_internals.modules[eeprom_decode_module_slot(cmd->target)].user_is_updated_wq[data_index]);
    break;

  case pilot_cmd_type_module_type_get:
    /* update the module_type */
    for (i = 0; i < pilot_cmd_t_data_size && i < MODULE_TYPE_LENGTH; i++)
      _internals.modules[target_t_get_module_slot(cmd->target)].type.name[i] = cmd->data[i];
    _internals.modules[target_t_get_module_slot(cmd->target)].type_is_updated = 1; /* mark the type as updated */
    WAIT_WAKEUP(_internals.modules[target_t_get_module_slot(cmd->target)].type_is_updated_wq);
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
    WAIT_WAKEUP(_internals.test_result_is_updated_wq);
    break;
  }
}

/* description: handles the specified command
                by looping through all registered command handlers */
static void pilot_spi0_handle_received_cmd(pilot_cmd_t cmd)
{
  struct list_head *ptr;
  pilot_cmd_handler_t *handler;

  LOG_DEBUG("pilot_spi0_handle_received_cmd: target: %x, type: %x, length: (%x), crc: %X",(int) cmd.target,(int) cmd.type, cmd.length, cmd.crc);

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
static void pilot_spi0_handle_received_cmd_byte(target_t target, uint8_t data)
{
  uint32_t crcindex;
  uint8_t length_parity;

#ifdef DEBUG
  int i;
#endif

  if (target == target_base)
  {
    /* ...reset the cmd */
    _internals.current_cmd.index = pilot_current_cmd_index_target;
    _internals.current_cmd.cmd_completion = 0x0;
    LOG_DEBUGALL("pilot_spi0_handle_received_cmd_byte MSG START (data=%i), current_cmd.index=%i", data, _internals.current_cmd.index);
  }

  if (target - target_base == _internals.current_cmd.index && _internals.current_cmd.index < pilot_current_cmd_index_data_begin)
  { //header
    switch((uint8_t)target)
    {
      case (uint8_t)target_base:_internals.current_cmd.cmd.target = (target_t)data; _internals.current_cmd.cmd_completion |= 0x1; break;
      case (uint8_t)target_base_type: _internals.current_cmd.cmd.type = (pilot_cmd_type_t)data; _internals.current_cmd.cmd_completion |= 0x2; break;
      case (uint8_t)target_base_length: 
        //calculate parity
        length_parity = data; 
        length_parity ^= length_parity >> 4;
        length_parity ^= length_parity >> 2;
        length_parity ^= length_parity >> 1;
        //if ( (data & 0x80) == ((length_parity & 1) !=0 ? 0 : 0x80))
        {
          _internals.current_cmd.length = (unsigned char)(data & 0x7F) << 2; 
          _internals.current_cmd.cmd.length = data;
          _internals.current_cmd.cmd_completion |= 0x4; 
        }
        LOG_DEBUGALL("pilot_received_cmd_byte() length: %i (0x%x)", _internals.current_cmd.length, _internals.current_cmd.cmd.length);

      break;
      case (uint8_t)target_base_reserved: _internals.current_cmd.cmd.reserved = data; _internals.current_cmd.cmd_completion |= 0x8; break;
      default:
        //general error in transmission structure, reset
        _internals.current_cmd.cmd_completion = 0x0;
      break;
    }
  }
  else if (target == target_base_data && 
    _internals.current_cmd.index >= pilot_current_cmd_index_data_begin && 
    _internals.current_cmd.cmd_completion == 0xF )
  { //header done, data block
    _internals.current_cmd.cmd.data[_internals.current_cmd.index-pilot_current_cmd_index_data_begin] = data;
  }
  else if (target == target_base_crc)
  { //crc check
    crcindex = _internals.current_cmd.index - pilot_current_cmd_index_data_begin - _internals.current_cmd.length;
    if (crcindex >= 0 && crcindex < 4) 
    {
      ((uint8_t *)&_internals.current_cmd.cmd.crc)[crcindex] = data;
      _internals.current_cmd.cmd_completion |= (0x10 << crcindex);
    }
  }
  else
  {
    //general error in transmission structure, reset
    _internals.current_cmd.cmd_completion = 0x0;
  }

  /* increment the index */
  _internals.current_cmd.index++;

  /* is the received command completed? */
  if ( (_internals.current_cmd.cmd_completion == 0xFF) && 
    (_internals.current_cmd.index >= pilot_cmd_t_size_without_data) && 
    (_internals.current_cmd.index >= (_internals.current_cmd.length + pilot_cmd_t_size_without_data)) )
  {
    //int32_t crc_check = crcFast((char *) &_internals.current_cmd.cmd, sizeof(pilot_cmd_t) - (sizeof(crc)));
    int checklen = pilot_current_cmd_index_data_begin + _internals.current_cmd.length;
    int32_t crc_check = crc((char *) &_internals.current_cmd.cmd, checklen);

    //LOG_DEBUG("cmd completed - target: %i, type: %i", _internals.current_cmd.cmd.target, _internals.current_cmd.cmd.type);
    LOG_DEBUG("pilot_received_cmd: target: %x, type: %x, length: %u (%x) (CRC received: %X, own calculated CRC: %X, crc check length = %i)",
      (int) _internals.current_cmd.cmd.target,(int) _internals.current_cmd.cmd.type, _internals.current_cmd.length,
      _internals.current_cmd.cmd.length, _internals.current_cmd.cmd.crc, crc_check, checklen);

    #ifdef DEBUG
      printk(KERN_CONT "received: '");

      for(i=0;i<_internals.current_cmd.length;i++)
        printk(KERN_CONT "%x ", _internals.current_cmd.cmd.data[i]);

      printk(KERN_CONT "'\n");
    #endif

    /* reset the current cmd index */
    _internals.current_cmd.index = pilot_current_cmd_index_target;


    if (crc_check == _internals.current_cmd.cmd.crc) 
    {
      /* update the stats */
      _internals.stats.recv_cmd_count++;
      if ((int)_internals.current_cmd.cmd.type < MAX_CMD_TYPE) /* sanity check */
        _internals.stats.recv_cmd_type_count[(int)_internals.current_cmd.cmd.type]++;
      memcpy(&_internals.last_recv_cmd, &_internals.current_cmd.cmd, sizeof(pilot_cmd_t));

      /* handle the command by copy */
      _internals.current_cmd.cmd.length &= 0x7F; //remove parity bit
      pilot_spi0_handle_received_cmd(_internals.current_cmd.cmd);
    }
    else
    {
      _internals.stats.crc_errors++;
      LOG_DEBUG("CRC ERROR");
    }
  }
}

#ifdef DEBUGALL
static const char * target_to_string(target_t target)
{
  static const char* str_target_invalid       = "target_invalid"; 
  static const char* str_target_module1_port1 = "target_module1_port1";
  static const char* str_target_module1_port2 = "target_module1_port2";
  static const char* str_target_module2_port1 = "target_module2_port1";
  static const char* str_target_module2_port2 = "target_module2_port2";
  static const char* str_target_module3_port1 = "target_module3_port1";
  static const char* str_target_module3_port2 = "target_module3_port2";
  static const char* str_target_module4_port1 = "target_module4_port1";
  static const char* str_target_module4_port2 = "target_module4_port2";
  static const char* str_target_plc_read      = "target_plc_read";
  static const char* str_target_plc_write     = "target_plc_write";
  static const char* str_target_base          = "target_base";
  static const char* str_target_base_type     = "target_base_type";
  static const char* str_target_base_length   = "target_base_length";
  static const char* str_target_base_reserved = "target_base_reserved";
  static const char* str_target_base_crc      = "target_base_crc";
  static const char* str_target_base_data     = "target_base_data";

  switch(target)
  {
    case target_invalid:       return str_target_invalid      ;
    case target_module1_port1: return str_target_module1_port1;
    case target_module1_port2: return str_target_module1_port2;
    case target_module2_port1: return str_target_module2_port1;
    case target_module2_port2: return str_target_module2_port2;
    case target_module3_port1: return str_target_module3_port1;
    case target_module3_port2: return str_target_module3_port2;
    case target_module4_port1: return str_target_module4_port1;
    case target_module4_port2: return str_target_module4_port2;
    case target_plc_read:      return str_target_plc_read     ;
    case target_plc_write:     return str_target_plc_write    ;
    case target_base:          return str_target_base         ;
    case target_base_type:     return str_target_base_type    ;
    case target_base_length:   return str_target_base_length  ;
    case target_base_reserved: return str_target_base_reserved;
    case target_base_crc:      return str_target_base_crc     ;
    case target_base_data:     return str_target_base_data    ;
    default: return str_target_invalid;
  }
}
#endif

/* description: handles a received word from the spi */
/* performance critical - enable inlining */
static void rpc_spi0_handle_received_data(spidata_t miso)
{
  target_t target;
  module_slot_t slot; module_port_t port;
  char data;

  //LOG_DEBUG("rpc_spi0_handle_received_data(miso=%x)", miso);

  target = (miso & 0xFF) & 0x7F;
  data   = (miso >> 8);

  //LOG_DEBUGALL("rpc_spi0_handle_received_data(target=%s, data=%x)", target_to_string(target), data);

  if (target >= target_base)
  {
    pilot_spi0_handle_received_cmd_byte(target, data);
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
#define pilot_proc_reset_name "reset" /* filename: /proc/pilot/reset */
#define pilot_proc_firmware_name "firmware" /* filename: /proc/pilot/firmware */
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

  /* register /proc/pilot/reset file */
  // proc_create_data(pilot_proc_reset_name, 0666 /* r+w */, base_dir, &proc_pilot_reset_fops, NULL);

  /* register /proc/pilot/firmware file */
  // proc_create_data(pilot_proc_firmware_name, 0666 /* r+w */, base_dir, &proc_pilot_firmware_fops, NULL);
  
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

    INIT_WQ_HEAD(_internals.modules[i].type_is_updated_wq);
    INIT_WQ_HEAD(_internals.modules[i].uid_is_updated_wq);
    INIT_WQ_HEAD(_internals.modules[i].hid_is_updated_wq);
    INIT_WQ_HEAD(_internals.modules[i].fid_is_updated_wq);

    /* create read- and writeable proc entries for user content */
    for (j = 0; j < EEPROM_USER_DATA_COUNT; j++)
    {
      INIT_WQ_HEAD(_internals.modules[i].user_is_updated_wq[j]);
      proc_create_data(pilot_proc_module_eeprom_user_names[j], 0666, eeprom_dir, &proc_pilot_module_eeprom_user_fops, (void*)PROC_USER_DATA_GET_INT(i,j));
    }
  }
}

/* description: removes the 'proc' filesystem entries ('/proc/pilotdrivers', '/proc/pilotmodules', '/proc/pilotmodule0', '/proc/pilotmodule1', '/proc/pilotmodule2' and '/proc/pilotmodule3') */
static void rpc_proc_deinit(void)
{
  int i, j;

  /* remove /proc/pilot/spiclk */
  remove_proc_entry(pilot_proc_spiclk_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/reset */
  // remove_proc_entry(pilot_proc_reset_name, _internals.proc_pilot_dir);

  /* remove /proc/pilot/firmware */
  // remove_proc_entry(pilot_proc_firmware_name, _internals.proc_pilot_dir);

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

static ssize_t pilot_proc_pilot_stats_write(struct file *file, const char *buf, size_t count, loff_t *off)
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
  cmd.length = MSG_LEN(EEPROM_HID_LENGTH);

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

/* timeout for eeprom requests get uid and get hid */
#define EEPROM_TIMEOUT 1000

/* description: callback function that gets called by the kernel, when /proc/pilot/moduleX/hid is written to */
static ssize_t pilot_proc_pilot_module_hid_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
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

  cmd.length = MSG_LEN(EEPROM_FID_LENGTH);

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

static ssize_t pilot_proc_pilot_module_fid_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
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
static ssize_t pilot_proc_pilot_module_type_write(struct file *file, const char *buf, size_t count, loff_t *off)
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

  cmd.length = MSG_LEN(EEPROM_DATA_LENGTH);

  /* send it to the pilot */
  pilot_send_cmd(&cmd);
}

// **************** END proc file system functions *****************************************

static int pilot_try_get_module_type(int module_index, int timeout, pilot_module_type_t **type)
{
  pilot_cmd_t cmd;
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
  cmd.length = 0; //no payload

  pilot_send_cmd(&cmd);

  /* wait until the pilot_module_type is updated or the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.modules[module_index].type_is_updated_wq, _internals.modules[module_index].type_is_updated != 0, (timeout * HZ / 1000)) <= 0)
  {
    is_timedout = 1;    
    LOG_INFO("pilot_try_get_module_type() timedout while waiting for module_type!");
    *type = NULL;
  }
  else
  {
    *type = module_type;
    LOG_DEBUGALL("pilot_try_get_module_type() successful");
  }

  return is_timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_uid(int module_index, int timeout, pilot_eeprom_uid_t **uid)
{
  pilot_cmd_t cmd;
  int timedout = 0;
  LOG_DEBUG("pilot_try_update_module_uid(module_index = %i, timeout = %i) called", module_index, timeout);

  /* clear the uid_is_updated flag for the module */
  _internals.modules[module_index].uid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_uid_get;
  cmd.length = 0; //no payload

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* wait until the uid is updated or the timeout occurs */
    if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.modules[module_index].uid_is_updated_wq, _internals.modules[module_index].uid_is_updated != 0, (timeout * HZ / 1000)) <= 0)
  {
    timedout = 1;
    LOG_INFO("pilot_try_get_module_uid() timeout reached while waiting for uid!");
  }
  else
  {
    *uid = &_internals.modules[module_index].uid;
    LOG_DEBUGALL("pilot_try_get_module_uid() successful");
  }

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_hid(int module_index, int timeout, pilot_eeprom_hid_t **hid)
{
  pilot_cmd_t cmd;
  int timedout = 0;

  /* clear the hid_is_updated flag for the module */
  _internals.modules[module_index].hid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_hid_get;
  cmd.length = 0; //no payload

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* wait until the uid is updated or the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.modules[module_index].hid_is_updated_wq, _internals.modules[module_index].hid_is_updated != 0, (timeout * HZ / 1000)) <= 0)
  {
    timedout = 1;
    LOG_INFO("pilot_try_get_module_hid() timeout reached while waiting for hid!");
  }
  else
  {
    *hid = &_internals.modules[module_index].hid;    
    LOG_DEBUGALL("pilot_try_get_module_hid() successful");
  }

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_fid(int module_index, int timeout, pilot_eeprom_fid_t **fid)
{
  pilot_cmd_t cmd;
  int timedout = 0;

  /* clear the fid_is_updated flag for the module */
  _internals.modules[module_index].fid_is_updated = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_fid_get;
  cmd.length = 0; //no payload

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* wait until the uid is updated or the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.modules[module_index].fid_is_updated_wq, _internals.modules[module_index].fid_is_updated != 0, (timeout * HZ / 1000)) <= 0)
  {
    timedout = 1;
    LOG_INFO("pilot_try_get_module_fid() timeout reached while waiting for fid!");
  }
  else
  {
    *fid = &_internals.modules[module_index].fid;    
    LOG_DEBUGALL("pilot_try_get_module_fid() successful");
  }

  return timedout ? -1 : SUCCESS;
}

static int pilot_try_get_module_eeprom_data(int module_index, int user_data_index, int timeout, pilot_eeprom_data_t **data)
{
  pilot_cmd_t cmd;
  int timedout = 0;

  /* clear the is_updated flag for the module & data */
  _internals.modules[module_index].user_is_updated[user_data_index] = 0;

  /* setup the cmd */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_t_from_module_slot_and_port(module_index, module_port_1);
  cmd.type = pilot_cmd_type_eeprom_userdata_get;
  cmd.data[pilot_eeprom_userdata_index_number] = (char)user_data_index;
  cmd.length = MSG_LEN(8); //TODO - check size

  /* send the cmd */
  pilot_send_cmd(&cmd);

  /* wait until the data is updated of the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.modules[module_index].user_is_updated_wq[user_data_index], _internals.modules[module_index].user_is_updated[user_data_index] != 0, (timeout * HZ / 1000)) <= 0)
  {
    timedout = 1;
    LOG_INFO("pilot_try_get_module_eeprom_data() timeout reached while waiting for data!");
  }
  else
  {
    *data = &_internals.modules[module_index].user[user_data_index];    
    LOG_DEBUGALL("pilot_try_get_module_eeprom_data() successful");
  }

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

/************************
* /proc/pilot/spiclk
************************/

static int pilot_proc_pilot_spiclk_show(struct seq_file *file, void *data)
{
  seq_printf(file, "%i\n", _internals.spiclk);
  return 0;
}

static ssize_t pilot_proc_pilot_spiclk_write(struct file* file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, ret=-EINVAL;

  /* try to get an int value from the user */
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL; /* return an error if the conversion fails */
  else
  {
    /* sanity check the value before setting the spiclk */
    if (new_value < 10 || new_value > _internals.spi0->max_speed_hz)
      ret = -EINVAL;
    else
    {
      /* store the new spi clk value and reset the spi */
      _internals.spiclk = new_value;
      _internals.spi_xfer.speed_hz = _internals.spiclk;

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

/************************
* /proc/pilot/reset
************************/

/*
static int pilot_proc_pilot_reset_show(struct seq_file *file, void *data)
{
  seq_printf(file, "%i\n", gpio_get_value(_internals.reset_gpio));
  return 0;
}

static int pilot_proc_pilot_reset_write(struct file* file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, ret=-EINVAL;

   // try to get an int value from the user
  if (kstrtoint_from_user(buf, count, 10, &new_value) != SUCCESS)
    ret = -EINVAL; // return an error if the conversion fails
  else
  {
    if (new_value == 0 || new_value == 1)
    {
       gpio_set_value(_internals.reset_gpio, new_value);    
      ret = count;
    }
   }
  return ret;
}

static int pilot_proc_pilot_reset_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_reset_show, NULL);
}

// file operations for /proc/pilot/reset
static const struct file_operations proc_pilot_reset_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_reset_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_reset_write
};
*/

/************************
* /proc/pilot/firmware
************************/

/*

static int pilot_proc_pilot_firmware_show(struct seq_file *file, void *data)
{
  uint8_t aRxBuffer[256];
  uint8_t aTxBuffer[256];
  uint8_t version;
  uint16_t chipID;

  // go into boot mode
  gpio_set_value(_internals.boot_gpio, 1);    
  gpio_set_value(_internals.reset_gpio, 1);
  msleep(100);    
  gpio_set_value(_internals.reset_gpio, 0);
  msleep(100);    

  LOG_DEBUG("BL_Init()");
  BL_Init(_internals.spi0);
  LOG_DEBUG("BL_Get_Command()");
  BL_Get_Command(aRxBuffer);
  LOG_DEBUG("BL_Get_Version()");
  version = BL_GetVersion_Command();
  LOG_DEBUG("BL_GetID()");
  chipID = BL_GetID_Command();

  LOG_DEBUG("Version %x", version);
  LOG_DEBUG("chipID %x", chipID);

  //seq_printf(file, "%i\n", _internals.spiclk);
  return 0;
}

static int pilot_proc_pilot_firmware_write(struct file* file, const char __user *buf, size_t count, loff_t *off)
{
  int new_value, ret=-EINVAL;

  // go into boot mode
  gpio_set_value(_internals.boot_gpio, 1);    
  gpio_set_value(_internals.reset_gpio, 1);
  msleep(100);    
  gpio_set_value(_internals.reset_gpio, 0);

  //start file write


  return ret;
}

static int pilot_proc_pilot_firmware_open(struct inode *inode, struct file *file)
{
  return single_open(file, pilot_proc_pilot_firmware_show, NULL);
}

// file operations for /proc/pilot/firmware
static const struct file_operations proc_pilot_firmware_fops = {
  .owner   = THIS_MODULE,
  .open    = pilot_proc_pilot_firmware_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
  .write   = pilot_proc_pilot_firmware_write
};

*/

/************************
* /proc/pilot/stats
************************/

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

  seq_printf(file, "recv cmds\t\t%u\n", _internals.stats.recv_cmd_count);
  for (i = 0; i < MAX_CMD_TYPE; i++)
    if (_internals.stats.recv_cmd_type_count[i] > 0)
      seq_printf(file, "%s\t\t%i\n", pilot_cmd_type_to_name(i), _internals.stats.recv_cmd_type_count[i]);

  seq_printf(file, "sent cmds\t\t%u\n", _internals.stats.sent_cmd_count);
  for (i = 0; i < MAX_CMD_TYPE; i++)
    if (_internals.stats.sent_cmd_type_count[i] > 0)
      seq_printf(file, "%s\t\t%i\n", pilot_cmd_type_to_name(i), _internals.stats.sent_cmd_type_count[i]);

  seq_printf(file, "cmd CRC checksum errors \t\t%u\n", _internals.stats.crc_errors);
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
  int is_timedout = 0;

  LOG_DEBUG("pilot_try_get_test_result() called");

  /* reset the flag */
  _internals.test_result_is_updated = 0;

  /* send the test run command to the pilot */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_test_run;
  cmd.length = 0; //no payload

  pilot_send_cmd(&cmd);

  /* wait until the test_result is updated or the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.test_result_is_updated_wq, _internals.test_result_is_updated != 0, (200 * HZ / 1000)) <= 0)
  {
    is_timedout = 1;
    LOG_INFO("pilot_try_get_test_result() timedout while waiting for test result!");
  }
  else
  {
     *result = _internals.test_result;   
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
  int is_timedout = 0;

  /* reset the uid_is_updated flag */
  _internals.uid_is_updated = 0;

  /* send the request */
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_eeprom_uid_get;
  cmd.length = 0; //no payload

  pilot_send_cmd(&cmd);

  /* wait until the test_result is updated or the timeout occurs */
  if (WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(_internals.uid_is_updated_wq, _internals.uid_is_updated != 0, (timeout * HZ / 1000)) <= 0)
  {
    is_timedout = 1;
    LOG_INFO("pilot_try_get_uid() timedout while waiting for uid!");
  }
  else
  {
    *uid = _internals.uid_is_updated;    
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

static ssize_t pilot_proc_pilot_module_eeprom_user_write (struct file *file, const char __user *buf, size_t count, loff_t *off)
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
  
  LOG_DEBUGALL("pilot_send(target=%i, count=%i) called", target, count);

  //spin_lock( &QueueLock );

  for (i = 0; i < count; i++)
    if (queue_enqueue( &_internals.TxQueue, ( (int)target | data[i]  << 8) ))
      ret++;
    else
      break;

  //spin_unlock( &QueueLock );

  /* start the spi transmission */
  LOG_DEBUGALL("queue_work() for _internals_irq_data_m2r_work()");

  //tasklet_schedule(&_internals_irq_data_m2r_tasklet);	
  //schedule_work(&_internals_irq_data_m2r_work);
  WAIT_WAKEUP(comm_wait_queue);

  LOG_DEBUGALL("work scheduled successfully");

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
  int i;
  uint8_t length_parity;
  int length = cmd->length << 2;
  
  //cmd->length = MSG_LEN(length);
  length_parity = cmd->length;

  //calculate parity
  length_parity ^= length_parity >> 4;
  length_parity ^= length_parity >> 2;
  length_parity ^= length_parity >> 1;
  cmd->length = cmd->length | ((length_parity & 1) !=0 ? 0 : 0x80);

  cmd->crc = crc((char *) cmd, pilot_current_cmd_index_data_begin + length );

  LOG_DEBUG("pilot_send_cmd: target: %x, type: %x, length: %i (0x%x), crc: %X",(int) cmd->target,(int) cmd->type, length, cmd->length, cmd->crc);

  #ifdef DEBUG
    printk(KERN_CONT "sent:     '");

    for(i=0;i< length;i++)
      printk(KERN_CONT "%x ", cmd->data[i]);

    printk(KERN_CONT "'\n");
  #endif
 //spin_lock( &QueueLock );

  //enqueue first header byte  
  queue_enqueue( &_internals.TxQueue, ( (int)target_base| cmd->target << 8) );
  //enqueue second header byte  
  queue_enqueue( &_internals.TxQueue, ( (int)target_base_type | cmd->type << 8) );
  //enqueue third header byte  
  queue_enqueue( &_internals.TxQueue, ( (int)target_base_length | cmd->length << 8) );
  //enqueue fourth header byte  
  queue_enqueue( &_internals.TxQueue, ( (int)target_base_reserved | cmd->reserved << 8) );

  for (i = 0; i < length; i++) //length *4 bytes
    queue_enqueue( &_internals.TxQueue, ( (int)target_base_data | cmd->data[i]  << 8) );

  for (i = 0; i < sizeof(uint32_t); i++) //length *4 bytes
    queue_enqueue( &_internals.TxQueue, ( (int)target_base_crc | (((uint8_t *)&cmd->crc)[i]) << 8) );

  //spin_unlock( &QueueLock );

  //tasklet_schedule(&_internals_irq_data_m2r_tasklet); 
  //schedule_work(&_internals_irq_data_m2r_work);
  WAIT_WAKEUP(comm_wait_queue);

  /* update the stats */
  _internals.stats.sent_cmd_count++;
  if (cmd->type < MAX_CMD_TYPE)
    _internals.stats.sent_cmd_type_count[(int)cmd->type]++;
}

/* description: gets the number of remaining free bytes of the send buffer for the specified target */
int pilot_get_free_send_buffer_size(target_t target)
{
  //LOG_DEBUG("pilot_get_free_send_buffer_size(target=%i) called", target);
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
