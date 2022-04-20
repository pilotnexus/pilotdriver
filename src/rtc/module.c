#include <linux/kernel.h>          /* needed for KERN_INFO */
#include "module.h"                /* include defines that describe the module */
#include "../driver/export.h"      /* needed for rpcp main driver functions */
#include "common.h"                
#include <linux/rtc.h>             /* for struct rtc_time, strut rtc_class_ops */
#include <linux/bcd.h>             /* for bcd2bin() macro */
#include <linux/platform_device.h> /* for struct platform_driver */
#include <linux/sched.h>
#include <linux/string.h>          /* for memset() */

// *******************************************************************
// START forward declaration
static int  __init pilot_rtc_init(void); /* kernel module entry function */
static void __exit pilot_rtc_exit(void); /* kernel module exit function */

/* forward declaration of rtc-kernel function */
static int pilot_rtc_get_time(struct device* dev, struct rtc_time *t);
static int pilot_rtc_set_time(struct device* dev, struct rtc_time *t);

/* forward declaration of raspicomm callback functions */
static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd);

/* sends a rtc_get request to the stm */
static void pilot_rtc_request_datetime(void);

// END forward declaration
// *******************************************************************

// *******************************************************************
// START local members

/* struct that groups internal members */
typedef struct {
  int is_cmd_handler_registered; /* set to 1 if the cmd handler is registered with the rpcp */
  struct rtc_time time;          /* the current time received from rtc */
  volatile int is_time_updated;           /* if the rtc time was already updated from the stm */
} internals_t;

/* internal variables */
static internals_t _internals = {
  .is_cmd_handler_registered = 0
};

/* rtc struct with callback functions for the kernel */
static const struct rtc_class_ops pilot_rtc_ops = {
  .read_time        = pilot_rtc_get_time,
  .set_time         = pilot_rtc_set_time
};

/* rpcp struct with callback functions for the main rpcp driver */
static pilot_cmd_handler_t pilot_cmd_handler = {
  .callback_cmd_received = pilot_callback_cmd_received
};


// END local members
// *******************************************************************

/* main entry point */
module_init(pilot_rtc_init);

/* main exit point */
module_exit(pilot_rtc_exit);

static void pilot_rtc_device_release(struct device *dev)
{
  LOG_DEBUG("pilot_rtc_device_release() called");
}

/* probe is called by the kernel when a device is detected whose name matches the name of the driver */
static int pilot_rtc_probe(struct platform_device *dev)
{
  int ret;
  struct rtc_device *rtc;

  LOG_DEBUG("pilot_rtc_probe() called");

  /* create the rtc device */
  //rtc = rtc_device_register("pilotrtc", &dev->dev, &pilot_rtc_ops, THIS_MODULE);
  rtc = devm_rtc_device_register(&dev->dev, "pilotrtc", &pilot_rtc_ops, THIS_MODULE);

  if (IS_ERR(rtc)) {
    ret = PTR_ERR(rtc);
  }
  else {
    /* store the rtc device inside the driver */
    platform_set_drvdata(dev, rtc);
    ret = SUCCESS;
  }

  return ret;
}

static int pilot_rtc_remove(struct platform_device *dev)
{
  struct rtc_device* rtc;

  LOG_DEBUG("pilot_rtc_remove() called");

  /* retrieve the associated rtc device */
  rtc = platform_get_drvdata(dev);

  /* remove the rtc device */
  //devm_rtc_device_unregister(rtc);

  return SUCCESS;
}

/* platform driver */
static struct platform_driver pilot_rtc_driver = {
    .driver = {
      .name = "pilotrtc", /* using the same name as the platform device */
      .owner = THIS_MODULE
    },
    .probe  = pilot_rtc_probe,
    .remove = pilot_rtc_remove
};

/* platform device */
static struct platform_device pilot_rtc_device = {
  .name = "pilotrtc", /* using the same name as the platform driver */
  .id = 0,
  .dev = {
    .release = pilot_rtc_device_release
  }
};

/* initialization routine, called when the module is loaded */
static int __init pilot_rtc_init()
{
  int ret = -1;

  LOG_DEBUG("pilot_rtc_init()");

  /* register with the base driver */
  if (pilot_register_cmd_handler(&pilot_cmd_handler) == SUCCESS) {

    LOG_DEBUG("pilot_register_cmd_handler() succeeded");
    _internals.is_cmd_handler_registered = 1;

    /* register the device */
    platform_device_register(&pilot_rtc_device);

    /* register the driver */
    platform_driver_register(&pilot_rtc_driver);

    /* request the datetime from the pilot */
    //pilot_rtc_request_datetime();

    ret = SUCCESS;
  }
  else
  {
    LOG(KERN_ERR, "pilot_register_cmd_handler() failed!");
  }

  return ret;
}

/* rtc module cleanup function, called when removing the module */
static void __exit pilot_rtc_exit()
{
  LOG_DEBUG("pilot_rtc_exit() called");

  /* unregister the driver */
  platform_driver_unregister(&pilot_rtc_driver);

  /* unregister the device */
  platform_device_unregister(&pilot_rtc_device);

  /* unregister with the base driver */
  if (_internals.is_cmd_handler_registered) {
    if (pilot_unregister_cmd_handler(&pilot_cmd_handler) == SUCCESS)
      _internals.is_cmd_handler_registered = 0;
  }
}

/* sends a rtc_get request to the stm */
static void pilot_rtc_request_datetime()
{
  pilot_cmd_t cmd;
  memset(&cmd, 0, sizeof(pilot_cmd_t));
  cmd.target = target_base;
  cmd.type = pilot_cmd_type_rtc_get;
  pilot_send_cmd(&cmd);
}

/* converter function that updates the supplied time struct from the pilot_cmd_t */
static void pilot_rtc_update_time_from_cmd(struct rtc_time *time, const pilot_cmd_t* cmd)
{
  /* update the time */
  time->tm_sec   = cmd->data[pilot_rtc_index_seconds];
  time->tm_min   = cmd->data[pilot_rtc_index_minutes];
  time->tm_hour  = cmd->data[pilot_rtc_index_hours];
  time->tm_mday  = cmd->data[pilot_rtc_index_dayofmonth];
  time->tm_mon   = cmd->data[pilot_rtc_index_month];
  time->tm_wday  = cmd->data[pilot_rtc_index_weekday];
  time->tm_year  = cmd->data[pilot_rtc_index_year];

  LOG_DEBUG("pilot_rtc_update_time_from_cmd() called: hours=%d, minutes=%d, seconds=%d, dayofmonth=%d, month=%d, weekday=%d, year=%d",
            cmd->data[pilot_rtc_index_hours], cmd->data[pilot_rtc_index_minutes], cmd->data[pilot_rtc_index_seconds],
            cmd->data[pilot_rtc_index_dayofmonth], cmd->data[pilot_rtc_index_month], cmd->data[pilot_rtc_index_weekday], cmd->data[pilot_rtc_index_year]);
}

static void pilot_rtc_update_cmd_from_time(pilot_cmd_t* cmd, const struct rtc_time* time)
{
  /* update the cmd */
  cmd->data[pilot_rtc_index_seconds]    = time->tm_sec;
  cmd->data[pilot_rtc_index_minutes]    = time->tm_min;
  cmd->data[pilot_rtc_index_hours]      = time->tm_hour;
  cmd->data[pilot_rtc_index_dayofmonth] = time->tm_mday;
  cmd->data[pilot_rtc_index_month]      = time->tm_mon;
  cmd->data[pilot_rtc_index_weekday]    = time->tm_wday;
  cmd->data[pilot_rtc_index_year]       = time->tm_year;
}

// *******************************************************************
// START rtc interface function implementation
static int pilot_rtc_get_time(struct device* dev, struct rtc_time *t)
{
  unsigned long timestamp;
  int timedout = 0;

  LOG_DEBUG("pilot_rtc_get_time() called");

  /* reset the is_time_updated flag */
  _internals.is_time_updated = 0;

  /* request the time from the stm */
  pilot_rtc_request_datetime();

  /* wait until the datetime request is fulfilled or until the timeout is hit */
  timestamp = jiffies + (HZ / 10); /* 100ms in the future */

  LOG_DEBUG("pilot_rtc_get_time() waits for the rtc_get request to be fulfilled");
  while(!_internals.is_time_updated)
  {
    if (time_after(jiffies, timestamp)) {
      timedout = 1;
      break;
    }
  }

  if (timedout)
  {
    LOG_DEBUG("pilot_rtc_get_time() timeout reached while waiting for rtc_get request");
  }
  else
  {
    /* fill the time with the last time we received from the pilot */
    t->tm_sec   = _internals.time.tm_sec;
    t->tm_min   = _internals.time.tm_min;
    t->tm_hour  = _internals.time.tm_hour;
    t->tm_mday  = _internals.time.tm_mday;
    t->tm_wday  = _internals.time.tm_wday;
    t->tm_mon   = _internals.time.tm_mon;
    t->tm_year  = _internals.time.tm_year;
    t->tm_isdst = _internals.time.tm_isdst;

    LOG_DEBUG("seconds: %d, minutes: %d, hours: %d, dayofmonth: %d, dayofweek: %d, month: %d, year: %d",
               t->tm_sec, t->tm_min, t->tm_hour, t->tm_mday, t->tm_wday, t->tm_mon, t->tm_year);

  }

  LOG_DEBUGALL("pilot_rtc_get_time() exiting");

  return rtc_valid_tm(t);
}

/* returns: 0 on success, otherwise negative */
static int pilot_rtc_set_time(struct device* dev, struct rtc_time *t)
{
  pilot_cmd_t cmd;

  LOG_DEBUG("pilot_rtc_set_time() called");

  memset(&cmd, 0, sizeof(pilot_cmd_t));    /* start with zeroed out cmd */
  cmd.type = pilot_cmd_type_rtc_set;       /* set command type to rtc_set */
  pilot_rtc_update_cmd_from_time(&cmd, t); /* update the cmd with the supplied time */
  pilot_send_cmd(&cmd);

  return SUCCESS;
}
// END rtc interface function implementation
// *******************************************************************

// *******************************************************************
// START pilot interface function implementation

static pilot_cmd_handler_status_t pilot_callback_cmd_received(pilot_cmd_t cmd)
{
  pilot_cmd_handler_status_t ret;
  struct rtc_time* t = &_internals.time;
  LOG_DEBUGALL("pilot_callback_cmd_received() called");

  switch (cmd.type)
  {
    /* we're receiving the answer to the rtc_get command */
    case pilot_cmd_type_rtc_get:
      LOG_DEBUG("pilot_callback_cmd_received() receiving rtc_get answer");

      /* update the cached time */
      pilot_rtc_update_time_from_cmd(t, &cmd);
      mb();
      _internals.is_time_updated = 1;

      /* mark the command as handled */
      ret = pilot_cmd_handler_status_handled;
      break;

    default: ret = pilot_cmd_handler_status_ignored;  break;
  }

  return ret;
}


// END pilot interface function implementation
// *******************************************************************
