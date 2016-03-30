/* this file contains common defines, used by all kernel modules */
/* ATTENTION: define MODULE_NAME before including this file */
/*
    defines the following constants:

    SUCCESS

    LOG() function 
    uses printk to log the supplied string. Prefixed it with MODULE_NAME

    LOG_DEBUG() function
    As Log(), but gets stripped when DEBUG is undefined.

*/

#ifndef __COMMON_H__
#define __COMMON_H__

#include "common_base.h"
#include "debug.h"

/* #define DEBUG */
#define SUCCESS 0

/* define DEBUG in debug.h to include debug messages */

/* use LOG_DEBUG() to trace messages in DEBUG mode */
#undef LOG_DEBUG
#ifdef DEBUG
#define LOG_DEBUG(fmt, args...) do { printk( KERN_DEBUG MODULE_NAME ": " fmt "\n", ## args); } while(0)
#else 
#define LOG_DEBUG(fmt, args...) do {} while(0);
#endif

/* use LOG() instead of printk() to prefix "rpc: " to traced messages */
#define LOG(LEVEL, fmt, args...) do { printk( LEVEL MODULE_NAME ": " fmt "\n", ## args); } while(0)
#define LOG_INFO(fmt, args...) do { printk( KERN_INFO MODULE_NAME ": " fmt "\n", ## args); } while(0)

#endif
