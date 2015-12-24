/* this file defines commands used by the rpi<->pilot communication */

#ifndef __RPC_STM_H__
#define __RPC_STM_H__

#include "common_base.h"

/* macros for manipulating spidata_t */
#define spi_header_t_get_target_t(header) (target_t)header

#endif
