#include <linux/module.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mdk/Daniel Amesberger");
MODULE_DESCRIPTION("PiloT PLC kernel module");

/* for set_plc_variable */
#define PLC_VAR_FORCE_BIT     0x8000
#define PLC_VAR_UNFORCE_BIT   0x4000

/*for get_plc_variable*/
#define PLC_VAR_SUBSCRIBE_BIT     0x8000
#define PLC_VAR_UNSUBSCRIBE_BIT   0x4000