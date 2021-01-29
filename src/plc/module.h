#include <linux/module.h>
#include <linux/wait.h>       /* waitqueue */
#include <linux/kfifo.h>
#include <linux/types.h>
#include "common.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Amesberger");
MODULE_DESCRIPTION("PiloT PLC kernel module");

// opt:
//     bit 7  : 1 - no struct follows, 0 - another struct at the end of this one
//     bit 4-6: value type: 00 - current value, 01 - changed value, 10 - forced value, 11 - config value
//     bit 0-3: length of value
#define GET_VAR_FOLLOWS(opt) (opt & 0x80)
#define GET_VAR_SUB(opt) ((opt >> 4) & 0x7)
#define GET_VAR_LEN(opt) (opt & 0xF)
#define SET_VAR_FOLLOWS(f) (f << 7)
#define SET_VAR_SUB(sub) ((sub & 0x7) << 4)
#define SET_VAR_LEN(len) (len & 0xF)

#define MSG_PLC_VAR_HEADER_LEN 3 //opt + number size

typedef struct __attribute__((__packed__)) {
  uint8_t config;
  uint16_t number;
  uint8_t value;
} msg_plc_var_config_t;

typedef struct __attribute__((__packed__)) {
  uint8_t opt;
  uint16_t number;
  uint8_t value[8];
} msg_plc_var_t;

typedef struct {
  uint16_t number;
  uint8_t subvalue;
  uint8_t length;
  uint8_t value[8];
} plc_var_t;

typedef struct __attribute__((__packed__)) {
  uint8_t cmd;
  uint8_t p1;
  uint16_t p2;
  uint8_t data[8];
} pilotevent_data;

struct pilotevent_state {
	wait_queue_head_t wait;
	DECLARE_KFIFO(events, pilotevent_data, 32);
	struct mutex read_lock;
};

struct mutex access_lock;

typedef struct __attribute__((__packed__)) {
  uint16_t min;
  uint16_t max;
  uint16_t cur;
  uint16_t tick;
  uint16_t comm;
  uint16_t read;
  uint16_t program;
  uint16_t write;
} pilot_plc_cycletimes_t;

#define FIFO_SIZE 16

#define MAX_CSV_COLS 6
#define MAX_VAR_COLS 6
#define MAX_VAR_DATA_LENGTH 8

#define MAX_VAR_DIR_DEPTH 64

enum iectypes {
  IEC_NONE = -1,
  IEC_BOOL, //char
  IEC_SINT, //int8_t
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
  IN,
  OUT,
  MEM,
  VAR,
  EXT
};

typedef struct pilot_plc_vardir {
  struct proc_dir_entry *self;
  struct pilot_plc_vardir *parent;
  struct pilot_plc_vardir **children;
  char* name;
  int childCount;
} pilot_plc_vardir_t;

typedef struct {
  int number;
  enum iecvarclass varclass;
  enum iectypes iectype;
  wait_queue_head_t in_queue;
  wait_queue_head_t out_queue;
  bool is_open;
  char value[MAX_VAR_DATA_LENGTH];
  char *variable;     /* full path as string */
  char *variablename; /* points to the name part of the variable */
  pilot_plc_vardir_t *dir;
  bool subscribed;
  bool forced;
  bool is_variable_updated;
  bool is_poll;
  bool is_force_value_updated;
  bool is_subscribed_updated;
  bool is_forced_updated;
  struct kfifo_rec_ptr_1 fifo;
} pilot_plc_variable_t;


/* struct that groups internal members */
typedef struct {
  volatile bool is_cmd_handler_registered; /* set to 1 if the cmd handler is registered with the rpcp */
  volatile bool is_read_stream_handler_registered; /* set to 1 if the read stream handler is registered with the rpcp */
  volatile bool is_state_updated;
  volatile bool is_info_updated;
  volatile pilot_plc_state_t state;
  volatile bool is_cycletimes_updated;
  volatile pilot_plc_cycletimes_t cycletimes;
  volatile bool is_fwinfo_updated;
  volatile uint8_t fwinfo[pilot_cmd_t_data_size];
  struct proc_dir_entry *proc_pilot_dir;
  struct proc_dir_entry *proc_pilot_plc_dir;
  pilot_plc_vardir_t proc_pilot_plc_vars_dir_root;
  pilot_plc_variable_t **variables;
  int variables_count;
  struct pilotevent_state event_state;
} internals_t;
