/*
    this file contains defines that are share between the rpi drivers and the microcontroller
*/
#ifndef __COMMON_BASE_H__
#define __COMMON_BASE_H__

#include <linux/kernel.h>    // needed u16

/* the number of modules that are available in the pilot */
#define MODULES_COUNT 4

/* the number of ports for each module */
#define MODULE_PORT_COUNT 2

/* command message payload calc macro */
#define MSG_LEN(x) (0x7F & ((x) >> 2))

/* we're using 16 bit SPI communication */
typedef u16 spidata_t;

/* stm commands */
typedef enum
{
  pilot_cmd_type_invalid = 0,
  pilot_cmd_type_eeprom_uid_get,             /* 0x01 rpi <-> pilot request to send the eeprom uid of the specified module */
  pilot_cmd_type_eeprom_hid_get,             /* 0x02 rpi <-> pilot request to send the eeprom hid of the specified module */
  pilot_cmd_type_eeprom_hid_set,             /* 0x03 rpi <-> pilot command to set the eeprom hid of the specified module */
  pilot_cmd_type_eeprom_fid_get,             /* 0x04 rpi <-> pilot request to get the eeprom fid of the specified module */
  pilot_cmd_type_eeprom_fid_set,             /* 0x05 rpi -> pilot command to set the eeprom fid of the specified module */
  pilot_cmd_type_usart_set_baudrate,         /* 0x06 rpi -> pilot command to set the baudrate of the specified usart */
  pilot_cmd_type_usart_set_stopbits,         /* 0x07 rpi -> pilot command to set the stopbits of the specified usart */
  pilot_cmd_type_usart_set_wordlength,       /* 0x08 rpi -> pilot command to set the wordlength of the specified usart */
  pilot_cmd_type_usart_set_parity,           /* 0x09 rpi -> pilot command to set the parity of the specified usart */
  pilot_cmd_type_usart_send_break,           /* 0x0A rpi -> pilot command to send a break on the specified usart */
  pilot_cmd_type_input_get_input,            /* 0x0B rpi <-> pilot request to send back the state of the input */
  pilot_cmd_type_rtc_get,                    /* 0x0C rpi <-> pilot messages to request and send the datetime of the rtc */
  pilot_cmd_type_rtc_set,                    /* 0x0D rpi -> pilot command to set the rtc */
  pilot_cmd_type_bufferstate,                /* 0x0E rpi <- pilot message to inform the rpi of the current bufferstate of the usart */
  pilot_cmd_type_input_get_counter,          /* 0x0F rpi <-> pilot request to send the value of the counter */
  pilot_cmd_type_input_set_counter,          /* 0x10 rpi -> pilot command to set the value of the counter */
  pilot_cmd_type_gps_set_enable,             /* 0x11 rpi -> pilot command to enable / disable the gps */
  pilot_cmd_type_gps_get_enable,             /* 0x12 rpi <-> pilot request to send the enabled state of the gps module */
  pilot_cmd_type_io16_set_direction,         /* 0x13 rpi -> pilot command to set direction of io block */
  pilot_cmd_type_output_set_value,           /* 0x14 rpi -> pilot command to set output value */
  pilot_cmd_type_input_changed,              /* 0x15 rpi <- pilot command that informs the rpi that an input changed */
  pilot_cmd_type_onewire_set_enable,         /* 0x16 rpi -> pilot command to enable / disable one-wire */
  pilot_cmd_type_onewire_get_enable,         /* 0x17 rpi <-> pilot request to send the enabled state of the one-wire module */
  pilot_cmd_type_gsm_set_enable,             /* 0x18 rpi <-> pilot command to enable / disable the gsm */
  pilot_cmd_type_gsm_get_enable,             /* 0x19 rpi <-> pilot request to send the enabled state of the gsm module */
  pilot_cmd_type_module_type_get,            /* 0x1A rpi <-> pilot request to send the module type that the firmware was built for */
  pilot_cmd_type_slcd_udpate,                /* 0x1B rpi -> pilot command that signals the start of an update stream */
  pilot_cmd_type_slcd_set_resolution,        /* 0x1C rpi -> pilot command to set the display resolution */
  pilot_cmd_type_slcd_get_resolution,        /* 0x1D rpi <-> pilot request to send the display resolution */
  pilot_cmd_type_eeprom_userdata_get,        /* 0x1E rpi <-> pilot request to send custom user data from the eeprom */
  pilot_cmd_type_eeprom_userdata_set,        /* 0x1F rpi -> pilot request to write custom user data to the eeprom */
  pilot_cmd_type_plc_state_get,              /* 0x20 rpi <-> pilot request to send the state of the plc */
  pilot_cmd_type_plc_state_set,              /* 0x21 rpi -> pilot command to set the state of the plc */
  pilot_cmd_type_plc_cycletimes_get,         /* 0x22 rpi <-> pilot request to send the plc cycle times */
  pilot_cmd_type_plc_read_var_config,        /* 0x23 rpi -> pilot request to get plc config item */
  pilot_cmd_type_plc_write_var_config,       /* 0x24 rpi -> pilot command to write plc config item  */
  pilot_cmd_type_plc_variable_get,           /* 0x25 rpi <-> pilot request to send plc variables */
  pilot_cmd_type_plc_variable_set,           /* 0x26 rpi -> pilot command to set plc variables */
  pilot_cmd_type_lora_set_enable,            /* 0x27 rpi -> pilot command to enable / disable the lora module */
  pilot_cmd_type_lora_get_enable,            /* 0x28 rpi <-> pilot request to send the enabled state of the lora module */
  pilot_cmd_type_test_run,                   /* 0x29 rpi <-> pilot request to run internal tests */
  pilot_cmd_type_reserved1,                  /* 0x2A rpi <-> pilot command to get single variable */
  pilot_cmd_type_reserved2,                  /* 0x2B rpi -> pilot command to write single */
  pilot_cmd_type_fpga_state,                 /* 0x2C rpi <-> fpga state */
  pilot_cmd_type_fpga_cmd,                   /* 0x2D rpi <-> fpga cmd */
  pilot_cmd_type_comm_stat,                  /* 0x2E rpi <-> stats */
  pilot_cmd_type_fwinfo                      /* 0x2F rpi <-> pilot command get fw info */
} pilot_cmd_type_t;

/* enum that specifies the target of the stream communication
   target_t is a combination of module_slot_t and module_port_t */
typedef enum
{
  target_invalid       = 0x00, /* there is no valid target, the following data should be discarded */

  target_module1_port1 = 0x01, /* the target is the first port of the Module in the slot 1  */
  target_module1_port2 = 0x02, /* the target is the second port of the Module in the slot 1 */

  target_module2_port1 = 0x03, /* the target is the first port of the Module in the slot 2  */
  target_module2_port2 = 0x04, /* the target is the second port of the Module in the slot 2 */

  target_module3_port1 = 0x05, /* the target is the first port of the Module in the slot 3  */
  target_module3_port2 = 0x06, /* the target is the second port of the Module in the slot 3 */

  target_module4_port1 = 0x07, /* the target is the first port of the Module in the slot 4  */
  target_module4_port2 = 0x08, /* the target is the second port of the Module in the slot 4 */

  target_plc_read      = 0x60, /* the target is the soft plc read */
  target_plc_write     = 0x61, /* the target is the soft plc write */

  target_base          = 0x70,
  target_base_type     = 0x71,
  target_base_length   = 0x72,
  target_base_reserved = 0x73,
  target_base_crc      = 0x74,
  target_base_data     = 0x75
} target_t;

#define pilot_cmd_t_data_size 256
#define pilot_cmd_t_size_without_data 8

/* rpi command struct */
typedef struct {
  unsigned char target; /* target of the command */
  unsigned char type;   /* type of the command */
  unsigned char length; /* command length */
  unsigned char reserved; /* reserved byte */
  char data[pilot_cmd_t_data_size]; /* command data, value depends on type */
  uint32_t crc;
} pilot_cmd_t;

typedef enum {
  pilot_current_cmd_index_target     = 0,
  pilot_current_cmd_index_type       = 1,
  pilot_current_cmd_index_length     = 2,
  pilot_current_cmd_index_data_begin = 4
} pilot_current_cmd_index_t;

/* struct that holds the current rpi command that is being received */
typedef struct {
  pilot_cmd_t cmd;                 /* the command we're receiving from the rpi */
  pilot_current_cmd_index_t index; /* current index of the fillstatus of this command */
  uint32_t length;
  uint8_t cmd_completion;
} pilot_current_cmd_t;

/* specifies the slot of the module */
typedef enum
{
  module_slot_1 = 0,
  module_slot_2 = 1,
  module_slot_3 = 2,
  module_slot_4 = 3
} module_slot_t;

/* specifies the port, every module has 2 possible ports */
typedef enum
{
  module_port_1 = 0,
  module_port_2 = 1
} module_port_t;

/* macros for creating target_t and converting module_slot_t and module_port_t to target_t */

#define target_t_get_module_slot(target) ((module_slot_t)((target-1)/2))
#define target_t_get_module_port(target) ((module_port_t)((target-1)%2))
#define target_t_from_module_slot_and_port(module,port) ((target_t)(((int)module)*2+1+(int)port))

#define eeprom_encode_module_slot_and_data_index(module_slot, data_index) ((module_slot << 4)|(data_index))
#define eeprom_decode_module_slot(data) (data >> 4)
#define eeprom_decode_data_index(data) (data & 0xF)

typedef enum
{
  stm_bufferstate_not_full,
  stm_bufferstate_full
} stm_bufferstate_t;

//#define spidata_t_get_bufferstatus(data, slot, port) ((data >> (2*slot+port)) & 1)

/* 
    hours....0-23
    minutes..0-59
    seconds..0-59
    weekday........1-7
    day of month...1-31
    month..........1-12
    year...........0-99  (2000-2099)
*/
typedef enum {
  pilot_rtc_index_hours      = 0,
  pilot_rtc_index_minutes    = 1,
  pilot_rtc_index_seconds    = 2,
  pilot_rtc_index_weekday    = 3,
  pilot_rtc_index_dayofmonth = 4,
  pilot_rtc_index_month      = 5,
  pilot_rtc_index_year       = 6
} pilot_rtc_index_t;

/* specifies the targetted counter, used as first data byte in pilot_cmd_type_input_get_counter */
typedef enum {
  pilot_counter_target_1   = 0,
  pilot_counter_target_2   = 1,
  pilot_counter_target_3   = 2,
  pilot_counter_target_4   = 3,
  pilot_counter_target_5   = 4,
  pilot_counter_target_6   = 5,
  pilot_counter_target_7   = 6,
  pilot_counter_target_8   = 7,
  pilot_counter_target_all = 0xff
} pilot_counter_target_t;

/* helper enum that for pilot_cmd_type_input_counter_X handling */
typedef enum {
  pilot_counter_index_target = 0, /* encode the counters index in the first byte */
  pilot_counter_index_value  = 1  /* encode the counter value in the last 7 bytes */
} pilot_counter_index_t;

/* specifies the targetted input, used as first data byte in pilot_cmd_type_input_get_input */
typedef enum {
  pilot_input_target_1 = 0,
  pilot_input_target_2 = 1,
  pilot_input_target_3 = 2,
  pilot_input_target_4 = 3,
  pilot_input_target_5 = 4,
  pilot_input_target_6 = 5,
  pilot_input_target_7 = 6,
  pilot_input_target_8 = 7
} pilot_input_target_t;

/* helper enum for pilot_cmd_type_input_get_input handling */
typedef enum {
  pilot_input_index_target = 0, /* encode the input index in the first byte */
  pilot_input_index_value  = 2  /* encode the input value in the last 2 bytes */
} pilot_input_index_t;

typedef enum {
  pilot_lora_enable_index_value = 0 /* encode the enable value (0=disable, 1=enable) in the last byte */
} pilot_lora_enable_index_t;

/* helper enum for pilot_cmd_type_gps_set_enable / pilot_cmd_type_gps_get_enable handling */
typedef enum {
  pilot_gps_enable_index_value = 0 /* encode the enable value (0=disable, 1=enable) in the last byte */
} pilot_gps_enable_index_t;

typedef enum {
  pilot_gsm_enable_index_value = 0 /* encode the enable value (0=disable, 1=enable) in the last byte */
} pilot_gsm_enable_index_t;

typedef enum {
  pilot_onewire_enable_index_value = 0 /* encode the enable value (0=disable, 1=enable) in the last byte */
} pilot_onewire_enable_index_t;

typedef enum {
  pilot_io16_block_0_to_3,
  pilot_io16_block_4_to_7,
  pilot_io16_block_8_to_11,
  pilot_io16_block_12_to_15
} pilot_io16_block_t;

typedef enum
{
  pilot_io16_direction_input,
  pilot_io16_direction_output
} pilot_io16_direction_t;

typedef enum
{
  pilot_io16_set_direction_index_block = 0,
  pilot_io16_set_direction_index_direction = 2
} pilot_io16_set_direction_index_t;

typedef enum {
  pilot_output_target_1  = 0,
  pilot_output_target_2  = 1,
  pilot_output_target_3  = 2,
  pilot_output_target_4  = 3,
  pilot_output_target_5  = 4,
  pilot_output_target_6  = 5,
  pilot_output_target_7  = 6,
  pilot_output_target_8  = 7,
  pilot_output_target_9  = 8,
  pilot_output_target_10 = 9,
  pilot_output_target_11 = 10,
  pilot_output_target_12 = 11,
  pilot_output_target_13 = 12,
  pilot_output_target_14 = 13,
  pilot_output_target_15 = 14,
  pilot_output_target_16 = 15
} pilot_output_target_t;

typedef enum
{
  pilot_output_index_target = 0, /* encode the output target in the first byte */
  pilot_output_index_value = 4,  /* encode the output value in the last four bytes */
} pilot_output_index_t;

typedef enum
{
  pilot_bufferstate_index_value = 0 /* encode the bufferstate value in the last byte */
} pilot_bufferstate_index_t;

typedef enum
{
  pilot_slcd_resolution_index_width = 0, /* encode the width in the first 4 bytes */
  pilot_slcd_resolution_index_height = 4 /* encode the height in the last 4 bytes */
} pilot_slcd_resolution_index_t;

typedef enum
{
  pilot_eeprom_userdata_index_number = 0 /* encode the eeprom user index in the last byte */
} pilot_eeprom_userdata_index_t;

typedef enum
{
  pilot_plc_state_stop = 0,
  pilot_plc_state_run = 1
} pilot_plc_state_t;

typedef enum
{
  pilot_plc_state_index = 0 /* encode the plc state in the last byte */
} pilot_plc_state_index_t;

typedef enum
{
  pilot_plc_cycletimes_index_min = 0, /* encode the min time in the first 2 bytes (0,1) */
  pilot_plc_cycletimes_index_max = 2, /* encode the max time in the bytes 2,3 */
  pilot_plc_cycletimes_index_cur = 4, /* encode the current time in bytes 4,5 */
  pilot_plc_cycletimes_index_tick = 6,  /* encode the tick in the bytes 6,7 */
  pilot_plc_cycletimes_index_comm = 8, /* encode the comm time in the bytes 8,9 */
  pilot_plc_cycletimes_index_read = 10, /* encode the read time in the bytes 10,11 */
  pilot_plc_cycletimes_index_program = 12, /* encode the program time in bytes 12,13*/
  pilot_plc_cycletimes_index_write = 14  /* encode the write time in the bytes 14,15 */
} pilot_plc_cycletimes_index_t;

typedef enum
{
  pilot_plc_variables_value_index_size = 4 /* encode the size of the returned variables in bytes 5, 6, 7, 8 */
} pilot_plc_variables_value_index_t;

typedef enum
{
  pilot_test_run_index_failed_index = 1,
  pilot_test_run_index_failed_low_count = 2,
  pilot_test_run_index_failed_high_count = 3,
  pilot_test_run_index_failed_count = 4,
  pilot_test_run_index_success_count = 5,
  pilot_test_run_index_total_count = 6,
  pilot_test_run_index_result = 7 /* encode the success of the test run reply in the last base */
} pilot_test_run_index_t;

typedef enum
{
  pilot_test_run_result_success = 0,       /* no error occurred */
  pilot_test_run_result_not_supported = 1, /* the image does not support test runs */
  pilot_test_run_result_failed = 2         /* the test run failed */
} pilot_test_run_result_t;

#define INT_FROM_BYTES(b) ((b[0] << 8*3) | (b[1] << 8*2) | (b[2] << 8*1) | (b[3] << 8*0))
#define BYTE_FROM_INT(i, c) ((i >> (3-c)*8) & 0xFF)

#define UINT16_FROM_BYTES(b) ((uint16_t)((b[0] << 8) | (b[1])))
#define BYTE_FROM_UINT16(i, c) ((i >> (1-c)*8) & 0xFF)

#define VAR_TO_UINT16(n, f) ((uint16_t) n | (f ? 0x8000 : 0x0) )
#define VAR_GET_NUMBER(v) ((uint16_t) v & ~0x8000)
#define VAR_IS_FORCED(v) (v & 0x8000)

#define EEPROM_UID_LENGTH 8 //needs to be multiple of 4!

/* unique factory programmed 64-bit eeprom id */
typedef struct {
  uint8_t uid[EEPROM_UID_LENGTH];
} pilot_eeprom_uid_t;

#define EEPROM_DATA_LENGTH 8 //needs to be multiple of 4!
typedef struct {
  uint8_t data[EEPROM_DATA_LENGTH];
} pilot_eeprom_data_t;

typedef pilot_eeprom_data_t pilot_eeprom_hid_t;
#define EEPROM_HID_LENGTH EEPROM_DATA_LENGTH

typedef pilot_eeprom_data_t pilot_eeprom_fid_t;
#define EEPROM_FID_LENGTH EEPROM_DATA_LENGTH

#define MODULE_TYPE_LENGTH 8 //needs to be multiple of 4!

#define MODULE_FWINFO_LENGTH 44
/* type of a module */
typedef struct {
  uint8_t name[MODULE_TYPE_LENGTH];
} pilot_module_type_t;

#define EEPROM_USER_DATA_COUNT 12

#endif
