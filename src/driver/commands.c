#include "commands.h"
#define pilot_CMD_TYPE_UNKNOWN "unknown"
#define pilot_CMD_TYPE_INVALID "invalid"
#define pilot_CMD_TYPE_UID_GET "eeprom get uid"
#define pilot_CMD_TYPE_HID_GET "eeprom get hid"
#define pilot_CMD_TYPE_HID_SET "eeprom set hid"
#define pilot_CMD_TYPE_USART_SET_BAUDRATE "usart set baudrate"
#define pilot_CMD_TYPE_USART_SET_STOPBITS "usart set stopbits"
#define pilot_CMD_TYPE_USART_SET_WORDLENGTH "usart set wordlength"
#define pilot_CMD_TYPE_USART_SET_PARITY "usart set parity"
#define pilot_CMD_TYPE_USART_SEND_BREAK "usart send break"
#define pilot_CMD_TYPE_BUFFERSTATE "bufferstate"
#define pilot_CMD_TYPE_INPUT_GET_INPUT "input get input"
#define pilot_CMD_TYPE_RTC_GET "rtc get"
#define pilot_CMD_TYPE_RTC_SET "rtc set"
#define pilot_CMD_TYPE_INPUT_GET_COUNTER "input get counter"
#define pilot_CMD_TYPE_INPUT_SET_COUNTER "input set counter"
#define pilot_CMD_TYPE_GPS_SET_ENABLE "gps set enable"
#define pilot_CMD_TYPE_GPS_GET_ENABLE "gps get enable"
#define pilot_CMD_TYPE_IO16_SET_DIRECTION "io16 set direction"
#define pilot_CMD_TYPE_OUTPUT_SET_VALUE "output set value"
#define pilot_CMD_TYPE_INPUT_CHANGED "input changed"
#define pilot_CMD_TYPE_ONEWIRE_SET_ENABLE "onewire set enable"
#define pilot_CMD_TYPE_ONEWIRE_GET_ENABLE "onewire get enable"
#define pilot_CMD_TYPE_GSM_SET_ENABLE "gsm set enable"
#define pilot_CMD_TYPE_GSM_GET_ENABLE "gsm get enable"
#define pilot_CMD_TYPE_MODULE_TYPE_GET "module type get"
#define pilot_CMD_TYPE_FID_GET "eeprom get fid"
#define pilot_CMD_TYPE_FID_SET "eeprom set fid"
#define pilot_CMD_TYPE_SLCD_UPDATE "slcd update"
#define pilot_CMD_TYPE_SLCD_SET_RESOLUTION "slcd set resolution"
#define pilot_CMD_TYPE_SLCD_GET_RESOLUTION "slcd get resolution"
#define pilot_CMD_TYPE_EEPROM_USERDATA_GET "eeprom get userdata"
#define pilot_CMD_TYPE_EEPROM_USERDATA_SET "eeprom set userdata"
#define pilot_CMD_TYPE_PLC_STATE_GET "plc get state"
#define pilot_CMD_TYPE_PLC_STATE_SET "plc set state"
#define pilot_CMD_TYPE_PLC_CYCLETIMES_GET "plc get cycletimes"
#define pilot_CMD_TYPE_PLC_VARIABLES_READ_CONFIG "plc variables read config"
#define pilot_CMD_TYPE_PLC_VARIABLES_WRITE_CONFIG "plc variables write config"
#define pilot_CMD_TYPE_PLC_VARIABLES_GET "plc variables get"
#define pilot_CMD_TYPE_PLC_VARIABLES_SET "plc variables set"
#define pilot_CMD_TYPE_TEST_RUN "test run"
#define pilot_CMD_TYPE_PLC_VARIABLE_GET "plc var get"
#define pilot_CMD_TYPE_PLC_VARIABLE_SET "plc var set"
#define pilot_CMD_TYPE_LORA_SET_ENABLE "LoRA set enable"
#define pilot_CMD_TYPE_LORA_GET_ENABLE "LoRA get enable"
#define pilot_CMD_TYPE_TEST_RUN "test run"
#define pilot_CMD_TYPE_MODULE_STATUS_GET "Module status get"  
#define pilot_CMD_TYPE_MODULE_STATUS_SET "Module status set"  
#define pilot_CMD_TYPE_FPGA_STATE "fpga state"
#define pilot_CMD_TYPE_FPGA_CMD "fpga cmd" 
#define pilot_CMD_TYPE_COMM_STAT "comm stat"
#define pilot_CMD_TYPE_FWINFO "fw info" 
#define pilot_CMD_TYPE_UART_MODE_GET "UART mode get"
#define pilot_CMD_TYPE_UART_MODE_SET "UART mode set"

char* pilot_cmd_type_to_name(pilot_cmd_type_t type)
{
  switch (type)
  {
    case pilot_cmd_type_invalid: return pilot_CMD_TYPE_INVALID;
    case pilot_cmd_type_eeprom_uid_get: return pilot_CMD_TYPE_UID_GET;
    case pilot_cmd_type_eeprom_hid_get: return pilot_CMD_TYPE_HID_GET;
    case pilot_cmd_type_eeprom_hid_set: return pilot_CMD_TYPE_HID_SET;
    case pilot_cmd_type_eeprom_fid_get: return pilot_CMD_TYPE_FID_GET;
    case pilot_cmd_type_eeprom_fid_set: return pilot_CMD_TYPE_FID_SET;
    case pilot_cmd_type_gps_get_enable: return pilot_CMD_TYPE_GPS_GET_ENABLE;
    case pilot_cmd_type_gps_set_enable: return pilot_CMD_TYPE_GPS_SET_ENABLE;
    case pilot_cmd_type_usart_set_baudrate: return pilot_CMD_TYPE_USART_SET_BAUDRATE;
    case pilot_cmd_type_usart_set_stopbits: return pilot_CMD_TYPE_USART_SET_STOPBITS;
    case pilot_cmd_type_usart_set_wordlength: return pilot_CMD_TYPE_USART_SET_WORDLENGTH;
    case pilot_cmd_type_usart_set_parity: return pilot_CMD_TYPE_USART_SET_PARITY;
    case pilot_cmd_type_usart_send_break: return pilot_CMD_TYPE_USART_SEND_BREAK;
    case pilot_cmd_type_input_get_input: return pilot_CMD_TYPE_INPUT_GET_INPUT;
    case pilot_cmd_type_rtc_get: return pilot_CMD_TYPE_RTC_GET;
    case pilot_cmd_type_rtc_set: return pilot_CMD_TYPE_RTC_SET;
    case pilot_cmd_type_bufferstate: return pilot_CMD_TYPE_BUFFERSTATE;
    case pilot_cmd_type_input_get_counter: return pilot_CMD_TYPE_INPUT_GET_COUNTER;
    case pilot_cmd_type_input_set_counter: return pilot_CMD_TYPE_INPUT_SET_COUNTER;
    case pilot_cmd_type_io16_set_direction: return pilot_CMD_TYPE_IO16_SET_DIRECTION;
    case pilot_cmd_type_output_set_value: return pilot_CMD_TYPE_OUTPUT_SET_VALUE;
    case pilot_cmd_type_input_changed: return pilot_CMD_TYPE_INPUT_CHANGED;
    case pilot_cmd_type_onewire_set_enable: return pilot_CMD_TYPE_ONEWIRE_SET_ENABLE;
    case pilot_cmd_type_onewire_get_enable: return pilot_CMD_TYPE_ONEWIRE_GET_ENABLE;
    case pilot_cmd_type_gsm_set_enable: return pilot_CMD_TYPE_GSM_SET_ENABLE;
    case pilot_cmd_type_gsm_get_enable: return pilot_CMD_TYPE_GSM_GET_ENABLE;
    case pilot_cmd_type_module_type_get: return pilot_CMD_TYPE_MODULE_TYPE_GET;
    case pilot_cmd_type_slcd_udpate: return pilot_CMD_TYPE_SLCD_UPDATE;
    case pilot_cmd_type_slcd_set_resolution: return pilot_CMD_TYPE_SLCD_SET_RESOLUTION;
    case pilot_cmd_type_slcd_get_resolution: return pilot_CMD_TYPE_SLCD_GET_RESOLUTION;
    case pilot_cmd_type_eeprom_userdata_get: return pilot_CMD_TYPE_EEPROM_USERDATA_GET;
    case pilot_cmd_type_eeprom_userdata_set: return pilot_CMD_TYPE_EEPROM_USERDATA_SET;
    case pilot_cmd_type_plc_state_get: return pilot_CMD_TYPE_PLC_STATE_GET;
    case pilot_cmd_type_plc_state_set: return pilot_CMD_TYPE_PLC_STATE_SET;
    case pilot_cmd_type_plc_cycletimes_get: return pilot_CMD_TYPE_PLC_CYCLETIMES_GET;
    case pilot_cmd_type_plc_read_var_config: return pilot_CMD_TYPE_PLC_VARIABLES_READ_CONFIG;
    case pilot_cmd_type_plc_write_var_config: return pilot_CMD_TYPE_PLC_VARIABLES_WRITE_CONFIG;
    case pilot_cmd_type_plc_variable_get: return pilot_CMD_TYPE_PLC_VARIABLES_GET;
    case pilot_cmd_type_plc_variable_set: return pilot_CMD_TYPE_PLC_VARIABLES_SET;
    case pilot_cmd_type_lora_set_enable: return pilot_CMD_TYPE_LORA_SET_ENABLE;
    case pilot_cmd_type_lora_get_enable: return pilot_CMD_TYPE_LORA_GET_ENABLE;
    case pilot_cmd_type_test_run: return pilot_CMD_TYPE_TEST_RUN;
    case pilot_cmd_type_module_status_get: return pilot_CMD_TYPE_MODULE_STATUS_GET; 
    case pilot_cmd_type_module_status_set:  return pilot_CMD_TYPE_MODULE_STATUS_SET; 
    case pilot_cmd_type_fpga_state: return pilot_CMD_TYPE_FPGA_STATE;
    case pilot_cmd_type_fpga_cmd: return pilot_CMD_TYPE_FPGA_CMD;
    case pilot_cmd_type_comm_stats_get: return pilot_CMD_TYPE_COMM_STAT;
    case pilot_cmd_type_fwinfo: return pilot_CMD_TYPE_FWINFO;
    case pilot_cmd_type_uart_mode_get: return pilot_CMD_TYPE_UART_MODE_GET;
    case pilot_cmd_type_uart_mode_set: return pilot_CMD_TYPE_UART_MODE_SET;
    default: return pilot_CMD_TYPE_UNKNOWN;
  }
}
