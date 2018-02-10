/*
   this file contains defines and functions used for communication between the raspberry pi driver and the pilot prototype implementation
   it is shared between the drivers and stm32f
*/

#ifndef __TTY_COMMANDS_H__
#define __TTY_COMMANDS_H__

/* usart baudrate command values */
typedef enum
{
  pilot_cmd_baudrate_9600,
  pilot_cmd_baudrate_19200,
  pilot_cmd_baudrate_38400,
  pilot_cmd_baudrate_57600,
  pilot_cmd_baudrate_115200,
  pilot_cmd_baudrate_230400,
  pilot_cmd_baudrate_250000
} pilot_cmd_baudrate_t;

/* usart stopbit command values */
typedef enum
{
  pilot_cmd_stopbits_1,
  pilot_cmd_stopbits_2
} pilot_cmd_stopbits_t;

/* usart wordlength command values */
typedef enum
{
  pilot_cmd_wordlength_8,
  pilot_cmd_wordlength_9
} pilot_cmd_wordlength_t;

/* usart parity command values */
typedef enum
{
  pilot_cmd_parity_none,
  pilot_cmd_parity_even,
  pilot_cmd_parity_odd
} pilot_cmd_parity_t;

static pilot_cmd_baudrate_t rpc_stm_proto_int_to_baudrate(int value)
{
  pilot_cmd_baudrate_t baudrate;

  switch (value)
  {
    case 9600: baudrate   = pilot_cmd_baudrate_9600; break;
    case 19200: baudrate  = pilot_cmd_baudrate_19200; break;
    case 38400: baudrate  = pilot_cmd_baudrate_38400; break;
    case 57600: baudrate  = pilot_cmd_baudrate_57600; break;
    case 115200: baudrate = pilot_cmd_baudrate_115200; break;
    case 230400: baudrate = pilot_cmd_baudrate_230400; break;
    case 250000: baudrate = pilot_cmd_baudrate_250000; break;
    default: baudrate     = pilot_cmd_baudrate_9600; break;
  }

  return baudrate;
}

#endif
