/*
  this file contains defines used for the encoding / decoding of IO specific messages
  it will be used by the io driver and the microcontroller
*/

#ifndef __IO_STM_H__
#define __IO_STM_H__

/*    
   7 6 5 4 3 2 1 0 
         | | | | \_ 
         \_\_\_|   `-{ state, 1-bit (0=low, 1=high) }         
               \_
                 `-{ gpio pin, 4-bit }
*/


typedef uint8_t io_data_t;

/* macro functions to manipulate io_data_t */

/* retrieves the io */
#define io_data_t_getio(io_data) io_data >> 1

/* retrieves the state of the gpio 0..low, 1..high */
#define io_data_t_getstate(io_data) io_data & 1

/* sets the state of the gpio to the specified value */
#define io_data_t_setstate(io_data,state) io_data |= (state ? 1 : 0)

/* sets the gpio number to the specified value */
#define io_data_t_setio(io_data,gpio) io_data |= (gpio << 1)

#endif
