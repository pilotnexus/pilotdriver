//#include "flash.h"
#include "module.h"
#include "common.h"
#include <asm/delay.h>       // needed for udelay()
#include <linux/wait.h>       /* waitqueue */
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 
#include <linux/mutex.h>   /* mutex */
#include <linux/delay.h>       // needed for msleep()
#include "../driver/export.h"
#include "module.h"

extern void pilot_send(target_t target, const char* data, int count);
extern internals_t m_internals;
extern struct mutex access_lock;

#define SPI_CONT 1
#define SPI_END 0

void flash_read_id(module_slot_t slot, char* buffer)
{
	uint8_t cmd[4] = {0x9F};

	LOG_DEBUG("read flash ID (9Fh)");
	memset(&cmd[1], 0, 3); 
  pilot_fpga_try_send_fpga_cmd(slot, cmd, 4, 500);
  memcpy(buffer, &cmd[1], 3);
}

void flash_power_up(module_slot_t slot)
{
	uint8_t cmd[5] = { 0xAB };
  pilot_fpga_try_send_fpga_cmd(slot, cmd, 5, 500);

	LOG_DEBUG("powered up, ID read: %x", (int)cmd[4]);
}

void flash_power_down(module_slot_t slot)
{
	uint8_t data[1] = { 0xB9 };
  pilot_fpga_try_send_fpga_cmd(slot, data, 1, 500);
}

void flash_write_enable(module_slot_t slot)
{
	uint8_t data[1] = { 0x06 };
  pilot_fpga_try_send_fpga_cmd(slot, data, 1, 500);
}

void flash_bulk_erase(module_slot_t slot)
{
	uint8_t data[1] = { 0xc7 };

	LOG_DEBUG("bulk erase (C7h)");

  pilot_fpga_try_send_fpga_cmd(slot, data, 1, 500);
}

void flash_64kB_sector_erase(module_slot_t slot, int addr)
{
	uint8_t command[4] = { 0xd8, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };

	LOG_DEBUG("erase 64kB sector at 0x%06X..", addr);

  pilot_fpga_try_send_fpga_cmd(slot, command, 4, 500);
}

bool flash_busy(module_slot_t slot)
{
	uint8_t data[2] = { 0x05 };

	if (pilot_fpga_try_send_fpga_cmd(slot, data, 2, 500) == 1)
	{
		if ((data[1] & 0x01) == 0)
			return false;
	}
	return true;
}

bool is_flash_write_enabled_and_not_busy(module_slot_t slot)
{
	uint8_t data[2] = { 0x05 };

	if (pilot_fpga_try_send_fpga_cmd(slot, data, 2, 500) == 1)
	{
		if ((data[1] & 0x03) == 0x2)
			return true;
	}
	return false;
}


void flash_wait(module_slot_t slot)
{
	LOG_DEBUG("waiting...");

	while (1)
	{
		uint8_t data[2] = { 0x05 };

    pilot_fpga_try_send_fpga_cmd(slot, data, 2, 500);

		if ((data[1] & 0x01) == 0)
			break;
		udelay(1000);
	}

	LOG_DEBUG("waiting done.");
}

bool flash_prog(module_slot_t slot, int addr, char *data, int n)
{
  int retryCount = 0;
  uint8_t buffer[270];


	 #ifdef DEBUG
	  printk("write enable...");
	  #endif
	  /* FLASH WRITE ENABLE */
	  flash_write_enable(slot);

	  #ifdef DEBUG
	  printk("wait for enabled...");
	  #endif

    retryCount = 0;
	  while(!is_flash_write_enabled_and_not_busy(slot)) {
	    msleep_interruptible(1);
      if (retryCount++ > 200)
        return false;
    }

	  #ifdef DEBUG
	  printk("wait for send buffer...");
	  #endif

	  /* wait for the internal sendbuffer to become available again */
    retryCount = 0;
	  while (pilot_get_free_send_buffer_size(slot) < 260) {
	    msleep_interruptible(1);
      if (retryCount++ > 200)
        return false;
    }

	  #ifdef DEBUG
	  printk("ok\n");
	  #endif

	  //calculate address
	  buffer[0] = 0x02;
	  buffer[1] = (uint8_t)(addr >> 16);
	  buffer[2] = (uint8_t)(addr >> 8);
	  buffer[3] = (uint8_t)addr;
    memcpy(&buffer[4], data, n);

	  //start sending data block (256 bytes max.)

	  #ifdef DEBUG
	  printk("writing %i bytes (start address %i)...", n, addr);
	  #endif

    pilot_fpga_try_send_fpga_cmd(slot, buffer, n+4, 500);

	  #ifdef DEBUG
	  printk("wait for ready...");
	  #endif

    retryCount = 0;
	  while(flash_busy(slot)) {
	    msleep_interruptible(1);
      if (retryCount++ > 200)
        return false;
    }

	  #ifdef DEBUG
	  printk("ok\n");
	  #endif

    return true;
}

bool flash_read(module_slot_t slot, int addr, char *data, int n)
{
  uint8_t command[4];
  int bytes_written = 0, blocksize, sendbuffersize, waitret;
  target_t target = target_t_from_module_slot_and_port(slot, module_port_1);

  /* VERIFY */
  command[0] = 0x03;
  command[1] = (uint8_t)(addr >> 16);
  command[2] = (uint8_t)(addr >> 8);
  command[3] = (uint8_t)addr;

  pilot_send(target, command, 4);
  m_internals.recv_buf_index = 0;

  target = target_t_from_module_slot_and_port(slot, module_port_2); //switch to receive

    while (bytes_written < n)
    {

      /* wait for the internal sendbuffer to become available again */
      while ((sendbuffersize = pilot_get_free_send_buffer_size(target)) <= 256)
        cpu_relax();

      blocksize = n - bytes_written; /* sent the remaining bytes */
      if (blocksize > sendbuffersize)    /* ...but not more than the remaining sendbuffersize */
        blocksize = sendbuffersize;

      if ((bytes_written + blocksize) == n) //last block
      {
        if (blocksize > 1)
          pilot_send(target, data+bytes_written, blocksize-1); /* send the bytes */
        pilot_send(target | 0x80, data+bytes_written+blocksize-1, 1); /* send the bytes */
      }
      else
      {
        //LOG_DEBUG("bitstream_write() sending %i bytes", blocksize);
        pilot_send(target, data+bytes_written, blocksize); /* send the bytes */
      }

 	waitret = wait_event_interruptible_timeout(m_internals.receive_queue, m_internals.recv_buf_index >= blocksize, (500 * HZ / 1000));

	if (waitret == 0 || waitret == -ERESTARTSYS)
	{
	  LOG_DEBUG("error waiting for verify receive data");
	  return false;
	}

  	memcpy(data+bytes_written, m_internals.recv_buf, blocksize);

      bytes_written += blocksize; /* increment the number of bytes written */
      m_internals.recv_buf_index = 0; //TODO temporary, while module_port_2
    }

    return true;
}