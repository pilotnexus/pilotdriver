//#include "flash.h"
#include "module.h"
#include "common.h"
#include <asm/delay.h>       // needed for udelay()
#include <linux/wait.h>       /* waitqueue */
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 
#include <linux/mutex.h>   /* mutex */
#include <linux/delay.h>       // needed for msleep()
#include "../driver/export.h"

extern void pilot_send(target_t target, const char* data, int count);
extern internals_t m_internals;
extern struct mutex access_lock;

#define SPI_CONT 1
#define SPI_END 0

int xfer_spi(module_slot_t slot, uint8_t *data, int n, uint8_t cont)
{
	#ifdef DEBUGALL
	int i;
	#endif
	int waitret, ret=0;
	target_t target = target_t_from_module_slot_and_port(slot, module_port_2);
	
	if (mutex_lock_interruptible(&access_lock) != 0)
		return -EINVAL;
	
	m_internals.recv_buf_index = 0;
	mb();

	if (cont == SPI_CONT)
	{
		pilot_send(target, (char *)data, n);
	}
	else if (n > 1)
	{
		pilot_send(target, (char *)data, n-1);
		pilot_send(target | 0x80, (char *)&data[n-1], 1);
	}
	else
	   	pilot_send(target | 0x80, data, 1);

    LOG_DEBUGALL("xfer_spi() target=%i waiting for completed reply", target);
	//TODO, use waitqueue...or similar with timeout

    waitret = wait_event_interruptible_timeout(m_internals.receive_queue, m_internals.recv_buf_index >= n, (100 * HZ / 1000));

	if (waitret == 0 || waitret == -ERESTARTSYS)
	{
		LOG_DEBUG("xfer_spi() timeout");
		ret = -EINVAL;
	}
	else
	{
		//message complete
		LOG_DEBUGALL("xfer_spi() received %i bytes", m_internals.recv_buf_index);
		#ifdef DEBUGALL
		  printk("     data: '");
  	      for(i=0;i < m_internals.recv_buf_index;i++)
		    printk("%x ", m_internals.recv_buf[i]);
		  printk("'\n");
		  #endif

		memcpy (data, m_internals.recv_buf, n);
		m_internals.recv_buf_index = 0;
	}

	mutex_unlock(&access_lock);
	return ret;
}

/*
void send_spi(module_slot_t slot, uint8_t *data, int n)
{
    target_t target = target_t_from_module_slot_and_port(slot, module_port_1);
    pilot_send(target, (char *)data, n);
}
*/

void flash_read_id(module_slot_t slot, char* buffer)
{
	uint8_t cmd = 0x9F;

	LOG_DEBUG("read flash ID (9Fh)");
	memset(buffer, 0, 3); 
	xfer_spi(slot, &cmd, 1, SPI_CONT);
	xfer_spi(slot, buffer, 3, SPI_END);
}

void flash_power_up(module_slot_t slot)
{
	uint8_t data[5] = { 0xAB };
	xfer_spi(slot, data, 5, SPI_END);
	LOG_DEBUG("powered up, ID read: %x", (int)data[4]);
}

void flash_power_down(module_slot_t slot)
{
	uint8_t data[1] = { 0xB9 };
	xfer_spi(slot, data, 1, SPI_END);
}

void flash_write_enable(module_slot_t slot)
{
	uint8_t data[1] = { 0x06 };

	//LOG_DEBUG("write enable (06h)");

	xfer_spi(slot, data, 1, SPI_END);
}

void flash_bulk_erase(module_slot_t slot)
{
	uint8_t data[1] = { 0xc7 };

	LOG_DEBUG("bulk erase (C7h)");

	xfer_spi(slot, data, 1, SPI_END);
}

void flash_64kB_sector_erase(module_slot_t slot, int addr)
{
	uint8_t command[4] = { 0xd8, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };

	LOG_DEBUG("erase 64kB sector at 0x%06X..", addr);

	xfer_spi(slot, command, 4, SPI_END);
}

bool flash_busy(module_slot_t slot)
{
	uint8_t data[2] = { 0x05 };

	if (xfer_spi(slot, data, 2, SPI_END) >= 0)
	{
		if ((data[1] & 0x01) == 0)
			return false;
	}
	return true;
}

bool is_flash_write_enabled_and_not_busy(module_slot_t slot)
{
	uint8_t data[2] = { 0x05 };

	if (xfer_spi(slot, data, 2, SPI_END) >= 0)
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

		xfer_spi(slot, data, 2, SPI_END);

		if ((data[1] & 0x01) == 0)
			break;
		udelay(1000);
	}

	LOG_DEBUG("waiting done.");
}

bool flash_prog(module_slot_t slot, int addr, char *data, int n)
{
  int retryCount = 0;
	uint8_t command[4];
    target_t target = target_t_from_module_slot_and_port(slot, module_port_1);

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
	  while (pilot_get_free_send_buffer_size(slot) < 256) {
	    msleep_interruptible(1);
      if (retryCount++ > 200)
        return false;
    }

	  #ifdef DEBUG
	  printk("ok\n");
	  #endif

	  //calculate address
	  command[0] = 0x02;
	  command[1] = (uint8_t)(addr >> 16);
	  command[2] = (uint8_t)(addr >> 8);
	  command[3] = (uint8_t)addr;

	  //start sending data block (256 bytes max.)
	  xfer_spi(target, command, 4, SPI_END);

	  #ifdef DEBUG
	  printk("writing %i bytes (start address %i)...", n, addr);
	  #endif

    xfer_spi(target, data, n, SPI_END);

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