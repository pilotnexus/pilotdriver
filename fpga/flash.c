//#include "flash.h"
#include "module.h"
#include "common.h"
#include <asm/delay.h>       // needed for udelay()
#include <linux/wait.h>       /* waitqueue */
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 
#include <linux/mutex.h>   /* mutex */

extern void pilot_send(target_t target, const char* data, int count);
extern internals_t m_internals;
extern struct mutex access_lock;

#define SPI_CONT 1
#define SPI_END 0

int xfer_spi(module_slot_t slot, uint8_t *data, int n, uint8_t cont)
{
	int waitret, ret=0;
	target_t target = target_t_from_module_slot_and_port(slot, module_port_2);
	
	if (mutex_lock_interruptible(&access_lock) != 0)
		return -EINVAL;
	
	m_internals.recv_buf_index = 0;

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

    LOG_DEBUG("xfer_spi() target=%i waiting for completed reply", target);
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
		LOG_DEBUG("xfer_spi() received %i bytes", m_internals.recv_buf_index);
		memcpy (data, m_internals.recv_buf, n);
		m_internals.recv_buf_index = 0;
	}

	mutex_unlock(&access_lock);
	return ret;
}

void send_spi(module_slot_t slot, uint8_t *data, int n)
{
    target_t target = target_t_from_module_slot_and_port(slot, module_port_1);
    pilot_send(target, (char *)data, n);
}

void flash_read_id(module_slot_t slot, char* buffer)
{
	uint8_t cmd = 0x9F;

	LOG_DEBUG("read flash ID (9Fh)");

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

	LOG_DEBUG("write enable..");

	xfer_spi(slot, data, 1, SPI_END);
}

void flash_bulk_erase(module_slot_t slot)
{
	uint8_t data[1] = { 0xc7 };

	LOG_DEBUG("bulk erase..");

	xfer_spi(slot, data, 1, SPI_END);
}

void flash_64kB_sector_erase(module_slot_t slot, int addr)
{
	uint8_t command[4] = { 0xd8, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };

	LOG_DEBUG("erase 64kB sector at 0x%06X..", addr);

	send_spi(slot, command, 4);
}

void flash_prog(module_slot_t slot, int addr, uint8_t *data, int n)
{
	uint8_t command[4] = { 0x02, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };

	LOG_DEBUG("prog 0x%06X +0x%03X..", addr, n);

	send_spi(slot, command, 4);
	send_spi(slot, data, n);

	//if (verbose)
	//	for (int i = 0; i < n; i++)
	//		fprintf(stderr, "%02x%c", data[i], i == n-1 || i % 32 == 31 ? '\n' : ' ');
}

void flash_read(module_slot_t slot, int addr, uint8_t *data, int n)
{
	uint8_t command[4] = { 0x03, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };

	LOG_DEBUG("read 0x%06X +0x%03X..", addr, n);

	send_spi(slot, command, 4);
	memset(data, 0, n);
	xfer_spi(slot, data, n, SPI_END);

	//if (verbose)
	//	for (int i = 0; i < n; i++)
	//		fprintf(stderr, "%02x%c", data[i], i == n-1 || i % 32 == 31 ? '\n' : ' ');
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