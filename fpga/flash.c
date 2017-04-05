//#include "flash.h"
#include "module.h"
#include "common.h"
#include <asm/delay.h>       // needed for udelay()

extern void pilot_send(target_t target, const char* data, int count);
extern internals_t m_internals;

void xfer_spi(module_slot_t slot, uint8_t *data, int n)
{
    target_t target = target_t_from_module_slot_and_port(slot, module_port_2);
	m_internals.recv_buf_index = 0;

    if (n > 1)
    {
    	pilot_send(target, (char *)data, n-1);
    	pilot_send(target & 0x80, (char *)&data[n-1], 1);
	}
	else
	   	pilot_send(target & 0x80, (char *)data, 1);

}

void send_spi(module_slot_t slot, uint8_t *data, int n)
{
    target_t target = target_t_from_module_slot_and_port(slot, module_port_1);
    pilot_send(target, (char *)data, n);
}

void flash_read_id(module_slot_t slot)
{
	uint8_t data[4] = { 0x9F };

	int i;

	LOG_DEBUG("read flash ID...");

	xfer_spi(slot, data, 4);

	LOG_DEBUG("flash ID:");
	for (i = 1; i < 4; i++)
		LOG_DEBUG(" 0x%02X", data[i]);
}

void flash_power_up(module_slot_t slot)
{
	uint8_t data[1] = { 0xAB };
	xfer_spi(slot, data, 1);
}

void flash_power_down(module_slot_t slot)
{
	uint8_t data[1] = { 0xB9 };
	xfer_spi(slot, data, 1);
}

void flash_write_enable(module_slot_t slot)
{
	uint8_t data[1] = { 0x06 };

	LOG_DEBUG("write enable..");

	xfer_spi(slot, data, 1);
}

void flash_bulk_erase(module_slot_t slot)
{
	uint8_t data[1] = { 0xc7 };

	LOG_DEBUG("bulk erase..");

	xfer_spi(slot, data, 1);
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
	xfer_spi(slot, data, n);

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

		xfer_spi(slot, data, 2);

		if ((data[1] & 0x01) == 0)
			break;
		udelay(1000);
	}

	LOG_DEBUG("waiting done.");
}