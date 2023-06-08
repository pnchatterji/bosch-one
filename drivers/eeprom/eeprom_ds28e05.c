/*
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    eeprom_ds28e05.c
 * @date    31 May, 2023
 * @brief   Zephyr driver for DS28e05 EEPROM via 1-wire bus.
 * LIMITATION: This driver assumes a single slave is connected to 1-wire bus
 * as it uses the SKIP ROM command, which permits communication with a single slave
 * without having to know the slave ID. This driver will have to be modified if
 * multiple slaves are to be supported.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/kernel.h>
#include "eeprom_ds28e05.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(ds28e05, CONFIG_EEPROM_LOG_LEVEL);

#define DT_DRV_COMPAT maxim_ds28e05

struct ds28e05_eeprom_config {
	const struct device *bus;
};

int ds28e05_eeprom_read(const struct device *dev, off_t offset, void *data_, size_t len)
{
	const struct ds28e05_eeprom_config *cfg = dev->config;
	const struct device *w1_dev = cfg->bus;
	uint8_t address = (uint8_t) offset;
	uint8_t *data = (uint8_t *)data_;
    uint8_t i;
    if (data == NULL)
	{
		return -EINVAL;
	}
	if (w1_reset_bus(w1_dev) == 0)
	{
		return -EIO;
	}
	/* Skip Rom command: to avoid sending the slave device ID. This assumes a single 
	device is connected to 1-wire bus*/
	(void)w1_write_byte(w1_dev,DS28E05_SKIP_ROM_CMD);

	(void)w1_write_byte(w1_dev,DS28E05_READ_MEMORY_CMD);

	(void)w1_write_byte(w1_dev,address);
	(void)w1_write_byte(w1_dev,0x00);

	for (i = 0; i < len; i++)
	{
		data[i] = w1_read_byte(w1_dev);
	}

	return 0;
}

int ds28e05_eeprom_write(const struct device *dev, off_t offset, const void *data_, size_t len)
{
	const struct ds28e05_eeprom_config *cfg = dev->config;
	const struct device *w1_dev = cfg->bus;

	uint8_t address = (uint8_t) offset;
	uint8_t *data = (uint8_t *)data_;
	if(data == NULL)
	{
		return -EINVAL;
	}
	uint8_t databyte1=0, databyte2=0;
	uint8_t sf, ef;
	uint8_t is_command_success;
	uint8_t sb[2] = { 0 };
	uint8_t eb[2] = { 0 };
	uint8_t spage, epage, npage, wpage;
	uint8_t nseg, wseg = 0;
	uint8_t wBytes = 0, rBytes = 0, wAddr = 0;

	/* Calculate pages */
	spage = (address & DS28E05_PAGE_MASK) >> 4;
	epage = ((address + len) & DS28E05_PAGE_MASK) >> 4;
	if (epage == DS28E05_NUM_PAGES)
	{
		epage = DS28E05_NUM_PAGES - 1;
	}

	npage = epage - spage;

	/* This memory must be written respecting 16bits boundaries */
	sf = (address & 0x01) != 0;
	ef = ((address + len) & 0x01) != 0;

	if (ef)
	{
		ds28e05_eeprom_read(dev, address + len, &eb[1], 1);
		eb[0] = data[len - 1];
		len++;
	}

	if (sf)
	{
		ds28e05_eeprom_read(dev, address - 1, &sb[0], 1);
		sb[1] = data[0];
		len++;
		address--;
	}

	/* Write pages */
	for (wpage = 0; wpage <= npage; wpage++)
	{
		wAddr = address + wBytes;

		/* Calculate segments to write */
		if ((len - wBytes) > (DS28E05_BYTES_PER_PAGE))
		{
			/* Will we pass a page boundary */
			if (wAddr % (DS28E05_SEG_SIZE * DS28E05_BYTES_PER_SEG) == 0)
			{
				nseg = DS28E05_SEG_SIZE;
			}
			else
			{
				nseg = (DS28E05_BYTES_PER_PAGE - (wAddr % (DS28E05_BYTES_PER_PAGE))) >> 1;
			}
		}
		else
		{
			nseg = ((len - wBytes) & DS28E05_SEG_MASK) >> 1;
		}

		if (w1_reset_bus(w1_dev) == 0)
		{
			return -EIO;
		}

		/* Skip Rom command: to avoid sending the slave device ID. This assumes a single 
		device is connected to 1-wire bus*/
		(void)w1_write_byte(w1_dev,DS28E05_SKIP_ROM_CMD);

		(void)w1_write_byte(w1_dev,DS28E05_WRITE_MEMORY_CMD);
		(void)w1_write_byte(w1_dev,wAddr);
		(void)w1_write_byte(w1_dev,0xff);

		/* Write segments within page */
		for (wseg = 0; wseg < nseg; wseg++)
		{
			if (sf)
			{
				(void)w1_write_byte(w1_dev,sb[0]);
				(void)w1_write_byte(w1_dev,sb[1]);
				wBytes += 2;
				rBytes++;
				sf = 0;
			}
			else if (ef && (len - wBytes) <= 2)
			{
				(void)w1_write_byte(w1_dev,eb[0]);
				(void)w1_write_byte(w1_dev,eb[1]);
				wBytes += 2;
				rBytes++;
				ef = 0;
			}
			else
			{
				(void)w1_write_byte(w1_dev,data[rBytes]);
				(void)w1_write_byte(w1_dev,data[rBytes + 1]);
				wBytes += 2;
				rBytes += 2;
			}

			databyte1 = w1_read_byte(w1_dev);
			databyte2 = w1_read_byte(w1_dev);

			(void)w1_write_byte(w1_dev,0xff);

			k_msleep(DS28E05_tPROG);

			is_command_success = w1_read_byte(w1_dev);

			if (is_command_success != DS28E05_COMMAND_SUCCESS)
			{
				(void)w1_reset_bus(w1_dev);

				return -EIO;
			}
		}

		(void)w1_reset_bus(w1_dev);
	}

	(void)databyte1;
	(void)databyte2;

	return 0;
}

size_t ds28e05_eeprom_size(const struct device *dev)
{
	return (size_t)(DS28E05_BYTES_PER_PAGE * DS28E05_NUM_PAGES);
}

static const struct eeprom_driver_api ds28e05_eeprom_driver_api = {
	.read = ds28e05_eeprom_read,
	.write = ds28e05_eeprom_write,
	.size = ds28e05_eeprom_size,
};

static int ds28e05_eeprom_init(const struct device *dev)
{
	const struct ds28e05_eeprom_config *cfg = dev->config;
	if (device_is_ready(cfg->bus) == 0) {
		LOG_DBG("w1 bus is not ready");
		return -ENODEV;
	}
	return 0;
}


#define ds28e05_EEPROM_INIT(inst)							\
	static const struct ds28e05_eeprom_config ds28e05_eeprom_config_##inst = { \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),		       \
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, &ds28e05_eeprom_init, NULL, NULL,		\
			      &ds28e05_eeprom_config_##inst,			\
			      POST_KERNEL, CONFIG_EEPROM_INIT_PRIORITY,		\
			      &ds28e05_eeprom_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ds28e05_EEPROM_INIT)
