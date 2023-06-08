/*
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * Dummy EEPROM driver to simulate EEPROM on boards without EEPROM
 */
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(dummy_eeprom, CONFIG_EEPROM_LOG_LEVEL);
#define DT_DRV_COMPAT dummy_eeprom

struct dummy_eeprom_config {
};

struct dummy_eeprom_data {
    uint32_t size;
	uint8_t eeprom_data[];
};

int dummy_eeprom_read( const struct device *dev, off_t addr, void *buf, size_t len)
{
    struct dummy_eeprom_data *data = dev->data;
    if(addr >= data->size || (addr+len)>= data->size)
    {
        LOG_ERR("Invalid data access addr %d len %d for eeprom size %d",(int)addr,(int)len,(int)data->size);
        return -ENOMEM;        
    };
    memcpy(buf,&data->eeprom_data[addr],len);
    return 0;
}

int dummy_eeprom_write(const struct device *dev, off_t addr, const void *buf, size_t len )
{
    struct dummy_eeprom_data *data = dev->data;
    if(addr >= data->size || (addr+len)>= data->size)
    {
        LOG_ERR("Invalid data access addr %d len %d for eeprom size %d",(int)addr,(int)len,(int)data->size);
        return -ENOMEM;        
    };
    memcpy(&data->eeprom_data[addr],buf,len);
    return 0;
}

size_t dummy_eeprom_size( const struct device *dev)
{
    struct dummy_eeprom_data *data = dev->data;
    return data->size;
}


static const struct eeprom_driver_api dummy_eeprom_driver_api = {
	.read = dummy_eeprom_read,
	.write = dummy_eeprom_write,
	.size = dummy_eeprom_size,
};

static int dummy_eeprom_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define DUMMY_EEPROM_INIT(inst)							\
	static const struct dummy_eeprom_config dummy_eeprom_config_##inst = {	\
	};									\
	static struct dummy_eeprom_data dummy_eeprom_data_##inst = {	\
        .size = DT_PROP_LEN(DT_INST(inst,DT_DRV_COMPAT), eeprom_data), \
		.eeprom_data = DT_PROP(DT_INST(inst,DT_DRV_COMPAT), eeprom_data),				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, &dummy_eeprom_init, NULL,   \
			      &dummy_eeprom_data_##inst,			\
			      &dummy_eeprom_config_##inst,			\
			      POST_KERNEL, CONFIG_EEPROM_INIT_PRIORITY,		\
			      &dummy_eeprom_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DUMMY_EEPROM_INIT)