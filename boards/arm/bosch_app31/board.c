/*
 * Copyright (c) 2023 Bosch Sensortec GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/init.h>
#include <drivers/bq25120.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(board_app31, CONFIG_SENSOR_LOG_LEVEL);

/*configuation to select 1.8V or 2.8 V for shuttle board VDD
is done in the PMIC device config in the DTS in case of APP3.1
CONFIG_BOSCH_SHUTTLE_VDD_MV is ignored in APP31*/
int16_t set_shuttle_vdd_vddio(uint16_t vdd_millivolt, uint16_t vddio_millivolt);
static void reset_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_INF("pmic reset");
}

static void int_status_cb(const struct device *dev, uint16_t int_status)
{
	LOG_INF("pmic status %04x",int_status);
	if(BQ25120_INT_STATUS_WAKE2(int_status))
	{
		bq25120_set_op_mode_ship(dev);
	}
}

static int board_bosch_app31_init(const struct device *board_dev)
{
	ARG_UNUSED(board_dev);

	int ret=0;
	struct gpio_dt_spec ios_vdd_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_en),gpios);
	ret |= gpio_pin_configure_dt(&ios_vdd_en, GPIO_OUTPUT_ACTIVE); 
	struct gpio_dt_spec ios_vddio_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vddio_en),gpios);
	ret |= gpio_pin_configure_dt(&ios_vddio_en, GPIO_OUTPUT_ACTIVE);
	const struct device *pmic_dev = DEVICE_DT_GET(DT_NODELABEL(pmic));
	if (!device_is_ready(pmic_dev)) {
		LOG_ERR("PMIC device %s is not ready\n", pmic_dev->name);
		return -EIO;
	}
	bq25120_reg_reset_cb(pmic_dev,reset_cb);
	bq25120_reg_status_cb(pmic_dev,int_status_cb);
	return ret; 
}
int16_t set_shuttle_vdd_vddio(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
	int rc =0;
	struct gpio_dt_spec ios_vdd_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_en),gpios);
	struct gpio_dt_spec ios_vddio_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vddio_en),gpios);
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(pmic));
	if (!device_is_ready(dev)) {
		LOG_ERR("PMIC device %s is not ready\n", dev->name);
		return -EIO;
	}
	rc |= bq25120_set_sys_vout(dev,vddio_millivolt);
	rc |= bq25120_set_ls_ldo(dev,vdd_millivolt);
	rc |= gpio_pin_configure_dt(&ios_vdd_en, (vdd_millivolt < 800)?
				GPIO_OUTPUT_INACTIVE:GPIO_OUTPUT_ACTIVE); 
	rc |= gpio_pin_configure_dt(&ios_vddio_en, (vddio_millivolt < 1800)?
				GPIO_OUTPUT_INACTIVE:GPIO_OUTPUT_ACTIVE);
    return rc;
}
#if defined(CONFIG_USB_DEVICE_STACK) && defined(CONFIG_BOSCH_USB_AUTO_START)
static int usb_cdc_init(const struct device *dev)
{
	int err = usb_enable(NULL);

	return err;
}

SYS_INIT(usb_cdc_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /*CONFIG_BOSCH_USB_AUTO_START*/
/*unlike APP3.0, in APP3.1 the init can be done only at APPLICATION time (just before main()
as PMIC driver needs to be up)*/
SYS_INIT(board_bosch_app31_init, APPLICATION, //PRE_KERNEL_2, 
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
