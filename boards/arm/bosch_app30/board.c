/*
 * Copyright (c) 2023 Bosch Sensortec GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/init.h>

int16_t set_shuttle_vdd_vddio(uint16_t vdd_millivolt, uint16_t vddio_millivolt);

/*configuation flag to select 1.8V or 2.8 V for shuttle board VDD*/
#if CONFIG_BOSCH_SHUTTLE_VDD_MV > 1800
	#define PWR_CTRL_VDD_SEL_1_8 0
#else
	#define PWR_CTRL_VDD_SEL_1_8 1
#endif

static int board_bosch_app30_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* The discrete power management system of the Application Board 3.0 allows 
	 * to configure the voltage supplied to the sensor with the help of GPIOs.
	 * The GPIOs need to be configured before the initialization of the sensor 
	 * drivers during the kernel boot, so that the sensors are powered on 
	 * during the sensor initialization.
	 */

	int ret=0;
	struct gpio_dt_spec ios_vdd_sel = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_sel),gpios);
	ret |= gpio_pin_configure_dt(&ios_vdd_sel, (PWR_CTRL_VDD_SEL_1_8?GPIO_OUTPUT_INACTIVE:GPIO_OUTPUT_ACTIVE)); 
	struct gpio_dt_spec ios_vdd_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_en),gpios);
	ret |= gpio_pin_configure_dt(&ios_vdd_en, GPIO_OUTPUT_ACTIVE); 
	struct gpio_dt_spec ios_vddio_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vddio_en),gpios);
	ret |= gpio_pin_configure_dt(&ios_vddio_en, GPIO_OUTPUT_ACTIVE);
	return ret; 
}

int16_t set_shuttle_vdd_vddio(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
	struct gpio_dt_spec ios_vdd_sel = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_sel),gpios);
	struct gpio_dt_spec ios_vdd_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_en),gpios);
	struct gpio_dt_spec ios_vddio_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vddio_en),gpios);
    if (vdd_millivolt == 0)
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_INACTIVE);
    }
    else if ((vdd_millivolt > 0) && (vdd_millivolt <= 1800))
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_INACTIVE);
    }
    else
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_ACTIVE);
    }

    if (vddio_millivolt == 0)
        gpio_pin_set_dt(&ios_vddio_en,GPIO_OUTPUT_INACTIVE);
    else
        gpio_pin_set_dt(&ios_vddio_en,GPIO_OUTPUT_ACTIVE);
    return 0;
}

#if defined(CONFIG_USB_DEVICE_STACK) && defined(CONFIG_BOSCH_USB_AUTO_START)
static int usb_cdc_init(const struct device *dev)
{
	int err = usb_enable(NULL);

	return err;
}

SYS_INIT(usb_cdc_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /*CONFIG_BOSCH_USB_AUTO_START*/

SYS_INIT(board_bosch_app30_init, PRE_KERNEL_2, //PRE_KERNEL_1,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
