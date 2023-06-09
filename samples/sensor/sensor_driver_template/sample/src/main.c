/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

/*TIP: Modify sample code to demonstrate use of BXX driver. The below
example code is a small extract from the BI270 sample. Modify as needed
*/
void dr_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bxx_dev));
	struct sensor_value value_XYX[3];
	struct sensor_value full_scale; 
	struct sensor_trigger dr_trig ={
		.chan =SENSOR_CHAN_ACCEL_XYZ,
		.type=	SENSOR_TRIG_DATA_READY,
	};
	if (dev == NULL) {
		printf("Device not found\n");
		return;
	}
	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);
	/* Setting scale in G, due to loss of precision if the SI unit m/s^2 is used*/
	full_scale.val1 = 2;            /* G */
	full_scale.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	/*set callback hander for data ready*/
	sensor_trigger_set(dev,&dr_trig,dr_trigger_handler);
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		//k_sleep(K_MSEC(10));
		k_sleep(K_MSEC(1000));

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, value_XYX);

		printf("Value XYZ X: %d.%06d; Y: %d.%06d; Z: %d.%06d;\n",
			value_XYX[0].val1, value_XYX[0].val2,
			value_XYX[1].val1, value_XYX[1].val2,
			value_XYX[2].val1, value_XYX[2].val2);
	}
}

/*callback in case of data ready interrupt from device*/

/*TIP: The below function will not get called if the bare template is
compiled and run, because the device-specific interrupt configuration
code is not implemented in the template (see the TIP inside bxx_init_interrupt()
in bxx.c ). The device interrupt configuration for the specific BXX device 
has to be properly implemented to see this feature working.*/

void dr_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	/*TIP: Do something meaningful here*/
	printf("Data Ready!\n");
}

