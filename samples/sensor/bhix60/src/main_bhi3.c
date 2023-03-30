/*
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * This BHix60 driver sample demonstrates the special features of BHI360 device
 * that are not covered by the BHI260 samples
 *
 *
 *               ****************************************************
 *               *        TODO: NOT YET IMPLEMENTED!!!              *
 *               ****************************************************
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

/*Define application specific firmware to be uploaded*/
#include <bhix60_bhi3.h>
#include <firmware/bhi360/bhi360.fw.h> 
int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(10));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif
		printf("This sample is not yet implemented!!\n");
	}
}

