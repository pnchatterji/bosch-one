/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <stdio.h>

/*Define application specific firmware to be uploaded*/
#include <bhix60_pdr.h>
#include <firmware/bhi260ap_pdr/BHI260AP_PDR.fw.h>
int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}

void pdr_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value sv_cfg; 

	struct sensor_trigger pdr_trig ={
		.chan = SENSOR_CHAN_BHIX60_PDR,
		.type =	SENSOR_TRIG_DATA_READY,
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);
	bhix60_pdr_reset_full(dev);

	/*Configure PDR virtual sensor for use*/
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_BHIX60_PDR,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/*Configure PDR details*/
	/*Step length and acccuracy is kept artifically small for test purpose,
	so that small physical movements of the test board will trigger data*/
	bhix60_pdr_set_step_info(dev,0.05f,0.01f); 	/*step length, accuracy in m.*/
	bhix60_pdr_set_ref_heading_del(dev,0.0f);   /*heading delta in degrees*/
	bhix60_pdr_set_hand(dev,1);					/*right hand or left hand*/

	/*set callback handler for PDR data*/
	sensor_trigger_set(dev,&pdr_trig,pdr_trigger_handler);
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(10));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif

	}
}

/*PDR data received in FIFO*/
/*NOTE: PDR data arrives only on-change, so no data will be dumped
unless device is physically moved*/
void pdr_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
    uint32_t s, ns;
	uint64_t timestamp;
	/*Access PDR data using PDR extention API*/
	struct bhix60_pdr_data data;
	bhix60_get_pdr_data(dev,&data,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	printf("PDR at %d.%09d  x=%d.%06d y=%d.%06d hac=%d.%06d hd=%d.%06d hdac=%d.%06d scnt=%d fr=%d tr=%d\n",
		s,ns,
		data.x.val1,data.x.val2,
		data.y.val1,data.y.val2,
		data.hor_acc.val1,data.hor_acc.val2,
		data.heading.val1,data.heading.val2,
		data.heading_acc.val1,data.heading_acc.val2,
		data.step_count,
		data.full_reset_status,
		data.track_reset_status);
}
