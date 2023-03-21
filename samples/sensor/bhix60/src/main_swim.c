/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <stdio.h>

/*Define application specific firmware to be uploaded*/
#include <bhix60_swim.h>
#include <firmware/bhi260ap_swim/BHI260AP_SWIM.fw.h>
int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}

void swim_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value sv_cfg; 

	struct sensor_trigger swim_trig ={
		.chan =SENSOR_CHAN_BHIX60_SWIM,
		.type=	SENSOR_TRIG_DATA_READY,
	};

	bhy2_swim_config_param_t swimcfg ={
		.update_swim_config = BHY2_SWIM_ENABLE_CONFIG, 		/*SWIM_ENABLE_CONFIG/SWIM_DISABLE_CONFIG */
   		.dev_on_left_hand = BHY2_SWIM_DEVICE_ON_RIGHT_HAND, /*SWIM_DEVICE_ON_LEFT_HAND,SWIM_DEVICE_ON_RIGHT_HAND */
    	.pool_length_integral = BHY2_SWIM_POOL_LENGTH_50M,  /*SWIM_POOL_LENGTH_25M, SWIM_POOL_LENGTH_50M */
    	.pool_length_floating = 0, 							/*Floating point support not yet available for pool length*/
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);
	/*Configure swim virtual sensor for use*/
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_BHIX60_SWIM,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/*Configure swim details*/
	bhix60_swim_set_config(dev,&swimcfg);
	/*set callback handler for swim data*/
	sensor_trigger_set(dev,&swim_trig,swim_trigger_handler);
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

struct bhy2_swim_algo_output data_out;
/*swim data received in FIFO*/
void swim_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	static int count2 =0;
    uint32_t s, ns;
	uint64_t timestamp;
	/*Access swim data using SWIM extention API*/
	bhix60_get_swim_data(dev,&data_out,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	if(count2 ==0)
	{
		/*dump swim data every 1 sec to avoid flooding console*/
		printf("Swim Data at: %d.%09d; %d, %d, %d, %d, %d, %d, %d\n",
		s,
		ns,
		data_out.total_distance,
		data_out.length_count,
		data_out.lengths_freestyle,
		data_out.lengths_breaststroke,
		data_out.lengths_butterfly,
		data_out.lengths_backstroke,
		data_out.stroke_count);
	}
	if(++count2>100)
		count2 =0;
}
