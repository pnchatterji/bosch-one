/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * This BHIx60 driver sample demonstrates the use of Wakeup type Virtual
 * Sensors in conjunction with the Zephyr Power Management Module.
 * It uses the motion and starionary sensors to detect motion and
 * no motion. If board is physically stationary for longer than 10 seconds
 * the host CPU goes into Soft Off state. It resumes when board is moved again.
 * Accelerometer reading is dumped in active state to simulate normal
 * functioning of the application
 * This sample will only work on a Nordic nrf52 platform (see comments below).
 * This sample is based on the nrf52 sample "system_off"
 * zephyr/samples/board/nrf/system_off
 * When resuming after a Soft Off, the application restarts from the beginning
 * and all data stored in RAM is lost. The above Nordic sample demonstrates a 
 * way to retain critical data on RAM between restarts. 
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <drivers/gpio.h>
#include <hal/nrf_gpio.h>
/*Define application specific firmware to be uploaded*/
#include <bhix60.h>
#include <firmware/bhi260ap/BHI260AP.fw.h> 
int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}
void stationary_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void motion_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);

bool go_to_sleep=false;
int countdown=0;
const int MAX_COUNTDOWN=1000;

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value acc[3];
	struct sensor_value sv_cfg;
	struct sensor_value sv_off = {0,0}; 
	struct sensor_trigger stationary_trig ={
		.chan =  SENSOR_CHAN_STATIONARY,
		.type=SENSOR_TRIG_STATIONARY,
	};
	struct sensor_trigger motion_trig ={
		.chan =SENSOR_CHAN_MOTION,
		.type=	SENSOR_TRIG_MOTION,
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/*Configure the required virtual sensors. Configuration has to be done
	prior to any other call, as this registers the virtual sensor for use*/
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	 sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
	 		SENSOR_ATTR_CONFIGURATION,
	 		&sv_cfg);
	sensor_attr_set(dev,  SENSOR_CHAN_STATIONARY,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_MOTION,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/*set callback hander for stationary detector*/
	sensor_trigger_set(dev,&stationary_trig,stationary_trigger_handler);
	/*set callback handler for motion detector*/
	sensor_trigger_set(dev,&motion_trig,motion_trigger_handler);
	int count =0;

	/* Configure BHI60 INT pin to generate wakeup event*/
	/* The below code is Nordic specific. There is currently no way to do this in a 
	platform-independant manner. pm_device_wakeup_enable() does not work on GPIO pins*/
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_NODELABEL(wup0), gpios),
			   NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_NODELABEL(wup0), gpios),
			       NRF_GPIO_PIN_SENSE_LOW);
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(10));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif
		/*Print accelerometer data to simulate activity*/
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);

		/* although sampling is set at 100 hz, as in a real-world application,
		in this sample we dump values only every 1 sec to avoid flooding the console */
		if(count==0)
		{
			printf("X=%d Y=%d Z=%d\n",acc[0].val1,acc[1].val1,acc[2].val1);
		}
		if(++count>100)
			count =0;
		/*Once stationary event is received, a countdown starts. If a movement does not take place
		during countdown, soft off is triggered. If a movement does take place, revert to normal*/		
		if(go_to_sleep)
			countdown--;
		else
			countdown = MAX_COUNTDOWN;
		if(countdown <=0)
		{
			printk("Entering standby mode\n");
			/*Prior to informing BHIx60 that host processor is about to
			enter standby, disable all unnecessary channels and flush all FIFOs,
			to avoid unexpected triggering of INT pin*/
			sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
	 		SENSOR_ATTR_SAMPLING_FREQUENCY,
	 		&sv_off);
			sensor_attr_set(dev, SENSOR_CHAN_STATIONARY,
	 		SENSOR_ATTR_SAMPLING_FREQUENCY,
	 		&sv_off);
			bhix60_flush_fifo(dev);
			/*Now the BHIx60 can be infirmed that host is entering standby
			Only Wakeup events will trigger INT pin*/
			bhix60_host_standby(dev, true);

			/*reactivate motion sensor, just to be sure of wakeup*/
			sensor_attr_set(dev, SENSOR_CHAN_MOTION,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sv_cfg);
			/* force entry to SOFT OFF when idle thread runs next time
			* NOTE: Nordic platforms only support PM_STATE_SOFT_OFF */
			pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
			/* k_sleep runs the idle thread, and power management subsystem 
			enters soft off as forced above*/
			k_msleep(2000);
			/*should not get here*/
			go_to_sleep=false;		
			printk("Soft off failed\n");
		}
	}
}


/*Triggered when test board has been stationary for a pre-defined period*/
void stationary_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	/*Re-charge Motion sensor to ensure it fires on next motion*/
	struct sensor_value sv_cfg; 
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_MOTION,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sv_cfg);
	printf("Board is stationary!\n");
	go_to_sleep=true;
}

/*Triggered when test board is physically moved*/
void motion_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	/*Re-charge Stationary sensor to ensure it fires when motion stops*/
	struct sensor_value sv_cfg; 
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev,  SENSOR_CHAN_STATIONARY,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sv_cfg);
	printf("Board has moved!\n");
	go_to_sleep=false;
}


