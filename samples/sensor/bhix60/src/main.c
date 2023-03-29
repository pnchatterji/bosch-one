/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * This BHix60 driver sample demonstrates the use of virtual sensors that use
 * the onboard hardware sensors, i.e. the ones that do not require additional 
 * slave sensors.
 * Specifically: Accelerometer, Gyroscope,Tilt Detector, Game Rotation Vector
 * It demonstrates accessing data using the Sensor API as well as the extended API
 * (for Game Rotation Vector) as well as the setting of scales and 
 * data-ready triggers, threshold triggers and event triggers 
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

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
void tilt_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void rotation_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void gyro_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void accel_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void sensor_print(char *name, struct sensor_value *v, char *unit);

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value acc[3], gyr[3];
	struct sensor_value full_scale, sv_cfg; 
	struct sensor_trigger tilt_trig ={
		.chan =  SENSOR_CHAN_TILT,
		.type=SENSOR_TRIG_BHIX60_EVENT,
	};
	struct sensor_trigger rotation_trig ={
		.chan =SENSOR_CHAN_GAME_ROTATION_QTR,
		.type=	SENSOR_TRIG_DATA_READY,
	};
	struct sensor_trigger gyro_trig ={
		.chan = SENSOR_CHAN_GYRO_XYZ,
		.type =	SENSOR_TRIG_THRESHOLD,
	};
	struct sensor_trigger accel_trig ={
		.chan = SENSOR_CHAN_ACCEL_XYZ,
		.type =	SENSOR_TRIG_THRESHOLD,
	};
	/*Set threshold for gyro triggers to 4 rad/s*/
	struct sensor_value gyro_thresh = {4,0};
	/*Set threshold for accel triggers to 10 m/s^2*/
	struct sensor_value accel_thresh = {10,0};

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
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev,  SENSOR_CHAN_TILT,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_GAME_ROTATION_QTR,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/* Setting Accl scale in m/s^2 */
	full_scale.val1 = 20;            /* 20 m/s^2 = 2G */
	full_scale.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);

	/* Setting Gyro scale in radians/s*/
	full_scale.val1 = 7;          /* 7 rad/s = 401 degrees/s */
	full_scale.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	/*set callback handler for tilt detector*/
	sensor_trigger_set(dev,&tilt_trig,tilt_trigger_handler);
	/*set callback handler for game rotation vector*/
	sensor_trigger_set(dev,&rotation_trig,rotation_trigger_handler);
	/*set callback handler for gyro*/
	sensor_trigger_set(dev,&gyro_trig,gyro_trigger_handler);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_UPPER_THRESH,
			&gyro_thresh);
	/*set callback handler for accel*/
	sensor_trigger_set(dev,&accel_trig,accel_trigger_handler);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_UPPER_THRESH,
			&accel_thresh);
	int count =0;
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(10));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif
		/*data can be accessed after a fetch, if available. Otherwise a data received
		trigger can be set up, as done for rotation data*/
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);

		/* although sampling is set at 100 hz, as in a real-world application,
		in this sample we dump values only every 1 sec to avoid flooding the console */
		if(count==0)
		{
			printf("Acc: ");
			sensor_print("X",&acc[0],"m/s2");
			sensor_print("Y",&acc[1],"m/s2");
			sensor_print("Z",&acc[2],"m/s2");
			printf("\n");				
			printf("Gyro: ");
			sensor_print("X",&gyr[0],"rad/s");
			sensor_print("Y",&gyr[1],"rad/s");
			sensor_print("Z",&gyr[2],"rad/s");
			printf("\n");
		}				
		if(++count>100)
			count =0;
	}
}

/*The test board needs to be physically manipulated to trigger this event*/
void tilt_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("Tilt Detected!\n");
}

/*rotation data received in FIFO*/
void rotation_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	static int count2 =0;
	struct bhy2_data_quaternion data;
    uint32_t s, ns;
	uint64_t timestamp;
	struct sensor_value quat[5];
	/*Access game rotation using raw interface*/
	bhix60_get_quaternion(dev,trigger->chan,&data,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	/*access rotation using sensor_channel_get*/
	sensor_channel_get(dev,SENSOR_CHAN_GAME_ROTATION_QTR,quat);
	if(count2 ==0)
	{
		/*dump raw value of quaternion vector*/
		printf("Game Rotation Raw: x: %d, y: %d, z: %d, w: %d; acc: %d; AT s:%d ns:%d\n",
			data.x,	data.y,	data.z,	data.w,	data.accuracy,s,ns);
		/*dump processed value of quaternion*/	
		printf("Game Rotation SV: ");
		sensor_print("X",&quat[0],"");
		sensor_print("Y",&quat[1],"");
		sensor_print("Z",&quat[2],"");
		sensor_print("W",&quat[3],"");
		sensor_print("A",&quat[4],"rad");
		printf("\n");
	}
	if(++count2>100)
		count2 =0;
}

void gyro_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("Gyro threshold crossed!\n");
}

void accel_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("Accel threshold crossed!\n");
}

void sensor_print(char *name, struct sensor_value *v, char *unit)
{
	printf(" %s=",name);
	if(v->val1 == 0 && v->val2 <0)
		printf("-0.%06d",-v->val2);
	else if(v->val1 < 0)
		printf("%d.%06d",v->val1,-v->val2);
	else
		printf("%d.%06d",v->val1,v->val2);
	printf(" %s,",unit);
}

