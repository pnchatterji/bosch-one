/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * This BHix60 driver sample demonstrates the use of the bhix60 driver
 * with custom virtual sensors and custom firmware
 * 
 * *******************************************************************
 * FOR INFORMATION ONLY. THIS SAMPLE CANNOT BE BUILT OR RUN          *
 * *******************************************************************
 * Since this sample refers to a fictitious firmware file that does not 
 * currently exist, it cannot be built or run. It is left as an exercise
 * to the user to create a suitable firmware using the BHY2 Firmware SDK  
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <bhix60.h>


/*Define application specific firmware to be uploaded*/

/*Custom firmware file developed by the user*/
#include "MyCustomFirmware.fw.h"

int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)my_custom_firmware_image;
     *fw_sz = sizeof(my_custom_firmware_image);
     return 0;
}

/*BHY2 Sensor IDs for the custom virtual sensors*/
/*Custom sensor IDs must be in the range BHY2_SENSOR_ID_CUSTOM_START
to BHY2_SENSOR_ID_CUSTOM_END*/

#define MY_CUSTOM_DATA_SID 	BHY2_SENSOR_ID_CUSTOM_START 
#define MY_CUSTOM_EVENT_SID BHY2_SENSOR_ID_CUSTOM_START +1 

/*Define the custom Channels*/
enum sensor_channel_bhix60 {
  SENSOR_CHAN_MY_CUSTOM_DATA	 = SENSOR_CHAN_PRIV_START +	MY_CUSTOM_DATA_SID,
  SENSOR_CHAN_MY_CUSTOM_EVENT	 = SENSOR_CHAN_PRIV_START +	MY_CUSTOM_EVENT_SID,
};

/*Assume that there are two custom virtual sensors:

custom sensor1 returns a 9 byte FIFO packet with two 32 bit integer elements 
called foo and bar, plus one header byte

custom sensor2 is of type Event, and returns no data
*/

/*Data type of extracted custom data*/
struct my_custom_data {
	uint32_t foo;
	uint32_t bar;
};

/*Define parser for the custom data type*/
void parse_my_custom_data(const uint8_t *data, struct my_custom_data *mcd)
{
    mcd->foo = BHY2_LE2U32(data);
    mcd->bar = BHY2_LE2U32(data + 4);
}

/*Define data accessor macro*/
static inline int get_my_custom_data(const struct device *dev,
			      enum sensor_channel chan,
			      struct my_custom_data *mcd,
				    uint64_t *timestamp){
    return bhix60_channel_parse_get(dev,chan,(bhix60_parser_t)parse_my_custom_data,mcd,timestamp);
};


/*Trigger handler for the DATA_READY trigger for Custom Channel1*/
void my_custom_data_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);

/*Trigger handler for the EVENT trigger for Custom Channel2*/
void my_custom_event_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);

void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value sv_cfg; 
	struct sensor_trigger my_custom_data_trig ={
		.chan 	=	SENSOR_CHAN_MY_CUSTOM_DATA,
		.type	=	SENSOR_TRIG_DATA_READY,
	};
	struct sensor_trigger my_custom_event_trig ={
		.chan 	=	SENSOR_CHAN_MY_CUSTOM_EVENT,
		.type	=	SENSOR_TRIG_BHIX60_EVENT, 
		/*It is also possible to use any other standard Event-type 
		trigger ID E.g. if the event is associated with a motion 
		of some kind, one could use SENSOR_TRIG_MOTION*/
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
	sensor_attr_set(dev, SENSOR_CHAN_MY_CUSTOM_DATA,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_MY_CUSTOM_EVENT,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);

	/* If my_custom_data virtual sensor supports scaling, it can be
	scaled here using low-level BHY2 API calls
	sensor_attr_set() with SENSOR_ATTR_FULL_SCALE is not currently 
	supported for custom channels
	Refer BHY2 API for details*/

	/* If the custom virtual sensors require additional custom
	configuration, it can be configured here using low-level BHY2 API calls
	Refer BHY2 API for details*/

	/*set callback handler for the custom channels*/
	sensor_trigger_set(dev,&my_custom_data_trig,my_custom_data_trigger_handler);
	sensor_trigger_set(dev,&my_custom_event_trig,my_custom_event_trigger_handler);
	int count =0;
	while (1) {
		/* 10ms period, 100Hz Sampling frequency */
		k_sleep(K_MSEC(10));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif
		/*data can be accessed after a fetch, if available. Otherwise 
		it can be done in a data received trigger, as done below*/
}

/*my custom data received in FIFO*/
void my_custom_data_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	struct my_custom_data data;
    uint32_t s, ns;
	uint64_t timestamp;
	/*Access my_custo_data using raw interface*/
	get_my_custom_data(dev,trigger->chan,&data,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	/*dump custom data to console*/
	printf("My Custom Data: foo: %d, bar: %d AT s:%d ns:%d\n",
		data.foo,
		data.bar,
		s,ns);
}

/*my custom event received in FIFO*/
void my_custom_event_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("My Custom Event has occurred!\n");
}
