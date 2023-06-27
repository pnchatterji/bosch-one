/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * This BHIx60 driver sample demonstrates reading the virtual sensors that require 
 * require auxiliary BME688, BMM150, BMP390 devices to be connected to BHIx60
 * This sample also demonstrates setting threshold triggers. It sets a 
 * threshold trigger for orientation sensor (triggered if Heading, Pitch or Roll
 * exceeds +/- 150 degrees ).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <bhix60.h>

/*Define application specific firmware to be uploaded 
Since this sample uses BME688, BMM150, BMP390 as slave to BHI260, a different firmware 
has to be uploaded for Nicla and for BHIx60 shuttle board as the connection of these 
slave devices is different on the two boards (SPI v/s I2C)*/
#if defined(CONFIG_BOARD_BOSCH_NICLA_SENSE) 
# if !defined(CONFIG_BHIX60_UPLOAD_FW_TO_FLASH) ||defined(CONFIG_BHIX60_UPLOAD_FW_TO_RAM)
#    error 	"CONFIG_BHIX60_UPLOAD_FW_TO_FLASH Should be set for this firmware"
# endif 
	/*Valid for Nicla or compatible
	NOTE: CONFIG_BHIX60_UPLOAD_FW_TO_FLASH=y has to be set in prj.conf in this case, as
	the Nicla firmware below is only suitable for FLASH*/
	#include <firmware/arduino_nicla_sense_me/arduino_nicla_sense_me.fw.h>
	#define bhy2_firmware_image BHI260AP_NiclaSenseME_flash_fw
#else 
# if defined(CONFIG_BHIX60_UPLOAD_FW_TO_FLASH) || !defined(CONFIG_BHIX60_UPLOAD_FW_TO_RAM)
#    error 	"CONFIG_BHIX60_UPLOAD_FW_TO_RAM Should be set for this firmware"
# endif 
	/*valid for Bosch AB3 with BHIx60 Shuttle Board, or compatible
	NOTE: CONFIG_BHIX60_UPLOAD_FW_TO_RAM=y has to be set in prj.conf in this case, as
	the below firmware is only suitable for RAM*/
	#include <firmware/bhi260ap/BHI260AP_aux_BMM150_BMP390_BME688.fw.h>
#endif

int bhix60_get_firmware(const struct device *dev, unsigned char **fw,unsigned int *fw_sz)
{
     (void)dev;
     *fw = (unsigned char *)bhy2_firmware_image;
     *fw_sz = sizeof(bhy2_firmware_image);
     return 0;
}
void sensor_print(char *name, struct sensor_value *v, char *unit);

void orient_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void magn_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value full_scale,sv_cfg;
	struct sensor_value  sv_temp,sv_pres,sv_gas,sv_hum,sv_mag[3],sv_orient[3]; 
	/*Set thresholds for orientation triggers to +/- 150 degrees*/
	struct sensor_value orient_thresh_up = {150,0};
	struct sensor_value orient_thresh_dn = {-150,0};
	/*Set threshold for magnetometer trigger to 1.5 Gauss*/
	struct sensor_value magn_thresh = {1,500000};
	struct sensor_trigger orient_trig ={
		.chan = SENSOR_CHAN_ORIENTATION_HPR,
		.type =	SENSOR_TRIG_THRESHOLD,
	};
	struct sensor_trigger magn_trig ={
		.chan = SENSOR_CHAN_MAGN_XYZ,
		.type =	SENSOR_TRIG_THRESHOLD,
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/*Configure BSEC virtual sensor for use*/
	sv_cfg.val1 = 1;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_GAS_RES,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_PRESS,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_MAGN_XYZ,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	sensor_attr_set(dev, SENSOR_CHAN_ORIENTATION_HPR,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);
	/* Setting Magnetometer scale in Gauss */
	full_scale.val1 = 25;            /* 25 Gauss = 2500 uT */
	full_scale.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_MAGN_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);

	/*set callback handler for orientation*/
	sensor_trigger_set(dev,&orient_trig,orient_trigger_handler);
	sensor_attr_set(dev, SENSOR_CHAN_ORIENTATION_HPR,
			SENSOR_ATTR_LOWER_THRESH,
			&orient_thresh_dn);
	sensor_attr_set(dev, SENSOR_CHAN_ORIENTATION_HPR,
			SENSOR_ATTR_UPPER_THRESH,
			&orient_thresh_up);
	/*set callback handler for magnetometer*/
	sensor_trigger_set(dev,&magn_trig,magn_trigger_handler);
	sensor_attr_set(dev, SENSOR_CHAN_MAGN_XYZ,
			SENSOR_ATTR_UPPER_THRESH,
			&magn_thresh);
	while (1) {
		/* 1000ms period, 1Hz Sampling frequency */
		k_sleep(K_MSEC(1000));

		/*If FIFO poll mode is active, fetch needs to be called in 
		a loop to unload events from the BHI60 FIFO loop.*/
#ifdef CONFIG_BHIX60_FIFO_POLL
		sensor_sample_fetch(dev);
#endif
		sensor_channel_get(dev,SENSOR_CHAN_AMBIENT_TEMP,&sv_temp);
		sensor_channel_get(dev,SENSOR_CHAN_GAS_RES,&sv_gas);
		sensor_channel_get(dev,SENSOR_CHAN_PRESS,&sv_pres);
		sensor_channel_get(dev,SENSOR_CHAN_HUMIDITY,&sv_hum);
		printf("T=%d.%06d C, GR=%d.%06d Ohm, P=%d.%06d kPa, H=%d.%06d pH\n",
			sv_temp.val1,sv_temp.val2,		
			sv_gas.val1,sv_gas.val2,		
			sv_pres.val1,sv_pres.val2,		
			sv_hum.val1,sv_hum.val2
		);
		sensor_channel_get(dev,SENSOR_CHAN_MAGN_XYZ,sv_mag);
		printf("Magnetometer");
		sensor_print("X",&sv_mag[0],"Gs");
		sensor_print("Y",&sv_mag[1],"Gs");
		sensor_print("Z",&sv_mag[2],"Gs");
		printf("\n");
		sensor_channel_get(dev,SENSOR_CHAN_ORIENTATION_HPR,sv_orient);
		printf("Orientation");
		sensor_print("H",&sv_orient[0],"d");
		sensor_print("P",&sv_orient[1],"d");
		sensor_print("R",&sv_orient[2],"d");
		printf("\n");
	}
}

/*orientation exceeds threshold in H, P or R*/
void orient_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("Orientation Exeedes 150 degrees in H P or R!\n");
}

/*Magnetometer exceeds threshold in X, Y or Z*/
/*NOTE: It may be necessary to bring a magnet close to the board to fire this!*/
void magn_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	printf("Magnetometer Threshold in X, Y or Z exceeded!\n");
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


