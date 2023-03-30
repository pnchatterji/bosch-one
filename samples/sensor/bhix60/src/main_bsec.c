/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <bhix60_bsec.h>

/*Define application specific firmware to be uploaded 
Since this sample uses BME688, BMM150, BMP390 as slave to BHI260, a different firmware 
has to be uploaded for Nicla and for BHIx60 shuttle board as the connection of these 
slave devices is different on the two boards (SPI v/s I2C)*/
#ifdef CONFIG_BOARD_ARDUINO_NICLA_SENSE_ME 
	/*Valid for Nicla or compatible
	NOTE: CONFIG_BHIX60_UPLOAD_FW_TO_FLASH=y has to be set in prj.conf in this case, as
	the Nicla firmware below is only suitable for FLASH*/
	#include <firmware/arduino_nicla_sense_me/arduino_nicla_sense_me.fw.h>
	#define bhy2_firmware_image BHI260AP_NiclaSenseME_flash_fw
#else 
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

void bsec_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger);
void main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bhix60_dev));
	struct sensor_value sv_cfg; 

	struct sensor_trigger bsec_trig ={
		.chan = SENSOR_CHAN_BHIX60_AIR_Q,
		.type =	SENSOR_TRIG_DATA_READY,
	};

	if (!device_is_ready(dev)) {
		printf("Device %s is not ready\n", dev->name);
		return;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	/*Configure BSEC virtual sensor for use*/
	sv_cfg.val1 = 100;       /* Sampling rate in hz */
	sv_cfg.val2 = 0;		 /* Latency */
	sensor_attr_set(dev, SENSOR_CHAN_BHIX60_AIR_Q,
			SENSOR_ATTR_CONFIGURATION,
			&sv_cfg);

	/*set callback handler for BSEC data*/
	sensor_trigger_set(dev,&bsec_trig,bsec_trigger_handler);
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

/*BSEC data received in FIFO*/
void bsec_trigger_handler(const struct device *dev,
					 const struct sensor_trigger *trigger)
{
	struct bhix60_air_quality airql;
    uint32_t s, ns;
	uint64_t timestamp;
	bhix60_get_air_quality(dev,&airql,&timestamp);
	bhix60_convert_time(timestamp, &s, &ns);
	printf("BSEC at: %d.%09d iaq=%d, iaqs=%d, acc=%d, bvoc=%d.%06dppm, co2=%d.%06dppm,  temp=%d.%06dC, hum=%d.%06dpH gas=%d.%06d Ohm\n",
		s,ns,
		airql.iaq,
		airql.iaq_s,
		airql.iaq_accuracy,
		airql.b_voc_eq.val1,airql.b_voc_eq.val2,
		airql.co2_eq.val1,airql.co2_eq.val2,
		airql.comp_temp.val1,airql.comp_temp.val2,
		airql.comp_hum.val1,airql.comp_hum.val2,
		airql.comp_gas.val1,airql.comp_gas.val2
	);	
}
