/**
  * @file bhix60_pdr.h
  * @brief BHIx60 driver extended public API for Virtual Sensor BSEC
  * @details  
  * Additional API declarations required for accessing the Air-Quality (BSEC)
  * Virtual Sensor of the BHIx60 family of devices. 
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr.h>
#include <bhix60.h>
#include <bhy2_bsec.h>

#ifndef _BHIX60_BSEC_H_
#define _BHIX60_BSEC_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_bsec {
     SENSOR_CHAN_BHIX60_AIR_Q	    = SENSOR_CHAN_PRIV_START +	BHY2_SENSOR_ID_AIR_QUALITY,	/* Air Quality (BSEC)*/
};

/**
 * @brief Air quality data from BSEC virtual sensor
 */
struct bhix60_air_quality {
  uint16_t                iaq;         //iaq value for regular use case
  uint16_t                iaq_s;       //iaq value for stationary use cases
  uint8_t                 iaq_accuracy;//IAQ Index accuracy level: [0-3]
  struct sensor_value     b_voc_eq;    //breath VOC equivalent (ppm)
  struct sensor_value     co2_eq;      //CO2 equivalent (ppm) [400,]
  struct sensor_value     comp_temp;   //compensated temperature (celcius)
  struct sensor_value     comp_hum;    //compensated humidity %pH
  struct sensor_value     comp_gas;    //compensated gas resistance (Ohms)
};

/**
 * @brief Get air quality from BHIx60 BSEC (Air Quality) Virtual Sensor 
 */
int bhix60_get_air_quality(const struct device *dev,
			      struct bhix60_air_quality *air_quality,
				    uint64_t *timestamp);

#ifdef __cplusplus
}
#endif

#endif //_BHIX60_BSEC_H_
