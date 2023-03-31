/**
  * @file bhix60_multi_tap.h
  * @brief BHIx60 driver extended public API for multi-tap virtual sensor for BHI360 device
  * @details 
  * 
  * Additional API declarations required for accessing the multi-tap virtual sensor
  * available on BHI360 and higher (not available on BHI260)
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr/kernel.h>
#include <bhix60.h>
#include <bhi3_multi_tap.h>

#ifndef _BHIX60_BHI3MT_H_
#define _BHIX60_BHI3MT_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_MT { 
  SENSOR_CHAN_BHI3_MULTI_TAP = SENSOR_CHAN_PRIV_START + BHI3_SENSOR_ID_MULTI_TAP, /* Multi-Tap Sensor*/
};

/**
 * @brief Get latest Multi-Tap Data from BHI360 FIFO. This function requires
 * a prior sensor_channel_fetch() on the BHI360 device
 * 
 * @param multi_tap_data multi-tap data 
 * @param timestamp time at which data was recorded (in nanoseconds since start of app)
 * @return  0 on success, errno on failure 
 */
static inline int bhix60_get_multi_tap_data(const struct device *dev,
			      uint8_t *multi_tap_data,
				  uint64_t *timestamp){
    return bhix60_channel_parse_get(dev,SENSOR_CHAN_BHI3_MULTI_TAP,
            (bhix60_parser_t)bhi3_multi_tap_parse_data,multi_tap_data,timestamp);
}

#ifdef __cplusplus

}
#endif

#endif //_BHIX60_BHI3MT_H_
