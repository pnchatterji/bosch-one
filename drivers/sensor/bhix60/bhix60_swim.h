/**
  * @file bhix60_swim.h
  * @brief BHIx60 driver extended public API for Virtual Sensor SWIM
  * @details 
  * 
  * Additional API declarations required for accessing the SWIM Virtual Sensor
  * of the BHIx60 family of devices. 
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr/kernel.h>
#include <bhix60.h>
#include <bhy2_swim.h>

#ifndef _BHIX60_SWIM_H_
#define _BHIX60_SWIM_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_swim {
  SENSOR_CHAN_BHIX60_SWIM	    = SENSOR_CHAN_PRIV_START +	BHY2_SENSOR_ID_SWIM,	    /* Swim */
};

/**
 * @brief Get latest SWIM Data from BHIx60 FIFO. This function requires
 * a prior sensor_channel_fetch() on the BHIx60 device
 * 
 * @param swim_data swim data structure 
 * @param timestamp time at which data was recorded (in nanoseconds since start of app)
 * @return  0 on success, errno on failure 
 */
static inline int bhix60_get_swim_data(const struct device *dev,
			      struct bhy2_swim_algo_output *swim_data,
				    uint64_t *timestamp){
    return bhix60_channel_parse_get(dev,SENSOR_CHAN_BHIX60_SWIM,
            (bhix60_parser_t)bhy2_swim_parse_data,swim_data,timestamp);
}


/**
 * @brief Set SWIM configuration parameters like swim length, handedness etc.
 * @param[in] swimcfg swim configuration structure
 * @return  0 on success, -EIO on failure
 */
static inline int bhix60_swim_set_config(const struct device *dev, 
        bhy2_swim_config_param_t *swimcfg){
     return (bhy2_swim_set_config(swimcfg, bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}

/**
 * @brief Get SWIM configuration parameters like swim length, handedness etc.
 * @param[out] swimcfg swim configuration structure
 * @return  0 on success, errno on failure
 */
static inline int bhix60_swim_get_config(const struct device *dev, 
        bhy2_swim_config_param_t *swimcfg){
     return (bhy2_swim_get_config(swimcfg, bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}

/**
 * @brief Get the SWIM Algorithm version
 * @param[out] version firmware version
 * @return  0 on success, errno on failure
 */
static inline int bhix60_swim_get_version(const struct device *dev, 
    bhy2_swim_version_t *version){
     return (bhy2_swim_get_version(version, bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}

#ifdef __cplusplus
}
#endif

#endif //_BHIX60_SWIM_H_
