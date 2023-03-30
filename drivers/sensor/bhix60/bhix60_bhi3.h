/**
  * @file bhix60_bhi3.h
  * @brief BHIx60 driver extended public API for BHI360 device
  * @details 
  * 
  * Additional API declarations required for accessing the special features
  * of the BHI360 device  
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr/kernel.h>
#include <bhix60.h>
#include <bhi3.h>

#ifndef _BHIX60_BHI3_H_
#define _BHIX60_BHI3_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_BHI3 { 
  SENSOR_CHAN_BHI3_AR_WEAR_WU       = SENSOR_CHAN_PRIV_START + BHI3_SENSOR_ID_AR_WEAR_WU,              /* Activity Recognition (wear/hear) */
  SENSOR_CHAN_BHI3_WRIST_GEST_LP_WU = SENSOR_CHAN_PRIV_START + BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, /* Wrist Gesture Low Power Wakeup*/
  SENSOR_CHAN_BHI3_WRIST_WEAR_LP_WU = SENSOR_CHAN_PRIV_START + BHI3_SENSOR_ID_WRIST_WEAR_LP_WU,        /* Wrist Wear Low Power Wakeup */
  SENSOR_CHAN_BHI3_NO_MOTION_LP_WU  = SENSOR_CHAN_PRIV_START + BHI3_SENSOR_ID_NO_MOTION_LP_WU,         /* No motion Low Power */
};

/**
 * @brief Get latest Wrist Gesture Detection Data from BHI360 FIFO. This function requires
 * a prior sensor_channel_fetch() on the BHI360 device
 * 
 * @param wrist_gest_data Wrist Gesture data structure 
 * @param timestamp time at which data was recorded (in nanoseconds since start of app)
 * @return  0 on success, errno on failure 
 */
static inline int bhix60_get_wrist_gest_data(const struct device *dev,
			      bhi3_wrist_gesture_detect_t *wrist_gest_data,
				    uint64_t *timestamp){
    return bhix60_channel_parse_get(dev,SENSOR_CHAN_BHI3_WRIST_GEST_LP_WU,
            (bhix60_parser_t)bhi3_wrist_gesture_detect_parse_data,wrist_gest_data,timestamp);
}

#ifdef __cplusplus

}
#endif

#endif //_BHIX60_BHI3_H_
