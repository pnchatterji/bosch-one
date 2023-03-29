/**
  * @file bhix60_pdr.h
  * @brief BHIx60 driver extended public API for Virtual Sensor PDR
  * @details Pedestrian Dead Reckoning (PDR) 
  * 
  * Additional API declarations required for accessing the PDR Virtual Sensor
  * of the BHIx60 family of devices. 
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr/kernel.h>
#include <bhix60.h>
#include <bhy2_pdr.h>

#ifndef _BHIX60_PDR_H_
#define _BHIX60_PDR_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_pdr {
   SENSOR_CHAN_BHIX60_PDR       = SENSOR_CHAN_PRIV_START + BHY2_SENSOR_ID_PDR,     /* PDR */
};

/**
 * @brief Processed PDR Data
 * PDR Data in SI units in Zephyr-compatible sensor_value data type
 */
struct bhix60_pdr_data
{
    struct sensor_value x;            /*Displacement X (m)*/
    struct sensor_value y;            /*Displacement Y (m)*/
    struct sensor_value hor_acc;      /*Horizontal accuracy (m)*/
    struct sensor_value heading;      /*Heading (degrees)*/
    struct sensor_value heading_acc;  /*Heading accuracy (degrees)*/
    uint32_t step_count;              /*Number of steps taken*/
    bool full_reset_status;           /*PDR has been reset*/
    bool track_reset_status;          /*Track has been reset*/
};

/**
 * @brief Get latest PDR Data from BHIx60 FIFO. This function requires
 * a prior sensor_channel_fetch() on the BHIx60 device
 * This function returns processed FIFO data as Zephyr-compatible sensor_value
 * structures in SI units, after extracting raw data from the PDR Frame and 
 * applying conversion factors.
 * @param pdr_data PDR data structure 
 * @param timestamp time at which data was recorded (in nanoseconds since start of app)
 * @return  0 on success, errno on failure 
 */
int bhix60_get_pdr_data(const struct device *dev,
			        struct bhix60_pdr_data *pdr_data,
				uint64_t *timestamp);

/**
 * @brief full reset of PDR algorithm
 * 
 * @return  0 on success, errno on failure 
 */
static inline int bhix60_pdr_reset_full(const struct device *dev){
        return (bhy2_pdr_reset_full(bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

/**
 * @brief reset position (depricated)
 * Has no effect at present
 * @return  0 on success, errno on failure 
 */
static inline int bhix60_pdr_reset_position(const struct device *dev){
        return (bhy2_pdr_reset_position(bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

/**
 * @brief Set Heading Delta
 * 
 * @param heading delta in degrees (precision 0.1 degrees)
 * @return 0 on success, errno on error
 */
static inline int bhix60_pdr_set_ref_heading_del(const struct device *dev,
                float heading){
        return (bhy2_pdr_set_ref_heading_del(heading, 
                bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

/**
 * @brief Set length of step and accuracy of step
 * 
 * @param step_length step length in meters (maximum 500m, precision 0.01m)
 * @param step_length_acc step length accuracy in meters
 * @return 0 on success, errno on error
 */
static inline int bhix60_pdr_set_step_info(const struct device *dev,
                float step_length, float step_length_acc){
        return (bhy2_pdr_set_step_info(step_length,step_length_acc, 
                bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

/**
 * @brief Set handedness of device : left hand or right hand
 * 
 * @param right_hand 0: left hand; 1: right hande 
 * @return 0 on success, errno on error
 */
static inline int bhix60_pdr_set_hand(const struct device *dev,
                uint8_t right_hand){
        return (bhy2_pdr_set_hand(right_hand, 
                bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

enum bhix60_pdr_position {
    wrist       = BHY2_PDR_DEV_POS_WRIST,
    head        = BHY2_PDR_DEV_POS_HEAD,
    shoe        = BHY2_PDR_DEV_POS_SHOE,
    backpack    = BHY2_PDR_DEV_POS_BACKPACK,
};

/**
 * @brief Get physical position of device
 * 
 * @param dev_pos position of device 
 * @return int 
 */
static inline int bhix60_pdr_get_device_position(const struct device *dev,
                enum bhix60_pdr_position *dev_pos){
        return (bhy2_pdr_get_device_position(dev_pos, 
        bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

static inline int bhix60_pdr_get_driver_version(const struct device *dev,
                struct bhy2_pdr_ver *version){
        return (bhy2_pdr_get_driver_version(version, 
                        bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                        0:-EIO;
}

static inline int bhix60_pdr_get_algo_version(const struct device *dev,
                struct bhy2_pdr_ver *version){
        return (bhy2_pdr_get_algo_version(version, bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

static inline int bhix60_pdr_get_pdr_variant(const struct device *dev,
                uint8_t *variant){
        return (bhy2_pdr_get_pdr_variant(variant, 
        bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
                0:-EIO;
}

#ifdef __cplusplus
}
#endif

#endif //_BHIX60_PDR_H_
