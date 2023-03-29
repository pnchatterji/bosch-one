/**
  * @file bhix60_pdr.h
  * @brief BHIx60 driver extended public API for Virtual Sensor KLIO
  * @details  
  * Additional API declarations required for accessing the Self-Learning
  * AI Klio Virtual Sensor of the BHIx60 family of devices. 
  * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
  * 
  * SPDX-License-Identifier: Apache-2.0
  */
#include <zephyr/kernel.h>
#include <bhix60.h>
#include <bhy2_klio.h>

#ifndef _BHIX60_KLIO_H_
#define _BHIX60_KLIO_H_
#ifdef __cplusplus
extern "C" {
#endif


enum sensor_channel_bhix60_klio {
     SENSOR_CHAN_BHIX60_KLIO	    = SENSOR_CHAN_PRIV_START +	BHY2_SENSOR_ID_KLIO,	    /* Klio*/
};

static inline void bhy2_bsec_parse_klio_frame(const uint8_t *payload, bhy2_klio_sensor_frame_t *data){
        memcpy(data,payload,sizeof(bhy2_klio_sensor_frame_t));
}

static inline int bhix60_get_klio_frame(const struct device *dev,
			        bhy2_klio_sensor_frame_t *klio_frame,
				uint64_t *timestamp){
    return bhix60_channel_parse_get(dev,SENSOR_CHAN_BHIX60_KLIO,
        (bhix60_parser_t)bhy2_bsec_parse_klio_frame,klio_frame,timestamp);
}

/*!
 * @brief Read and reset current driver status.
 * The driver_status will be set to one of the values in
 * @ref bhy2_klio_driver_error_state_t. This function will not return a driver
 * status of
 * @ref bhy2_klio_driver_error_state_t::KLIO_DRIVER_ERROR_OPERATION_PENDING.
 * but will retry until the pending operation is finished.
 * @param[out] driver_status Pointer to uint32_t
 * @return 0 on success, errno on failure
 */
static inline int bhix60_klio_read_reset_driver_status(const struct device *dev, 
                                        uint32_t *driver_status){
     return ( bhy2_klio_read_reset_driver_status(driver_status, 
                                bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Read learnt pattern.
 * @param[in] id pattern id (only 0 supported now)
 * @param[out] buffer buffer big enough to fit complete pattern fingerprint data
 * @param[in,out] length size of buffer, return size of actual pattern data
 * @return 0 on success, errno on failure
 * After learning of a new pattern has been signaled by a sensor data event,
 * call this function to retrieve the learnt pattern.
 */
static inline int bhix60_klio_read_pattern(const struct device *dev, const uint8_t id, 
                                uint8_t *buffer, uint16_t *length){
     return ( bhy2_klio_read_pattern(id,buffer,length, 
                        bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Set KLIO algorithm learning and recognition state.
 * @param[in] state new state for algorithm
 * @return 0 on success, errno on failure
 * Writing 1 to the reset fields resets all internal algorithm state and
 * removes any patterns loaded for recognition.
 */
static inline int bhix60_klio_set_state(const struct device *dev, 
                        const bhy2_klio_sensor_state_t *state){
     return ( bhy2_klio_set_state(state, 
                        bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Get KLIO algorithm learning and recognition state.
 * @param[out] state current state for algorithm
 * @return 0 on success, errno on failure
 * The resets are always read as 0.
 */
static inline int bhix60_klio_get_state(const struct device *dev, 
                bhy2_klio_sensor_state_t *state){
     return ( bhy2_klio_get_state(state, 
                bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Write pattern to recognition algorithm.
 * @param[in] idx pattern index
 * @param[in] pattern_data containing pattern
 * @param[in] size pattern size
 * @return 0 on success, errno on failure
 *
 * Write a pattern to the specified recognition index. The allowed number of
 * recognition patterns may be retrieved using the
 * @ref bhy2_klio_parameter::KLIO_PARAM_RECOGNITION_MAX_PATTERNS parameter.
 *
 * After a pattern is written, it is in disabled state and not used. It must
 * be enabled using @ref bhy2_klio_set_pattern_states() before it will be used
 * by the recognition algorithm.
 */
static inline int bhix60_klio_write_pattern(const struct device *dev, 
                        const uint8_t idx, 
                        const uint8_t *pattern_data, 
                        const uint16_t size){
     return ( bhy2_klio_write_pattern(idx,pattern_data,size,
                               bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Sets the specified state for the specified pattern ids.
 * @param[in] operation operation to perform
 * @param[in] pattern_ids ids of pattern(s) to perform operation on
 * @param[in] size size of pattern_ids array
 * @return 0 on success, errno on failure
 */
static inline int bhix60_klio_set_pattern_states(const struct device *dev, 
                                const bhy2_klio_pattern_state_t operation,
                                const uint8_t *pattern_ids,
                                const uint16_t size){
     return ( bhy2_klio_set_pattern_states(operation,
                                    pattern_ids,
                                    size,
                                    bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}

/*!
 * @brief Calculates similarity score between two patterns.
 *
 * The similariy between patterns is a meassure of the similarity between the
 * performed activities or exercises. If the two patterns represent the same
 * excercise the similarity will be close to 1.0, and if they are for very
 * different excercises, the similarity will be negative.
 *
 * @param[in]  first_pattern  Buffer containing first pattern.
 * @param[in]  second_pattern Buffer containing second pattern.
 * @param[in]  size           Pattern size.
 * @param[out] similarity     Similarity score.
 * @return 0 on success, errno on failure
 */
static inline int bhix60_klio_similarity_score(const struct device *dev, 
                                const uint8_t *first_pattern,
                                const uint8_t *second_pattern,
                                const uint16_t size,
                                float *similarity){
     return ( bhy2_klio_similarity_score(first_pattern,
                                  second_pattern,
                                  size,
                                  similarity,
                                  bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Calculates similarity score between multiple patterns.
 *
 * This is more efficient than bhy2_klio_similarity_score() since if the patterns
 * are already available on the bhy2, they need not be uploaded again to
 * perform the comparison. And several comparisons may be performed with one
 * API call.
 *
 * @param[in]  idx      Reference pattern.
 * @param[in]  indexes    Buffer containing indexes of patterns to compare reference pattern with.
 * @param[in]  count      Number of patterns in indexes buffer.
 * @param[out] similarity Similarity scores (one score for each pattern in indexes buffer).
 * @return 0 on success, errno on failure
 */
static inline int bhix60_klio_similarity_score_multiple(const struct device *dev, 
                                        const uint8_t idx,
                                        const uint8_t *indexes,
                                        const uint8_t count,
                                        float *similarity){
     return ( bhy2_klio_similarity_score_multiple(idx,indexes,count,similarity,
                                           bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Set KLIO algorithm parameter
 *
 * @param[in] id             Parameter to set.
 * @param[in] parameter_data Buffer containing new parameter value.
 * @param[in] size           Parameter size.
 * @return 0 on success, errno on failure
 */
static inline int bhix60_klio_set_parameter(const struct device *dev, const bhy2_klio_parameter_t id,
                               const void *parameter_data,
                               const uint16_t size){
     return ( bhy2_klio_set_parameter(id,parameter_data,size,
                               bhix60_get_bhy2_dev(dev)) == BHY2_OK)?
        0:-EIO;
}


/*!
 * @brief Get KLIO algorithm parameter
 * @param[in]     id             Parameter id.
 * @param[out]    parameter_data Buffer to store parameter data.
 * @param[in,out] size           Parameter data size.
 * @return 0 on success, errno on failure
 *
 * The expected type or buffer size of parameter_data for respective parameter
 * id is specified in the @ref bhy2_klio_parameter_t enum.
 */
static inline int bhix60_klio_get_parameter(const struct device *dev, 
                                const bhy2_klio_parameter_t id,
                                uint8_t *parameter_data,
                                uint16_t *size){
     return ( bhy2_klio_get_parameter(id,parameter_data,size,
        bhix60_get_bhy2_dev(dev)) == BHY2_OK)? 0:-EIO;
}


#ifdef __cplusplus
}
#endif

#endif //_BHIX60_KLIO_H_
