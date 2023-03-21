/**
 * @file bhix60_int.h
 * @brief BHIx60 driver internal API
 * @copyright Copyright (c) 2023 Bosch Sensortec GmbH
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BHIX60_INT_H_
#define _BHIX60_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <bhy2.h>
#include <drivers/sensor.h>

int bhix60_data_conv(enum sensor_channel chan,uint8_t *data,
			uint16_t range, struct sensor_value *val);

int bhix60_range_conv(enum sensor_channel chan,	uint16_t *range, 
				const struct sensor_value *val);

int bhix60_thresh_conv(enum sensor_channel chan,const struct sensor_value *val,
		int32_t *thresh, uint16_t range);

bool bhix60_is_event_trigger(enum sensor_trigger_type trig);

bool bhix60_is_event_chan(enum sensor_channel chan);

const char* bhix60_get_api_error(int8_t error_code);

const char* bhix60_get_sensor_name(uint8_t sensor_id);

const char* bhix60_get_sensor_error_text(uint8_t sensor_error);

void bhix60_parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, 
                    void *callback_ref);

void bhix60_parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, 
                    void *callback_ref);

void bhix60_convert_time(uint64_t time_ticks, uint32_t *s, uint32_t *ns);

void bhix60_print_sensors(struct bhy2_dev *p_bhy2);

#ifdef __cplusplus
}
#endif

#endif //_BHIX60_INT_H_