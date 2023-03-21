/**
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * @file: Support functions for Pedestrian Dead Reckoning (PDR) sensor
 */

#include "bhix60_pdr.h"

/**
 * @brief convert PDR raw frame data to sensor_value datatype in SI units
 */
static inline void pdr_to_sv(struct sensor_value *sval,int32_t pdrval)
{
        sval->val1 = pdrval /10;
        sval->val2 = (pdrval % 10)*100000;
        /* Normalize val to make sure val->val2 is positive */
	if (sval->val2 < 0) {
		sval->val1 -= 1;
		sval->val2 += 1000000;
	}
}

int bhix60_get_pdr_data(const struct device *dev,
			        struct bhix60_pdr_data *pdr_data,
				uint64_t *timestamp)
{
   //TODO:MUTEX LOCKING
    struct bhy2_pdr_frame pdr_frame;
    int ret = bhix60_channel_parse_get(dev,SENSOR_CHAN_BHIX60_PDR,
    (bhix60_parser_t)bhy2_pdr_parse_frame,&pdr_frame,timestamp);     
    if(ret ==0)
    {
            pdr_to_sv(&pdr_data->x,pdr_frame.pos_x);
            pdr_to_sv(&pdr_data->y,pdr_frame.pos_y);
            pdr_to_sv(&pdr_data->hor_acc,pdr_frame.hor_acc);
            pdr_to_sv(&pdr_data->heading,pdr_frame.heading);
            pdr_to_sv(&pdr_data->heading_acc,pdr_frame.heading_acc);
            pdr_data->step_count = pdr_frame.step_count;
            pdr_data->full_reset_status = (pdr_frame.status & 0x01)  !=0;
            pdr_data->track_reset_status = (pdr_frame.status & 0x02) !=0;
    }
    return ret;
}



