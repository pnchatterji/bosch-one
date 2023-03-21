/**
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * @file: Support functions for BSEC Air Quality sensor
 */

#include <drivers/sensor.h>
#include <bhix60.h>
#include <logging/log.h>
#include <bhix60_bsec.h>

LOG_MODULE_DECLARE(bhix60, CONFIG_SENSOR_LOG_LEVEL);

static inline float BHY2_LE2F(uint8_t *data)
{
    uint32_t f = BHY2_LE2U32(data);
    float *pf = (float *)&f; 
    return *pf;
}

static void bhix60_parse_air_quality(uint8_t *data,struct bhix60_air_quality *air_quality)
{
  air_quality->iaq = BHY2_LE2U16(&data[0]);
  air_quality->iaq_s = BHY2_LE2U16(&data[2]);
  //b-VOC-eq in the FIFO frame is scaled up by 100
  uint32_t bvoc = BHY2_LE2U16(&data[4]);
  air_quality->b_voc_eq.val1 = bvoc/100;
  air_quality->b_voc_eq.val2 = (bvoc%100)*10000;  //(1000000/100 = 10000)
  air_quality->co2_eq.val1 = BHY2_LE2U24(&data[6]);
  air_quality->co2_eq.val2 = 0;
  air_quality->iaq_accuracy = data[9];
  //comp_temp in the FIFO frame is scaled up by 256
  uint32_t temp = BHY2_LE2S16(&data[10]);
  air_quality->comp_temp.val1 =  temp/256;
  air_quality->comp_temp.val2 =  (temp%256)*3906; //(1000000 /256 = 3906)
  //comp_temp in the FIFO frame is scaled up by 500
  uint32_t hum = BHY2_LE2U16(&data[12]);
  air_quality->comp_hum.val1 = hum/500;
  air_quality->comp_hum.val2 = (hum%500) * 2000;//(1000000 /500 = 2000)
  //gas resistence is in floating pt format in the FIFO data
  float gas = BHY2_LE2F(&data[14]);
  uint32_t igas = (uint32_t)gas;
  air_quality->comp_gas.val1 = igas;
  air_quality->comp_gas.val2 = (uint32_t)((gas - (float)igas)*1000000.0f);
}

int bhix60_get_air_quality(const struct device *dev,
			      struct bhix60_air_quality *air_quality,
				    uint64_t *timestamp)
{
//TODO:MUTEX LOCKING, REMOVE PARSER
    return bhix60_channel_parse_get(dev,SENSOR_CHAN_BHIX60_AIR_Q,
        (bhix60_parser_t)bhix60_parse_air_quality,air_quality,timestamp);
}