/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * Driver for battery level sensing via ADC and voltage divider. The 
 */
#define DT_DRV_COMPAT bat_level_adc
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#define MODULE bat_level_adc
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE);

/*ADC Settings
TODO: Make this configurable in DTS?*/
#define ADC_RESOLUTION			10 							
#define ADC_GAIN				ADC_GAIN_1_6 				
#define ADC_REFERENCE			ADC_REF_INTERNAL 			
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT 		


uint8_t battery_level_in_percentage(const struct device *dev, const uint32_t mvolts);

struct bat_level_adc_config {
	const struct device *adc_dev; 
	uint8_t io_channel;
	struct gpio_dt_spec power_gpios;	
	uint32_t output_ohm;
	uint32_t full_ohm;
	uint32_t bat_max_mv;
	uint32_t bat_min_mv;
	uint32_t work_interval_ms;
	uint32_t lut_delta;
	uint32_t lut_len;
	uint32_t bat_mv_soc_lut[];
};

struct bat_level_adc_data {
	uint32_t battery_mv;			//current battery voltage in mv
	uint32_t battery_soc;			//current battery state-of-charge in %
};

static int16_t batt_adc_buffer;

static struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,	
	.channel_id = 0,	/* channel ID will be overwritten below */
	.differential = 0
};

static struct adc_sequence sequence = {
	.channels    = 0,						/* individual channels will be added below */
	.buffer      = &batt_adc_buffer,	
	.buffer_size = sizeof(batt_adc_buffer),	/* buffer size in bytes, not number of samples */
	.resolution  = ADC_RESOLUTION,
};

struct adc_work_context {

	struct k_work_delayable d_work;
	const struct device *adc_dev;
	uint32_t work_interval_ms;

}adc_work_ctx;

static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event  async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &async_sig);

static bool adc_async_read_pending;
static atomic_t batt_adc_reading = ATOMIC_INIT(0);

/*Battery ADC read function which is called cyclically by the zephyr Work Thread*/
static void adc_read_fn(struct k_work *work)
{
	int err;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct adc_work_context *ctx = CONTAINER_OF(dwork,struct adc_work_context,d_work);

	if (!adc_async_read_pending) {
		err = adc_read_async(ctx->adc_dev,&sequence, &async_sig);
		if (err != 0) {
			LOG_WRN("ADC async read failed with error %d",err);
		} else {
				adc_async_read_pending = true;
		}
	} else {
		err = k_poll(&async_evt, 1, K_NO_WAIT);
		if (err) {
			LOG_WRN("ADC poll failed with error %d", err);
		} else {
			adc_async_read_pending = false;
			atomic_set(&batt_adc_reading,batt_adc_buffer);
		}		
	}
	k_work_reschedule(dwork, K_MSEC(ctx->work_interval_ms));
}

/*Initialize ADC input for reading battery level and set up the ADC read task for Zephyr Work thread. 
*/
int bat_meas_init(const struct device *dev)
{
	const struct bat_level_adc_config *cfg = dev->config;
	int err;
	if (!device_is_ready(cfg->adc_dev)) {
		LOG_WRN("ADC device for battery level monitoring not found");
		return -ENOENT;
	}
	adc_work_ctx.adc_dev = cfg->adc_dev;
	adc_work_ctx.work_interval_ms = cfg->work_interval_ms;
	channel_cfg.channel_id = cfg->io_channel;
	channel_cfg.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
	 			     + cfg->io_channel;
	//channel_cfg.input_positive = SAADC_CH_PSELP_PSELP_VDD;  		//for testing
	//channel_cfg.input_positive = SAADC_CH_PSELP_PSELP_VDDHDIV5;	//for testing
	channel_cfg.input_negative = SAADC_CH_PSELN_PSELN_NC;

	adc_channel_setup(cfg->adc_dev, &channel_cfg);
	sequence.channels |= BIT(cfg->io_channel);

	err = gpio_pin_configure_dt(&cfg->power_gpios,
			   GPIO_OUTPUT | GPIO_OPEN_DRAIN);
	if (err < 0) {
		LOG_ERR("Unable to configure voltage divider gpio");
		return err;
	}
	err = gpio_pin_set_dt(&cfg->power_gpios,1);
	if (err < 0) {
		LOG_ERR("Cannot enable battery monitor circuit: %d",err);
		return err;
	}

	k_work_init_delayable(&adc_work_ctx.d_work, adc_read_fn);
	k_work_reschedule(&adc_work_ctx.d_work, K_MSEC(cfg->work_interval_ms));
	return 0;
}
/**@brief Function for converting battery voltage to percentage.
 *
 * @details This is a Look Up Table version of above function, inspired by similar
 * samples in Zephyr
 */

uint8_t battery_level_in_percentage(const struct device *dev, const uint32_t mvolts)
{
	const struct bat_level_adc_config *cfg = dev->config;
    uint8_t battery_level=0;

	if(mvolts>=cfg->bat_max_mv) {
		battery_level =100;
	}
	else if(mvolts<=cfg->bat_min_mv) {
		battery_level=0;
	}
	else{
		size_t lut_id = (mvolts - cfg->bat_min_mv + (cfg->lut_delta >> 1))
								 / cfg->lut_delta;
		if(lut_id >= cfg->lut_len) //safety check, just in case...
			lut_id = cfg->lut_len -1;
		battery_level = cfg->bat_mv_soc_lut[lut_id];
	}
    
    return battery_level;
}

/*!
 * @brief This API is used to read the battery status .
 *
 * @param[out] bat_status_mv            :  Buffer to retrieve the battery status in millivolt.
 *
 * @param[out] bat_status_percent       :  Buffer to retrieve the battery status in %.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t read_bat_status(const struct device *dev)
{
	struct bat_level_adc_data *devdata = dev->data;
	const struct bat_level_adc_config *cfg = dev->config;

	int32_t raw_value = (int32_t)atomic_get(&batt_adc_reading);		
	int32_t mv_value = raw_value;
	int32_t adj_mv_value = raw_value;
	uint8_t batt_level =0;
	int32_t adc_vref = adc_ref_internal(cfg->adc_dev);
	int err = adc_read(cfg->adc_dev, &sequence);
	if (err != 0) {
		LOG_ERR("ADC reading failed with error %d", err);
		return err;
	}
	if (adc_vref > 0) {
			//Convert raw reading to millivolts if driver
			//supports reading of ADC reference voltage
		adc_raw_to_millivolts(adc_vref, ADC_GAIN,
			ADC_RESOLUTION, &mv_value);
	}
	if (cfg->output_ohm != 0) {
			adj_mv_value = mv_value * (uint64_t)cfg->full_ohm
				/ cfg->output_ohm;
	} else {
			adj_mv_value = mv_value;
	}
	batt_level = battery_level_in_percentage(dev, adj_mv_value);
	devdata->battery_mv = adj_mv_value;
	devdata->battery_soc = batt_level;
	// LOG_DBG("buf=%d raw_value=%d mv_value=%d adj_mv_value=%d adc_vref=%d batt_level=%d",
	// 	(int)batt_adc_buffer,(int)raw_value,(int)mv_value,(int)adj_mv_value,(int)adc_vref,(int)batt_level);
	return 0;
}

/**
 * @brief Sensor API supprt functions
 * functions and definitions required to wrap ADC battery level measurement code 
 * into a Zephyr Sensor API compliant driver
 */

/**
 * @brief sensor value get
 * sensor value : val1: integral part val2:decimal part, multiplied by 1 million
 * @return -ENOTSUP for unsupported channels
 */
static int bat_level_adc_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct bat_level_adc_data *devdata = dev->data;
	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		val->val1 = ((devdata->battery_mv / 1000));
		val->val2 = ((devdata->battery_mv % 1000) * 1000U);
		break;
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		val->val1 = devdata->battery_soc;
		val->val2 = 0;
		break;
	//TODO: support other channels like charging current etc., if possible 
	//(see \zephyr\drivers\sensor\bq274xx\bq274xx.c for other possibilities)
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int bat_level_adc_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{


	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		if(read_bat_status(dev)) {
			LOG_ERR("Failed to read battery status");
			return -EIO;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief initialise the battery manager
 *
 * @return 0 for success
 */
static int bat_level_adc_init(const struct device *dev)
{
	return bat_meas_init(dev);
}

static const struct sensor_driver_api bat_level_adc_batlevel_api = {
	.sample_fetch = bat_level_adc_sample_fetch,
	.channel_get = bat_level_adc_channel_get,
};

const static struct bat_level_adc_config bat_level_adc_config_0index = {
	.adc_dev = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(DT_INST(0,DT_DRV_COMPAT))),
	.io_channel = DT_IO_CHANNELS_INPUT(DT_INST(0,DT_DRV_COMPAT)),
	.power_gpios =	GPIO_DT_SPEC_INST_GET(0, power_gpios),	
	.output_ohm = DT_PROP(DT_INST(0,DT_DRV_COMPAT), output_ohms),
	.full_ohm = DT_PROP(DT_INST(0,DT_DRV_COMPAT), full_ohms),
	.bat_max_mv = DT_PROP(DT_INST(0,DT_DRV_COMPAT), bat_max_mv),
	.bat_min_mv = DT_PROP(DT_INST(0,DT_DRV_COMPAT), bat_min_mv),
	.work_interval_ms = DT_PROP(DT_INST(0,DT_DRV_COMPAT), work_interval_ms),
	.lut_delta = DT_PROP(DT_INST(0,DT_DRV_COMPAT), lut_delta),
	.lut_len = DT_PROP_LEN(DT_INST(0,DT_DRV_COMPAT), bat_mv_soc_lut),
	.bat_mv_soc_lut = DT_PROP(DT_INST(0,DT_DRV_COMPAT), bat_mv_soc_lut),
};

static struct bat_level_adc_data bat_level_adc_data_0index = {0};

DEVICE_DT_INST_DEFINE(0, &bat_level_adc_init, NULL, 
		      &bat_level_adc_data_0index,&bat_level_adc_config_0index, 
			  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &bat_level_adc_batlevel_api);

