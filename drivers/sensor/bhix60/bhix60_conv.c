/**
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * @file: Data conversion functions Zephyr Sensor API data format <to/from> BHIx60 FIFO data format 
 */

#include <drivers/sensor.h>
#include <bhix60.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(bhix60, CONFIG_SENSOR_LOG_LEVEL);

/*Input: Quaternion X,Y,Z,W, (range -1 to 1) and Accuracy (radians) ,
	represented as 16bit int scaled by 2^-14 = 16384, 
  Output: Corresponding Unscaled value as integeral part: fraction part in millionths
 */
static void channel_quat_convert(struct sensor_value *val, int64_t raw_val, 
					uint8_t range)
{
	(void)range; /*range is not used in this case*/
	val->val1 = raw_val / 16384LL;
	val->val2 = ((raw_val % 16384LL)*1000000LL)/16384LL;
}

/*Input: Euler Heading, Pitch, Roll in degrees
	represented as 16bit int scaled by 360/2^15 = 360/32768, 
  Output: Corresponding Unscaled value as integeral part: fraction part in millionths*/
static void channel_orient_convert(struct sensor_value *val, int64_t raw_val, 
					uint8_t range)
{
	(void)range; /*range is not used in this case*/
	raw_val = raw_val * 360LL;
	val->val1 = raw_val/32768LL;
	val->val2 = ((raw_val % 32768LL)*1000000LL)/32768LL;
}

/*Common connversion routine for Acclerometer, Linear Acc and Gravity sensor 
Input: 2^15 bits represent the range in G. i.e. if range=1, 2^15 => G 
Output: m/s^2 in integer: millionth */
static void channel_accel_convert(struct sensor_value *val, int64_t raw_val,
				  uint8_t range)
{
	raw_val = (raw_val * SENSOR_G * (int64_t) range) / INT16_MAX;

	val->val1 = raw_val / 1000000LL; /*because SENSOR_G is in micro m s^2*/
	val->val2 = raw_val % 1000000LL;
}

/* Input: 2^15 bits represent the range in degrees/s. i.e. if range=1, 2^15=> 1 degrees/s */
/* Output: radians/s in integer:millionths */
static void channel_gyro_convert(struct sensor_value *val, int64_t raw_val,
				 uint16_t range)
{
	int64_t val_mrs = ((raw_val * (int64_t) range * SENSOR_PI) /*Value in micro radians/s*/
		     				/ (180LL * INT16_MAX)); 
	val->val1 = val_mrs / 1000000LL;
	val->val2 = val_mrs % 1000000LL;
}

/* Input: 2^15 bits represent the range in uT. i.e. if range=1, 2^15=> 1 uT */
/* Output: Gauss in integer:millionths. 1 Tesla = 10000 Gauss i.e. 1Gauss = 100 uT */
static void channel_magn_convert(struct sensor_value *val, int64_t raw_val,
				 uint16_t range)
{
	int64_t val_T = ((raw_val * (int64_t) range * 1000000LL) /*Value in pT */
		     				/INT16_MAX); 
	val->val1 = val_T / (100LL*1000000LL); 			/*pT to Gauss*/
	val->val2 = (val_T % 100000000LL)/100LL;		/*fractional part in millionths*/
}

/*raw temp has a resolution of 0.01 degC. Eg 51.23 degC = 5123.
Output is degrees C, fractional part in millionth
E.g 51 : 230000*/
static void channel_temp_convert(struct sensor_value *val, int raw_val,
				 uint16_t range)
{
	(void)range; 							/*range is not used in this case*/
	val->val1 = raw_val / 100;
	val->val2 = (raw_val % 100) * (1000000/100);
}


/*raw humidity unit is 1% RH, without fractional part. 
Output is RH percentage */
static void channel_humidity_convert(struct sensor_value *val, uint32_t raw_val,
				 uint16_t range)
{
	(void)range; /*range is not used in this case*/
	val->val1 = raw_val;
	val->val2 = 0;
}

/*Raw pressure is in units of  100/128Pa (as a 24bit uint). 
(NOTE:This is wrongly written as 1/128Pa in older versions of the BHI260 product specs )
E.g. Atmospheric Pressure 101325 Pa = 129696
Output is in Kilo Pascal (integral : fractional part in millionths)
E.g. 101 : 325000
*/
static void channel_pressure_convert(struct sensor_value *val, uint64_t raw_val,
				 uint16_t range)
{
	(void)range; /*range is not used in this case*/
	val->val1 = raw_val / (1280); 							/*Kpa*/
	val->val2 = ((raw_val% (1280)) * 100000LL)/128LL;		/*fractional part, millionth Kpa*/
}

/*Raw light data is in units of 10000Lux/216. E.g. 3000Lux = 64
Output is in Lux (integral : fractional part in millionths)*/
static void channel_light_convert(struct sensor_value *val, uint64_t raw_val,
				 uint16_t range)
{
	(void)range; 											/*range is not used in this case*/
	raw_val *= 10000LL; 									/*Original data is 16bit, so no overflow*/
	val->val1 =  raw_val / 216LL; 							/*Lux*/
	val->val2 = ((raw_val % 216LL) * 1000000LL)				/*fractional part, millionth Lux*/ 
						/216LL;			
}

/*Gas resistance is in Ohms
Output is in Ohms (integral : fractional part in millionths)*/
static void channel_gasres_convert(struct sensor_value *val, uint32_t raw_val,
				 uint16_t range)
{
	(void)range; 					/*range is not used in this case*/
	val->val1 = raw_val; 			/*Ohm*/
	val->val2 = 0;					/*fractional part not available*/
}

/*Raw value is a unit-less numeric value : count, bool, status code*/
static void channel_num_convert(struct sensor_value *val, uint32_t raw_val,
				 uint16_t range)
{
	(void)range; 					/*range is not used in this case*/
	val->val1 = raw_val; 			/*same as input*/
	val->val2 = 0;					/*not used*/
}


int bhix60_data_conv(enum sensor_channel chan,uint8_t *data,
			uint16_t range, struct sensor_value *val)
{
	/*typecast to int as both standard and extension channels are 
	being processed, to avoid warnings*/
	switch((int)chan)
	{
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_GRAVITY_XYZ:
		case SENSOR_CHAN_LINEAR_ACCEL_XYZ:
		case SENSOR_CHAN_BHIX60_ACC_PASS:
		case SENSOR_CHAN_BHIX60_ACC_RAW:
		case SENSOR_CHAN_BHIX60_ACC:
		case SENSOR_CHAN_BHIX60_ACC_BIAS:
		case SENSOR_CHAN_BHIX60_ACC_WU:
		case SENSOR_CHAN_BHIX60_ACC_RAW_WU:
		case SENSOR_CHAN_BHIX60_GRA:
		case SENSOR_CHAN_BHIX60_GRA_WU:
		case SENSOR_CHAN_BHIX60_LACC:
		case SENSOR_CHAN_BHIX60_LACC_WU:
		case SENSOR_CHAN_BHIX60_ACC_BIAS_WU:		
			channel_accel_convert(&val[0], BHY2_LE2S16(data),range);
			channel_accel_convert(&val[1], BHY2_LE2S16(data+2),range);
			channel_accel_convert(&val[2], BHY2_LE2S16(data+4),range);
			break;
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_BHIX60_GYRO_PASS:
		case SENSOR_CHAN_BHIX60_GYRO_RAW:
		case SENSOR_CHAN_BHIX60_GYRO:
		case SENSOR_CHAN_BHIX60_GYRO_BIAS:
		case SENSOR_CHAN_BHIX60_GYRO_WU:
		case SENSOR_CHAN_BHIX60_GYRO_RAW_WU:
		case SENSOR_CHAN_BHIX60_GYRO_BIAS_WU:
			channel_gyro_convert(&val[0], BHY2_LE2S16(data),range);
			channel_gyro_convert(&val[1], BHY2_LE2S16(data+2),range);
			channel_gyro_convert(&val[2], BHY2_LE2S16(data+4),range);
			break;
		case SENSOR_CHAN_MAGN_XYZ:
		case SENSOR_CHAN_BHIX60_MAG_PASS:
		case SENSOR_CHAN_BHIX60_MAG_RAW:
		case SENSOR_CHAN_BHIX60_MAG:
		case SENSOR_CHAN_BHIX60_MAG_BIAS:
		case SENSOR_CHAN_BHIX60_MAG_WU:
		case SENSOR_CHAN_BHIX60_MAG_RAW_WU:
		case SENSOR_CHAN_BHIX60_MAG_BIAS_WU:
			channel_magn_convert(&val[0], BHY2_LE2S16(data),range);
			channel_magn_convert(&val[1], BHY2_LE2S16(data+2),range);
			channel_magn_convert(&val[2], BHY2_LE2S16(data+4),range);
			break;
		case SENSOR_CHAN_PROX:
		case SENSOR_CHAN_BHIX60_PROX:
		case SENSOR_CHAN_BHIX60_PROX_WU:
			channel_num_convert(val, data[0],range);
			break;
		case SENSOR_CHAN_AMBIENT_TEMP:
		case SENSOR_CHAN_BHIX60_TEMP:
		case SENSOR_CHAN_BHIX60_TEMP_WU:
			channel_temp_convert(val, BHY2_LE2S16(data),range);
			break;
		case SENSOR_CHAN_HUMIDITY:
		case SENSOR_CHAN_BHIX60_HUM:
		case SENSOR_CHAN_BHIX60_HUM_WU:
			channel_humidity_convert(val, data[0],range);
			break;
		case SENSOR_CHAN_LIGHT:
		case SENSOR_CHAN_BHIX60_LIGHT:
		case SENSOR_CHAN_BHIX60_LIGHT_WU:
			channel_light_convert(val, BHY2_LE2S16(data),range);
			break;
		case SENSOR_CHAN_GAS_RES:
		case SENSOR_CHAN_BHIX60_GAS:
		case SENSOR_CHAN_BHIX60_GAS_WU:
			channel_gasres_convert(val, BHY2_LE2U32(data),range);
			break;
		case SENSOR_CHAN_PRESS:
		case SENSOR_CHAN_BHIX60_BARO:
		case SENSOR_CHAN_BHIX60_BARO_WU:
			channel_pressure_convert(val, BHY2_LE2U24(data),range);
			break;
		case SENSOR_CHAN_ROTATION_QTR:
		case SENSOR_CHAN_GAME_ROTATION_QTR:
		case SENSOR_CHAN_GEOMAG_ROTATION_QTR:
		case SENSOR_CHAN_BHIX60_RV:
		case SENSOR_CHAN_BHIX60_RV_WU:
		case SENSOR_CHAN_BHIX60_GAMERV:
		case SENSOR_CHAN_BHIX60_GAMERV_WU:
		case SENSOR_CHAN_BHIX60_GEORV:
		case SENSOR_CHAN_BHIX60_GEORV_WU:
			channel_quat_convert(&val[0], BHY2_LE2S16(data),range);
			channel_quat_convert(&val[1], BHY2_LE2S16(data+2),range);
			channel_quat_convert(&val[2], BHY2_LE2S16(data+4),range);
			channel_quat_convert(&val[3], BHY2_LE2S16(data+6),range);
			channel_quat_convert(&val[4], BHY2_LE2U16(data+8),range);
			break;
		case SENSOR_CHAN_ORIENTATION_HPR:
		case SENSOR_CHAN_BHIX60_ORI:
		case SENSOR_CHAN_BHIX60_ORI_WU:
			channel_orient_convert(&val[0], BHY2_LE2S16(data),range);
			channel_orient_convert(&val[1], BHY2_LE2S16(data+2),range);
			channel_orient_convert(&val[2], BHY2_LE2S16(data+4),range);
			break;
		case SENSOR_CHAN_STEP_CNT:
		case SENSOR_CHAN_BHIX60_STC:
		case SENSOR_CHAN_BHIX60_STC_WU:
		case SENSOR_CHAN_BHIX60_STC_HW:
		case SENSOR_CHAN_BHIX60_STC_HW_WU:
			channel_num_convert(val, BHY2_LE2U32(data),range);				
			break;
		case SENSOR_CHAN_BHIX60_AR:				/*16 bit bit field*/
		case SENSOR_CHAN_BHIX60_ACTIVITY:
			channel_num_convert(val, BHY2_LE2U16(data),range);				
			break;
		case SENSOR_CHAN_CAMERA_SHUTTER:
		case SENSOR_CHAN_DEVICE_ORIENTATION:
		case SENSOR_CHAN_BHIX60_DEVICE_ORI:
		case SENSOR_CHAN_BHIX60_DEVICE_ORI_WU:
		case SENSOR_CHAN_BHIX60_EXCAMERA:
			channel_num_convert(val, data[0],range);				
			break;
		default: 
			return -ENOTSUP;			/*mapping not supported*/
	}
	return 0;
}

/*Zephyr full scale to BHY2 range conversions*/

/*Input: Range in m/s^2 in integer:millionths
Output: Range in Earth g*/
void accl_range_convert(const struct sensor_value *val,int16_t *range)
{
	int64_t range_ums2 = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*accl range in micro m/s^2*/
	*range = (int16_t)(range_ums2/SENSOR_G);
}

/*Input: Range in radians/s in integer:millionths
Output: Range in degree/s*/
void gyro_range_convert(const struct sensor_value *val,int16_t *range)
{
	int64_t range_urs = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*gyro range in micro rad/s*/
	*range = (int16_t)((range_urs * 180LL)/SENSOR_PI);
}

/*Input Range in Gauss in integer:millionths
Output: Range in uT. */
void magn_range_convert(const struct sensor_value *val,int16_t *range)
{
	int64_t range_ugs = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*magn range in micro gauss*/
	*range = (int16_t)(range_ugs / 10000LL); /*1 Tesla = 10000 Gauss, 1 Gauss = 100 uT*/
}

int bhix60_range_conv(enum sensor_channel chan,	uint16_t *range, 
				const struct sensor_value *val)
{
	/*typecast to int as both standard and extension channels are 
	being processed, to avoid warnings*/
	switch((int)chan)
	{
		case SENSOR_CHAN_ACCEL_XYZ: 
		case SENSOR_CHAN_GRAVITY_XYZ:
			accl_range_convert(val,range);
			break;
		case SENSOR_CHAN_GYRO_XYZ:
			gyro_range_convert(val,range);
			break;
		case SENSOR_CHAN_MAGN_XYZ: 
			magn_range_convert(val,range);
			break;
		default: 
			return -ENOTSUP;			/*range not supported*/
	}
	return 0;
}

/*check if channel corresponds to a Virtual Sensor of Event type*/
bool bhix60_is_event_chan(enum sensor_channel chan)
{
	switch((int)chan)
	{
		/*Custom Channels*/
		case SENSOR_CHAN_BHIX60_TILT:
		case SENSOR_CHAN_BHIX60_STD:
		case SENSOR_CHAN_BHIX60_SIG:
		case SENSOR_CHAN_BHIX60_WAKE_GESTURE:
		case SENSOR_CHAN_BHIX60_GLANCE_GESTURE:
		case SENSOR_CHAN_BHIX60_PICKUP_GESTURE:
		case SENSOR_CHAN_BHIX60_WRIST_TILT_GESTURE:
		case SENSOR_CHAN_BHIX60_STATIONARY_DET:
		case SENSOR_CHAN_BHIX60_MOTION_DET:
		case SENSOR_CHAN_BHIX60_STD_WU:
		case SENSOR_CHAN_BHIX60_STD_HW:
		case SENSOR_CHAN_BHIX60_SIG_HW:
		case SENSOR_CHAN_BHIX60_STD_HW_WU:
		case SENSOR_CHAN_BHIX60_SIG_HW_WU:
		case SENSOR_CHAN_BHIX60_ANY_MOTION:
		case SENSOR_CHAN_BHIX60_ANY_MOTION_WU:
		case SENSOR_CHAN_BHIX60_MULTI_TAP:
		case SENSOR_CHAN_BHIX60_WRIST_GEST:
		case SENSOR_CHAN_BHIX60_WRIST_WEAR_WU:
		case SENSOR_CHAN_BHIX60_NO_MOTION:
		/*Zephyr Channels (proposed)*/
		case SENSOR_CHAN_SIG_MOTION:
		case SENSOR_CHAN_STEP:
		case SENSOR_CHAN_TILT:
		case SENSOR_CHAN_WAKE_GEST:
		case SENSOR_CHAN_GLANCE_GEST:
		case SENSOR_CHAN_PICKUP_GEST:
		case SENSOR_CHAN_WRIST_TILT_GEST:
		case SENSOR_CHAN_STATIONARY:
		case SENSOR_CHAN_MOTION:

			return true;
	}
	return false;
}

/*check if trigger corresponds to a Virtual Sensor of Event type*/
bool bhix60_is_event_trigger(enum sensor_trigger_type trig)
{
	switch((int)trig)
	{
		case SENSOR_TRIG_BHIX60_EVENT:
		/*Zephyr Triggers (proposed)*/
		case SENSOR_TRIG_SIG_MOTION:
		case SENSOR_TRIG_STEP:
		case SENSOR_TRIG_TILT:
		case SENSOR_TRIG_WAKE_GEST:
		case SENSOR_TRIG_GLANCE_GEST:
		case SENSOR_TRIG_PICKUP_GEST:
		case SENSOR_TRIG_WRIST_TILT_GEST:
		case SENSOR_TRIG_STATIONARY:
		case SENSOR_TRIG_MOTION:

			return true;
	}
	return false;
}

/**
 * @brief THRESHOLD COMPUTATIONS 
 * convert Zephyr compliant sensor_value to a BHY2 compatible threshold value 
 * for threshold detection during FIFO parsing
 */

/*Input: Quaternion W (range -1 to 1) as integeral part: fraction part in millionths,
  Output: Corresponding value as 16bit int scaled by 2^-14 = 16384 
 */
static void channel_quat_thresh(const struct sensor_value *val, int32_t *thresh, 
					uint8_t range)
{
	(void)range; /*range is not used in this case*/
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /* W in millionths*/
	*thresh = (t_in * 16384LL) / 1000000LL;
}

/*Input: Euler Heading, Pitch or Roll in degrees as integeral part: fraction part in millionths
  Output: Corresponding value as 16bit int scaled by 360/2^15 = 360/32768*/
static void channel_orient_thresh(const struct sensor_value *val, int32_t *thresh, 
					uint8_t range)
{
	(void)range; /*range is not used in this case*/
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /* degree in millionths*/
	*thresh = (t_in *32768LL)/(360LL*1000000LL);
}

/*Input: m/s^2 in integer: millionth 
Output: 2^15 bits represent the range in G. i.e. if range=1, 2^15 => G */
static void channel_accel_thresh(const struct sensor_value *val, int32_t *thresh,
				  uint8_t range)
{
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*micro ms2*/
	*thresh = (t_in * INT16_MAX) / (SENSOR_G * (int64_t) range);
}

/* Input: radians/s in integer:millionths */
/* Output: 2^15 bits represent the range in degrees/s. i.e. if range=1, 2^15=> 1 degrees/s */
static void channel_gyro_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*uradians/s*/
	*thresh = (t_in * 180LL * INT16_MAX)/ ((int64_t) range * SENSOR_PI); 
}

/* Input: Gauss in integer:millionths. 1 Tesla = 10000 Gauss  i.e. 1 Gauss = 100 uT*/
/* Output: 2^15 bits represent the range in uT. i.e. if range=1, 2^15=> 1 uT */
static void channel_magn_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /*threshold micro gauss*/
	*thresh = (t_in * INT16_MAX * 100LL) / ((int64_t)range * 1000000LL);
}

/*
Input:degrees C, integer :fractional part in millionth E.g 51.23 degC = 51 : 230000
Output : 16 bit with resolution of 0.01 degC. Eg 51.23 degC = 5123.
*/
static void channel_temp_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	(void)range; 							/*range is not used in this case*/
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /* degree C in millionths*/
	*thresh = (t_in * 100LL)/ 1000000LL;
}


/*Input RH percentage as integer: millionths
Output: % RH without fractional part.*/
static void channel_humidity_thresh(const struct sensor_value *val,int32_t *thresh,
				 uint16_t range)
{
	(void)range; /*range is not used in this case*/
	*thresh = val->val1;
	if(val->val2 >= 500000) /*round up fractional part*/
		*thresh+=1;
}

/*
Input is in Kilo Pascal (integral : fractional part in millionths)
E.g. Atmospheric Pressure 101325 Pa = 101 : 325000
Output pressure in units of  100/128Pa (as a 24bit uint). 
E.g. Atmospheric Pressure 101325 Pa = 129696 (0xC5E680)*/
static void channel_pressure_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	(void)range; /*range is not used in this case*/
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /* KiloPascal in millionths*/
	*thresh = (t_in * 128LL *10LL) / 1000000LL;
}

/*Input is in Lux (integral : fractional part in millionths)
Output is in units of 10000Lux/216. E.g. 3000Lux = 64*/
static void channel_light_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	(void)range; 											/*range is not used in this case*/
	int64_t t_in = (int64_t)val->val1 * 1000000LL + (int64_t)val->val2;  /* Lux in millionths*/
	*thresh = (t_in * 216LL)/(10000LL *1000000LL);
}

/*Input is in Ohms (integral : fractional part in millionths)
Output is in Ohms, no fractional part*/
static void channel_gasres_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	(void)range; 					/*range is not used in this case*/
	*thresh = val->val1;
	if(val->val2 >= 500000) 		/*round up fractional part*/
		*thresh+=1;
}

/*Input is a unit-less numeric value : count, bool, status code*/
static void channel_num_thresh(const struct sensor_value *val, int32_t *thresh,
				 uint16_t range)
{
	(void)range; 			/*range is not used in this case*/
	*thresh = val->val1;	/*same as input, val2 not used*/
}

int bhix60_thresh_conv(enum sensor_channel chan,const struct sensor_value *val,
		int32_t *thresh, uint16_t range)
{
	/*typecast to int as both standard and extension channels are 
	being processed, to avoid warnings*/
	switch((int)chan)
	{
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_GRAVITY_XYZ:
		case SENSOR_CHAN_LINEAR_ACCEL_XYZ:
		case SENSOR_CHAN_BHIX60_ACC_PASS:
		case SENSOR_CHAN_BHIX60_ACC_RAW:
		case SENSOR_CHAN_BHIX60_ACC:
		case SENSOR_CHAN_BHIX60_ACC_BIAS:
		case SENSOR_CHAN_BHIX60_ACC_WU:
		case SENSOR_CHAN_BHIX60_ACC_RAW_WU:
		case SENSOR_CHAN_BHIX60_GRA:
		case SENSOR_CHAN_BHIX60_GRA_WU:
		case SENSOR_CHAN_BHIX60_LACC:
		case SENSOR_CHAN_BHIX60_LACC_WU:
		case SENSOR_CHAN_BHIX60_ACC_BIAS_WU:		
			channel_accel_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_BHIX60_GYRO_PASS:
		case SENSOR_CHAN_BHIX60_GYRO_RAW:
		case SENSOR_CHAN_BHIX60_GYRO:
		case SENSOR_CHAN_BHIX60_GYRO_BIAS:
		case SENSOR_CHAN_BHIX60_GYRO_WU:
		case SENSOR_CHAN_BHIX60_GYRO_RAW_WU:
		case SENSOR_CHAN_BHIX60_GYRO_BIAS_WU:
			channel_gyro_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_MAGN_XYZ:
		case SENSOR_CHAN_BHIX60_MAG_PASS:
		case SENSOR_CHAN_BHIX60_MAG_RAW:
		case SENSOR_CHAN_BHIX60_MAG:
		case SENSOR_CHAN_BHIX60_MAG_BIAS:
		case SENSOR_CHAN_BHIX60_MAG_WU:
		case SENSOR_CHAN_BHIX60_MAG_RAW_WU:
		case SENSOR_CHAN_BHIX60_MAG_BIAS_WU:
			channel_magn_thresh(val,thresh,range);
			break;
		case SENSOR_CHAN_PROX:
		case SENSOR_CHAN_BHIX60_PROX:
		case SENSOR_CHAN_BHIX60_PROX_WU:
			channel_num_thresh(val, thresh,range);	
		case SENSOR_CHAN_AMBIENT_TEMP:
		case SENSOR_CHAN_BHIX60_TEMP:
		case SENSOR_CHAN_BHIX60_TEMP_WU:
			channel_temp_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_HUMIDITY:
		case SENSOR_CHAN_BHIX60_HUM:
		case SENSOR_CHAN_BHIX60_HUM_WU:
			channel_humidity_thresh(val,thresh,range);
			break;
		case SENSOR_CHAN_LIGHT:
		case SENSOR_CHAN_BHIX60_LIGHT:
		case SENSOR_CHAN_BHIX60_LIGHT_WU:
			channel_light_thresh(val,thresh,range);
			break;
		case SENSOR_CHAN_GAS_RES:
		case SENSOR_CHAN_BHIX60_GAS:
		case SENSOR_CHAN_BHIX60_GAS_WU:
			channel_gasres_thresh(val,thresh,range);
			break;
		case SENSOR_CHAN_PRESS:
		case SENSOR_CHAN_BHIX60_BARO:
		case SENSOR_CHAN_BHIX60_BARO_WU:
			channel_pressure_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_ROTATION_QTR:
		case SENSOR_CHAN_GAME_ROTATION_QTR:
		case SENSOR_CHAN_GEOMAG_ROTATION_QTR:
		case SENSOR_CHAN_BHIX60_RV:
		case SENSOR_CHAN_BHIX60_RV_WU:
		case SENSOR_CHAN_BHIX60_GAMERV:
		case SENSOR_CHAN_BHIX60_GAMERV_WU:
		case SENSOR_CHAN_BHIX60_GEORV:
		case SENSOR_CHAN_BHIX60_GEORV_WU:
			channel_quat_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_ORIENTATION_HPR:
		case SENSOR_CHAN_BHIX60_ORI:
		case SENSOR_CHAN_BHIX60_ORI_WU:
			channel_orient_thresh(val, thresh,range);
			break;
		case SENSOR_CHAN_STEP_CNT:
		case SENSOR_CHAN_BHIX60_STC:
		case SENSOR_CHAN_BHIX60_STC_WU:
		case SENSOR_CHAN_BHIX60_STC_HW:
		case SENSOR_CHAN_BHIX60_STC_HW_WU:
			channel_num_thresh(val, thresh,range);				
			break;
		case SENSOR_CHAN_BHIX60_AR:				/*16 bit bit field*/
		case SENSOR_CHAN_BHIX60_ACTIVITY:
			channel_num_thresh(val, thresh,range);				
			break;
		case SENSOR_CHAN_CAMERA_SHUTTER:
		case SENSOR_CHAN_DEVICE_ORIENTATION:
		case SENSOR_CHAN_BHIX60_DEVICE_ORI:
		case SENSOR_CHAN_BHIX60_DEVICE_ORI_WU:
		case SENSOR_CHAN_BHIX60_EXCAMERA:
			channel_num_thresh(val, thresh,range);				
			break;
		default: 
			return -ENOTSUP;			/*mapping not supported*/
	}
	return 0;
}
