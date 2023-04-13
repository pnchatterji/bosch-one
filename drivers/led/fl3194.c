/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_is31fl3194

/**
 * @file
 * @brief IS31FL3194 LED driver for Zephyr
 * Limitations:
 * Blink mode is currently not supported
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fl3194);

/*FL3194 Product ID*/
#define FL3194_PROD_ID		0xCE

/* FL3194 Init Registers */
#define FL3194_ID_REG        0x00
#define FL3194_OP_CONF_REG   0x01
#define FL3194_OUT_EN_REG    0x02
#define FL3194_CURRENT_REG   0x03
#define FL3194_HOLD_FCT_REG  0x04
#define FL3194_RESET_REG	 0x4F

/*Current Level Registers*/
#define FL3194_OUT1_REG			0x10  
#define FL3194_OUT2_REG    		0x21
#define FL3194_OUT3_REG    		0x32

/*Color update registers*/
#define FL3194_UPDATE_REG 		0x40

/*Code to execute commands*/
#define FL3194_EXECUTE_CMD		0xC5

/*FL3194 Constants*/
#define FL3194_NUM_LED 			3
#define FL3194_MAX_BRIGHTNESS 	0x7F
#define FL3194_MIN_BRIGHTNESS 	0x00

/*Operating Configure Register settings*/
#define FL3194_SOFTWARE_SHUTDOWN_MODE          UINT8_C(0)
#define FL3194_NORMAL_OPERATION                UINT8_C(1)

#define FL3194_SINGLE_MODE                     UINT8_C(3 << 1) // also UINT8_C(0 << 1)
#define FL3194_RG_W_MODE                       UINT8_C(1 << 1)
#define FL3194_RGB_MODE                        UINT8_C(2 << 1)

#define FL3194_CURRENT_LVL_MODE                UINT8_C(0 << 4)
#define FL3194_PATTERN_MODE                    UINT8_C(1 << 4)

/*Output Enable Register settings*/
#define FL3194_OUTPUT_1                        UINT8_C(1)
#define FL3194_OUTPUT_2                        UINT8_C(1 << 1)
#define FL3194_OUTPUT_3                        UINT8_C(1 << 2)

/*Current Band Register settings*/
#define FL3194_OP1_CURRENT_10MA                UINT8_C(0)
#define FL3194_OP1_CURRENT_20MA                UINT8_C(1)
#define FL3194_OP1_CURRENT_30MA                UINT8_C(2)
#define FL3194_OP1_CURRENT_40MA                UINT8_C(3)

#define FL3194_OP2_CURRENT_10MA                UINT8_C(0 << 2)
#define FL3194_OP2_CURRENT_20MA                UINT8_C(1 << 2)
#define FL3194_OP2_CURRENT_30MA                UINT8_C(2 << 2)
#define FL3194_OP2_CURRENT_40MA                UINT8_C(3 << 2)

#define FL3194_OP3_CURRENT_10MA                UINT8_C(0 << 4)
#define FL3194_OP3_CURRENT_20MA                UINT8_C(1 << 4)
#define FL3194_OP3_CURRENT_30MA                UINT8_C(2 << 4)
#define FL3194_OP3_CURRENT_40MA                UINT8_C(3 << 4)

struct fl3194_config {
	struct i2c_dt_spec bus;
};

uint8_t fl3194_led_brightness[FL3194_NUM_LED] ={FL3194_MAX_BRIGHTNESS,
												FL3194_MAX_BRIGHTNESS,
												FL3194_MAX_BRIGHTNESS};

static int fl3194_get_led_reg(uint32_t led, uint8_t *reg)
{
	switch (led) {
	case 0:
		*reg = FL3194_OUT1_REG;
		break;
	case 1:
		*reg = FL3194_OUT2_REG;
		break;
	case 2:
		*reg = FL3194_OUT3_REG;
		break;
	default:
		LOG_ERR("Invalid LED specified");
		return -EINVAL;
	}

	return 0;
}

static int fl3194_get_led_brightness(uint32_t led, uint8_t *value)
{
	if(led >= FL3194_NUM_LED)
	{
		LOG_ERR("Invalid LED specified");
		return -EINVAL;
	}
	*value = fl3194_led_brightness[led];
	return 0;
}

static int fl3194_led_set_brightness(const struct device *dev, uint32_t led,
				     uint8_t value)
{
	ARG_UNUSED(dev);
	if(led >= FL3194_NUM_LED)
	{
		LOG_ERR("Invalid LED specified");
		return -EINVAL;
	}
	if(value > FL3194_MAX_BRIGHTNESS)
	{
		LOG_ERR("Invalid brightness specified");
		return -EINVAL;
	}
	fl3194_led_brightness[led] = value;
	return 0;
}

static inline int fl3194_led_on(const struct device *dev, uint32_t led)
{
	const struct fl3194_config *config = dev->config;
	int ret;
	uint8_t reg, val;

	ret = fl3194_get_led_reg(led, &reg);
	if (ret) {
		return ret;
	}
	ret = fl3194_get_led_brightness(led, &val);
	if (ret) {
		return ret;
	}
	/* Set OUT Reg to max selected brightness*/
	if (i2c_reg_write_byte_dt(&config->bus, reg, val)
		||	i2c_reg_write_byte_dt(&config->bus,FL3194_UPDATE_REG,FL3194_EXECUTE_CMD)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}

	return 0;
}

static inline int fl3194_led_off(const struct device *dev, uint32_t led)
{
	const struct fl3194_config *config = dev->config;
	int ret;
	uint8_t reg;

	ret = fl3194_get_led_reg(led, &reg);
	if (ret) {
		return ret;
	}
	/* Set OUT Reg to max selected brightness*/
	if (i2c_reg_write_byte_dt(&config->bus, reg, FL3194_MIN_BRIGHTNESS)
		||	i2c_reg_write_byte_dt(&config->bus,FL3194_UPDATE_REG,FL3194_EXECUTE_CMD)) {
		LOG_ERR("LED reg update failed");
		return -EIO;
	}

	return 0;
}

static int fl3194_led_blink(const struct device *dev, uint32_t led,
			    uint32_t delay_on, uint32_t delay_off)
{
	//TODO
	return 0;
}

static int fl3194_led_init(const struct device *dev)
{
	const struct fl3194_config *config = dev->config;
	uint8_t devid=0;
	int ret =0;
	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}
	LOG_INF("Initializing fl3194 led");
	ret = i2c_reg_read_byte_dt(&config->bus,FL3194_ID_REG,&devid);
	if(ret)
	{
		LOG_ERR("Error reading Product ID");
		return ret;
	}
	if(devid != FL3194_PROD_ID)
	{
		LOG_ERR("Unexpected Product ID received");
		return -ENODEV;
	}
	uint8_t operation_mode = FL3194_NORMAL_OPERATION		//Normal operation mode
							| FL3194_SINGLE_MODE			//Single mode
							| FL3194_CURRENT_LVL_MODE;		//Current level mode

	uint8_t outputs_en = FL3194_OUTPUT_1 					//Enable output 1
						| FL3194_OUTPUT_2 					//Enable output 2
						| FL3194_OUTPUT_3 ;				//Enable output 3

	uint8_t max_current = FL3194_OP1_CURRENT_10MA 			//Max current 10 mA for output 1
						| FL3194_OP2_CURRENT_10MA			//Max current 10 mA for output 2
						| FL3194_OP3_CURRENT_10MA;			//Max current 10 mA for output 3
	ret |= i2c_reg_write_byte_dt(&config->bus,FL3194_RESET_REG,FL3194_EXECUTE_CMD);
	ret |= i2c_reg_write_byte_dt(&config->bus,FL3194_OP_CONF_REG,operation_mode);
	ret |= i2c_reg_write_byte_dt(&config->bus,FL3194_OUT_EN_REG,outputs_en);
	ret |= i2c_reg_write_byte_dt(&config->bus,FL3194_CURRENT_REG,max_current);
	ret |= i2c_reg_write_byte_dt(&config->bus,FL3194_HOLD_FCT_REG,0);	//not used in current mode
	if(ret)
	{
		LOG_ERR("Error initializing LED device");
		return -EIO;
	}
	return 0;
}

static const struct fl3194_config fl3194_led_config = {
	.bus = I2C_DT_SPEC_INST_GET(0),
};

static const struct led_driver_api fl3194_led_api = {
	.blink = fl3194_led_blink,
	.set_brightness = fl3194_led_set_brightness,
	.on = fl3194_led_on,
	.off = fl3194_led_off,
};

DEVICE_DT_INST_DEFINE(0, &fl3194_led_init, NULL, NULL,
		      &fl3194_led_config, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
		      &fl3194_led_api);
