/** @file
 *  @brief HTS Service Interface
 */

/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 * Get AB3.0 board temperature sensor (TMP112) data via I2C
 * and indicate as HTS service
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>
#include <coines.h>
#define LOG_MODULE_NAME htsbas
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/**
 * @brief: This is the default temperature indicated in HTS if temperature sensor fails
 */ 

#define DEFAULT_TEMPERATURE 20U

static uint8_t indicate_htm;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void htmc_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	indicate_htm = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_INF("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	LOG_INF("Indication complete\n");
	indicating = 0U;
}

/* Health Thermometer Service Declaration */
BT_GATT_SERVICE_DEFINE(hts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(htmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* more optional Characteristics */
);


/**
 * @brief read temperature sensor and return temperature in celcius.
 * In case of error, the buffer is not changed 
 * @param temperature sensend temperature in Celcius (not changed in case of error)
 * @return int16_t 0 on success, else error code
 */

void hts_indicate(void)
{
	/* Temperature measurements simulation */
	if (indicate_htm) {
		static uint8_t htm[5];
		static float temperature = DEFAULT_TEMPERATURE;
		uint32_t mantissa;
		uint8_t exponent;


		if (indicating) {
			return;
		}
		/*if this function fails, default temperature is indicated*/
		coines_read_temp_data(&temperature);

		LOG_INF("temperature is %fC\n", temperature);

		mantissa = (uint32_t)(temperature * 100);
		exponent = (uint8_t)-2;

		htm[0] = 0; /* temperature in celcius */
		sys_put_le24(mantissa, (uint8_t *)&htm[1]);
		htm[4] = exponent;

		ind_params.attr = &hts_svc.attrs[2];
		ind_params.func = indicate_cb;
		ind_params.destroy = indicate_destroy;
		ind_params.data = &htm;
		ind_params.len = sizeof(htm);

		if (bt_gatt_indicate(NULL, &ind_params) == 0) {
			indicating = 1U;
		}
	}
}
static void bas_notify(void)
{
	uint16_t bat_status_mv;
	uint8_t bat_status_percent;
	//uint8_t battery_level = bt_bas_get_battery_level();
	coines_read_bat_status(&bat_status_mv, &bat_status_percent);
	bt_bas_set_battery_level(bat_status_percent);
}

void hts_proc(void)
{
		/* Temperature measurements simulation */
		hts_indicate();

		/* Battery level simulation */
		bas_notify();
}

