/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * Copyright (c) 2022 Bosch Sensortec GmbH
 * adaption of original NUS sample code from Nordic for providing NUS service
 * to COINES applications
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) adapter for COINES
 */

#include <zephyr/types.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/device.h>

#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME nus_adapter
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define BLE_WRITE_THREAD_PRIORITY 7
void ble_write_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_STACK_DEFINE(ble_write_thread_stack_area, CONFIG_BT_NUS_THREAD_STACK_SIZE);
struct k_thread ble_write_thread_data;
k_tid_t ble_write_thread_id;
								 
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

void hts_init(void);
void hts_proc(void);
//K_SEM_DEFINE(ble_init_ok, 0, 1); //not reqd if thread started in init
K_PIPE_DEFINE(nus_recv_pipe, CONFIG_NUS_READ_BUF_SIZE, 4);
static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static bool ad_active =false;

/*HTS elements added to advertisment structure*/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		  BT_UUID_16_ENCODE(BT_UUID_HTS_VAL),
		  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),
		  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)
		),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};


static void connected(struct bt_conn *conn, uint8_t err)
{

	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);	//log_strdup(addr) no longer required

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason); //log_strdup(addr) no longer required

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	ad_active = false; /*restart advt after disconnection*/
}

int ble_service_ad_proc()
{
	if(current_conn != NULL || ad_active)
		return 0;
	int cerr = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (cerr) {
		LOG_ERR("Advertising failed to (re)start (err %d)", cerr);
	}
	ad_active =true;
	return 0;
}

bool ble_service_nus_connected(void)
{
	return current_conn != NULL;
}

static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
};


static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int    rc;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	size_t bytes_written =0;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);	//log_strdup(addr) no longer required
 	/* send data to the consumers */
	rc = k_pipe_put(&nus_recv_pipe, (void *)data, len, &bytes_written,0, K_NO_WAIT);

	if (rc < 0) {
		LOG_ERR("Error %d writing to nus pipe",rc);
	} else if (bytes_written < len) {
		LOG_ERR("%d bytes lost while writing to nus pipe",len-bytes_written);
	} else {
		/* All data sent */
	}		
#ifdef	CONFIG_BT_NUS_UART_ECHO
		bt_nus_send(NULL, data, len);/*ECHO back data from UART*/
#endif
}

//internal function used by coines_comm_intf.c
uint16_t ble_service_nus_bytes_available(void)
{
	return k_pipe_read_avail(&nus_recv_pipe);
}

//internal function used by coines_comm_intf.c
uint16_t ble_service_nus_read(void *buffer, uint16_t len)
{
	int    rc;
	size_t bytes_read =0;
	rc = k_pipe_get(&nus_recv_pipe, buffer, len, &bytes_read,0, K_NO_WAIT);
	if (rc < 0) {
		LOG_ERR("Error %d reading from nus pipe",rc);
	}
	return bytes_read;
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

//to set name and power to something other than default values
int ble_service_config(const char *bt_dev_name, int power)
{
	ARG_UNUSED(power);
	int rc = bt_set_name(bt_dev_name);	
	return rc;
}
//internal function used by Coines_comm_intf.c
int ble_service_init(void)
{
	int err = 0;

	bt_conn_cb_register(&conn_callbacks);


	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("BT Enable failed (err %d)", err);
		return err;
	}
	//k_sem_give(&ble_init_ok);//not reqd if thread starting in init
	ble_write_thread_id = k_thread_create(&ble_write_thread_data, ble_write_thread_stack_area,
                                 K_THREAD_STACK_SIZEOF(ble_write_thread_stack_area),
                                 ble_write_thread,
                                 NULL, NULL, NULL,
                                 BLE_WRITE_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize NUS UART service (err: %d)", err);
		return err;
	}

	return ble_service_ad_proc();
}
//internal function used by Coines_comm_intf.c
int ble_service_nus_write(uint8_t *data,int len)
{
	return bt_nus_send(NULL, data, len);
}

void ble_write_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	/* Don't go any further until BLE is initialized */
	//not reqd if thread starting in init
	//k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		k_sleep(K_MSEC(CONFIG_HTS_PROC_RATE_MS));
		hts_proc();
		ble_service_ad_proc();
	}
}

//alternative to doing thred create in the init func. Not done like this to have
//more control at COINES level
// K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
// 		NULL, PRIORITY, 0, 0);
	

