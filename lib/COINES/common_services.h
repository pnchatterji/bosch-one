/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    common_services.h
 * @brief   This file contains internal function prototypes required for COINES Zephyr implementation
 *           which are not exposed in the main API header COINES.h
 *
 */

#ifndef COMMON_SERVICES_
#define COMMON_SERVICES_

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

//internal ble nus defined in nus_adapter.c
int ble_service_nus_write(uint8_t *data,int len);
int ble_service_init(void);
uint16_t ble_service_nus_bytes_available(void);
uint16_t ble_service_nus_read(void *buffer, uint16_t len);
bool ble_service_nus_connected(void);
int ble_service_config(const char *bt_dev_name, int power);
//usb cdc acm services defined in usb_cdc.c
int usb_cdc_write(uint8_t *data,int len);
int usb_cdc_init(void);
uint16_t usb_cdc_bytes_available(void);
uint16_t usb_cdc_read(void *buffer, uint16_t len);
bool usb_cdc_connected();
//timed interrupt feature initialization
int timed_interrupt_init();


#ifdef __cplusplus
}
#endif

#endif /* COMMON_SERVICES_ */

