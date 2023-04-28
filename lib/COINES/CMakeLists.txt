# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_COINES_TIMED_INTR_ENABLE coines_timed_interrupt.c)
zephyr_library_sources(coines_zephyr.c)
zephyr_library_sources_ifdef(CONFIG_COINES_INTF_BLE_ENABLE hts.c)
zephyr_library_sources_ifdef(CONFIG_COINES_INTF_BLE_ENABLE nus_adapter.c)
zephyr_library_sources_ifdef(CONFIG_COINES_INTF_USB_ENABLE usb_cdc.c)