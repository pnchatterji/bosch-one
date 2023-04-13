.. _COINES:
# Copyright (c) 2023 Bosch Sensortec GmbH
# SPDX-License-Identifier: Apache-2.0

COINES on Zephyr
################

Overview
********

This is the COINES implementation for the Zephyr RTOS for Application Board 3.0

The implementations exist in this folder. The include folder contains the COINES API header

The COINES_Test application, which is in the samples folder, contains comprehensive tests for 
the complete API. It also serves as a reference application.

An attempt has been made to implement COINES for Zephyr as a 1:1 replacement for standard COINES. However, it has been necessary to take the following deviations:
1.	BLE transmission power cannot be set by the API coines_ble_config(). The tx_power element of the config structure is ignored. Instead, the transmission power has to be set by setting one of the CONFIG_BT_CTLR_TX_PWR_ configuration variables to “y” in conf.prj
Example (in conf.prj)
CONFIG_BT_CTLR_TX_PWR_0=y 	 	#0Db This is default
Other possibilities are CONFIG_BT_CTLR_TX_PWR_MINUS_12, CONFIG_BT_CTLR_TX_PWR_MINUS_16 … upto CONFIG_BT_CTLR_TX_PWR_PLUS_8. Refer the below link for a complete list:
https://docs.zephyrproject.org/3.0.0/reference/kconfig/index-all.html
2.	COINES_SPI_BUS_0 is mapped to NRF SPI2 and COINES_SPI_BUS_0 is mapped to NRF SPI3. This is done to avoid conflicts with I2C0 and 1, which cannot be enabled at the same time with SPI0 and 1. I2C0 and 1 are required for other purposes. All these details are noted in the DTS file.

 NOTE on Board Revisions
 =======================
 There are multiple alternative overlay DTS files for AB3.0 for different project configurations
 Each alternative allocates the I2C and SPI modules for the shuttle board ports 
 in a different way (in terms of pin allocation). Each overlay corresponds to a so-called
 "revision number". The project should be built with the revision number that 
 corresponds to its configuration. Note that "Revision" here actually means 
 "alternative configuration" as per the terminology used in the Zephyr Build System
  
 
 Rev No.		i2c1 					spi1 				spi2
  1		primary_i2c_port		disabled			primary_spi_port [DEFAULT]
  2		primary_i2c_port		disabled			aux_spi_port	
  3		aux_i2c_port			disabled			primary_spi_port	
  4		disabled				primary_spi_port	aux_spi_port
 
 Revision 1 is used by default if no revision is selected in the build. In this configuration
 the primary port of the shuttle can be either i2c or spi. The application code can decide at runtime
 which port is to be used (note that both cannot be used simultaneously, due to the design of the
 sensors. This is compatible with applications written for V2.6 and older of COINES. The downside 
 of this configuration is that the so-called "auxiliary ports" are not available (i.e. the OIS interface
 or external temp sensor on some shuttle boards.)
 Configurations 2, 3 and 4 are useful if access to the auxiliary ports is required. These are accessed by
 using the COINES_SPI_BUS_1 and COINES_I2C_BUS_1 as first argument to the COINES i2c and spi API. using 
 these arguments results in a runtime error in the default configuration, or in a configuration where one
 or the other is not supported.

