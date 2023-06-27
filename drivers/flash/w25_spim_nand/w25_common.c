/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    w25_common.c
 * @date    Apr 30, 2021
 * @brief   W25 flash chip common APIs
 */

/*!
 * @addtogroup
 * @brief
 * @{*/

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
/**********************************************************************************/
/* own header files */
/**********************************************************************************/
#include "w25_common.h"
#include "w25n01gw.h"
#include "w25n02jw.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define SPI_TX_SIZE  (256 + 1)

/*! Opcode to read the Status Register */
#define W25_CMD_RD_REG                0x0F
/*! Command length for reading from the registers */
#define W25_RD_SREG_CMD_LEN           0x01
/*! First two bytes of send buffer will receive nothing.Hence reading from 3rd byte */
#define W25_MIN_RCV_BYTES_LEN         0x03
/*! Opcode to get JEDEC id */
#define W25_CMD_JDEC_ID               0x9F
/*! Opcode to Reset */
#define W25_CMD_RESET                0xFF

/*! Position of BUSY bit of status register-3 */
#define W25_BUSY_STAT                 (1 << 0)

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/*! Holds the initialization status of the module */
w25_nand_error_t w25_init_status = W25_NAND_UNINITIALIZED;

/*! Holds the spi handle */
uint8_t w25_spi_intf_handle = 0;

#define SPI_OPERATION (SPI_OP_MODE_MASTER | SPI_MODE_CPOL | \
			SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE | \
			SPI_TRANSFER_MSB)
#define SPI_DELAY 2U
struct spi_dt_spec spi_spec = SPI_DT_SPEC_GET(DT_NODELABEL(nandflashspi),SPI_OPERATION,SPI_DELAY);
static uint8_t flash_spi_tx_buff[SPI_TX_SIZE];

/**********************************************************************************/
/* function prototypes */
/**********************************************************************************/
void w25_gpio_pin_output_set(uint8_t pin);
uint8_t w25_spi_init(void);
static void w25_device_reset(void);
static uint8_t w25_read_reg(w25_reg_t reg);
uint8_t w25_spi_rx_tx(uint8_t spi_entity, uint8_t address, uint8_t* tx_buffer,
                      uint16_t tx_count,
                      uint8_t* rx_buffer, uint16_t rx_count);

/*!
 * @brief This function initializes the w25 driver
 *
 */
w25_nand_error_t w25_init(uint16_t *device_id)
{
    w25_nand_error_t ret_code = W25_NAND_INITIALIZATION_FAILED;
    w25_deviceinfo_t info;
    int timeout =100;

    /* If already initialized, return initiliazation success */
    if (w25_spi_intf_handle)
    {
        return W25_NAND_INITIALIZED;
    }

        w25_spi_intf_handle = w25_spi_init();

    if (!w25_spi_intf_handle)
    {
        ret_code = W25_NAND_ERROR;
        return ret_code;
    }

    /*Reset the chip*/
    w25_device_reset();
    /*Wait until all the device is powered up */
    while ((w25_read_reg(W25_STATUS_REG_ADDR)) & W25_BUSY_STAT)
    {
        if(timeout-- <=0) //prevent hanging, in case of bugs
            break;
    }

    w25_get_manufacture_and_devid(&info);

    if (((info.device_id == W25N01GW_DEVICE_ID) || (info.device_id == W25M02GW_DEVICE_ID)) &&
        (info.mfg_id == W25_MANUFACTURER_ID))
    {
        w25n01gw_init_protect_reg();
        w25n01gw_init_config_reg();

        *device_id = info.device_id;
        w25_init_status = W25_NAND_INITIALIZED;
        ret_code = W25_NAND_INITIALIZED;

    }
	else if (((info.device_id == W25N02JW_DEVICE_ID) || 
			(info.device_id == W25N02KW_DEVICE_ID)) && 
			(info.mfg_id == W25_MANUFACTURER_ID))
    {
        w25n02jw_init_protect_reg();
        w25n02jw_init_config_reg();

        *device_id = info.device_id;
        w25_init_status = W25_NAND_INITIALIZED;
        ret_code = W25_NAND_INITIALIZED;
    }
    else
    {
        *device_id = 0;
        ret_code = W25_NAND_INITIALIZATION_FAILED;
    }

    return ret_code;
}

/*!
 * @brief       This function gives the manufacture id and device id
 */
void w25_get_manufacture_and_devid(w25_deviceinfo_t* info)
{
    uint8_t dummy_byte = 0;
    uint8_t recv_buff[5];

    memset(recv_buff, 0, 5);

    /*Gets the manufacture id and device id
     * mfg id and device id together is only 3 bytes.
     * However 5 bytes are read because first two bytes received are not valid data.This is a deviation from the datasheet specification
     * */
    (void)w25_spi_rx_tx(w25_spi_intf_handle, W25_CMD_JDEC_ID, &dummy_byte, W25_RD_SREG_CMD_LEN, &recv_buff[0], 5);

    /*Update the structure*/
    info->mfg_id = recv_buff[2];
    info->device_id = recv_buff[3] << 8 | recv_buff[4];

}
/*!
 * @brief       This function resets the chip
 *
 */
static void w25_device_reset(void)
{
    (void)w25_spi_rx_tx(w25_spi_intf_handle, W25_CMD_RESET, NULL, 0, NULL, 0);
}

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle (1 on success, 0 on error)
 */

uint8_t w25_spi_init()
{
    int ret=0;

	struct gpio_dt_spec ios_hold = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(nandflashspi),hold_gpios,{});
    if(ios_hold.port)
    {
    	ret |= gpio_pin_configure_dt(&ios_hold, GPIO_OUTPUT_ACTIVE); 
    }

	struct gpio_dt_spec ios_wp = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(nandflashspi),wp_gpios,{});
    if(ios_wp.port)
    {
    	ret |= gpio_pin_configure_dt(&ios_wp, GPIO_OUTPUT_ACTIVE); 
    }


    return (ret==0)?1:0;
}

static uint8_t w25_read_reg(w25_reg_t reg)
{
    uint8_t recv_buff[W25_MIN_RCV_BYTES_LEN] = { 0 };

    (void)w25_spi_rx_tx(w25_spi_intf_handle,
    W25_CMD_RD_REG,
                  (uint8_t *)&reg,
                  W25_RD_SREG_CMD_LEN,
                  &recv_buff[0],
                  W25_MIN_RCV_BYTES_LEN);

    return recv_buff[W25_MIN_RCV_BYTES_LEN - 1];

}

/*!
 * @brief       This function sends and receives the data via SPI
 *
 * @param[in]   spi_entity : SPI handle
 * @param[in]   address : reg address
 * @param[in]   tx_buffer : transmission buffer
 * @param[in]   tx_count : transmission buffer size
 * @param[in]   rx_buffer : receiver buffer
 * @param[in]   rx_count : receiver buffer size
 *
 */

uint8_t w25_spi_rx_tx(uint8_t spi_entity, uint8_t address, uint8_t* tx_buffer, uint16_t tx_count, uint8_t* rx_buffer, uint16_t rx_count)
{
    (void)spi_entity;
	int result;
    flash_spi_tx_buff[0] = address;
    memcpy(&flash_spi_tx_buff[1], tx_buffer, tx_count);

	const struct spi_buf tx_buf = {
		.buf = flash_spi_tx_buff,
		.len = tx_count+1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
			.buf = rx_buffer,
			.len = rx_count,
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	result = spi_transceive(spi_spec.bus, &spi_spec.config,&tx, &rx);
	if (result) {
		return result;
	}

	return 0;
}