/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file        w25n01gw_config.h
 *
 * @brief
 *
 */

/*!
* @addtogroup w25n01gw_config
* @brief
* @{*/

#ifndef W25N01GW_CONFIG_H_
#define W25N01GW_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "nrfx_spim.h"
//#include "nrf_gpio.h"
/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Write protect pin of w25n01gw chip */
#define W25N01GW_WRITE_PROTECT_PIN   NRF_GPIO_PIN_MAP(0, 22)
/*! Hold pin of w25n01gw chip */
#define W25N01GW_HOLD_PIN           NRF_GPIO_PIN_MAP(0, 23)
/*! Chip Select pin of w25n01gw chip */
#define W25N01GW_CHIP_SEL_PIN       NRF_GPIO_PIN_MAP(0, 17)

/**********************************************************************************/
/* type definitions */
/**********************************************************************************/

/*!
 * @brief       This function sets the gpio pin given as input
 *
 * @retval      none
 */
#define w25n01gw_gpio_pin_output_set w25_gpio_pin_output_set


/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */
#define w25n01gw_spi_init w25_spi_init

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
#define  w25n01gw_spi_rx_tx w25_spi_rx_tx

#ifdef __cplusplus
}
#endif

#endif /* W25N01GW_CONFIG_H_ */

/** @}*/

