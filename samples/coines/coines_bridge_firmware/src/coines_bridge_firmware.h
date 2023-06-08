/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines_bridge_firmware.h
 * @brief   This file contains function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef COINES_BRIDGE_FIRMWARE_H_
#define COINES_BRIDGE_FIRMWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! coines buffer size */
#define COM_READ_BUFF                     2060

/*! coines maximum sensor for streamin */
#define STREAM_MAX_COUNT_T                3

/*! coines maximum interrupt line count */
#define COINES_MAX_INT_LINE               2

/*! coines maximum blocks to read in streaming */
#define COINES_MAX_BLOCK                  10

/*! coines maximum start address to read in streaming */
#define COINES_MAX_ADDR                   COINES_MAX_BLOCK

/*! coines maximum data bytes to read in streaming */
#define COINES_MAX_DATA_BYTES             COINES_MAX_BLOCK

/*! coines streaming hardware pin state mask */
#define COINES_STREAM_INT_PIN_MASK        0x0F

/*! coines streaming hardware pin state mask */
#define COINES_STREAM_INT_PIN_STATE_MASK  0x10

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * @brief timestamp config
 */
enum coines_stream_timestamp {
    COINES_STREAM_NO_TIMESTAMP = 0, /*< no timestamp */
    COINES_STREAM_USE_TIMESTAMP = 1 /*< Timestamp is present */
};

/*!
 * @brief streaming mode
 */
enum coines_stream_mode {
    COINES_STREAM_MODE_INTERRUPT, /*< interrupt*/
    COINES_STREAM_MODE_POLLING, /*< polling*/
    COINES_STREAM_MODE_FIFO_POLLING /*<fifo polling*/
};

struct coines_stream_settings
{
    uint32_t gst_period_us; /*< Global Sampling Timer period (in polling mode, all sensor sampling periods are multiple
                             * of this period)*/
    enum coines_stream_timestamp ts_mode; /*< ts mode */
    enum coines_stream_mode stream_mode; /*< stream mode */
};

/*!
 * @brief streaming clear on write settings
 */
struct coines_stream_clear_on_write
{
    uint8_t dummy_byte; /*< dummy byte count */
    uint8_t startaddress; /*< starting address */
    uint16_t num_bytes_to_clear; /* < No. of bytes to clear */
    uint8_t data_buf[255]; /*< data chunks */
};

/*!
 * @brief polling streaming config settings
 */
struct coines_poll_streaming
{
    uint8_t sensor_id; /*< streaming sensor id */
    uint8_t timestamp;  /*< 1- enable /0- disable time stamp for corresponding sensor */
    enum coines_sensor_intf intf; /*< Sensor Interface */
    uint8_t intf_bus; /*< Bus Interface */
    uint8_t intf_addr; /*< I2C address/SPI CS pin */
    uint16_t sampling_time; /*< Sampling time */
    enum coines_sampling_unit sampling_units; /*< micro second / milli second - Sampling unit */
    uint16_t no_of_blocks; /*< Number of blocks to read*/
    uint8_t reg_start_addr[COINES_MAX_ADDR]; /*< Register start address */
    uint8_t no_of_data_bytes[COINES_MAX_DATA_BYTES]; /*< Number of data bytes */
    uint8_t spi_type; /*< spi type */
    uint8_t clear_on_write; /*< clear on write */
    struct coines_stream_clear_on_write clear_on_write_config;
    uint8_t intline_count; /*< interrupt line count */
    uint8_t intline_info[COINES_MAX_INT_LINE]; /*< interrupt line number */
    uint8_t DATA_intline[COINES_MAX_INT_LINE]; /*< interrupt line state */
    uint64_t packet_timestamp_us; /*< packet timestamp */
    uint32_t sampling_period_us;
    uint32_t gst_ticks_counter;
    uint32_t gst_multiplier;
};

/*!
 * @brief interrupt streaming config settings
 */
struct coines_int_streaming
{
    uint8_t sensor_id; /*< streaming sensor id */
    uint8_t timestamp;
    enum coines_sensor_intf intf; /*< Sensor Interface SPI/I2C */
    uint8_t intf_bus; /*< Bus Interface */
    uint8_t intf_addr; /*< I2C address/SPI CS pin */
    uint8_t int_pin; /*< data ready initialization */
    uint16_t no_of_blocks; /*< Number of blocks to read*/
    uint8_t reg_start_addr[COINES_MAX_ADDR]; /*< Register start address */
    uint8_t no_of_data_bytes[COINES_MAX_DATA_BYTES]; /*< Number of data bytes */
    uint8_t spi_type; /*< spi type */
    uint8_t clear_on_write; /*< clear on write */
    uint8_t hw_pin_state; /*< hardware pin state active low/high */
    struct coines_stream_clear_on_write clear_on_write_config;
    uint8_t intline_count; /*< intterupt line count */
    uint8_t intline_info[COINES_MAX_INT_LINE]; /*< interrupt line number */
    uint8_t DATA_intline[COINES_MAX_INT_LINE]; /*< interrupt line state */
    uint64_t packet_timestamp_us; /*< packet timestamp */
};

union coines_streaming
{
    struct coines_poll_streaming poll_config;
    struct coines_int_streaming int_config;
};

#ifdef __cplusplus
}
#endif

#endif /* COINES_BRIDGE_FIRMWARE_H_ */
