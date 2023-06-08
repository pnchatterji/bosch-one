/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "coines_bridge_firmware.h"
#include "coines_bridge_client.h"
#include "decoder.h"
#include "stream.h"

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define MAX_HEADER_LEN  UINT8_C(4)

#define TIMESTAMP_SIZE  UINT8_C(6)

#define RSP_OVERHEAD    (MAX_HEADER_LEN + TIMESTAMP_SIZE)

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
//extern uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX];
//TODO: not available in Z-COINES. It is used at only one place (See TODO comment in line1411).

/*! Buffer used to hold the streaming response data */
uint8_t streaming_resp_buff[COM_READ_BUFF_SIZE];

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/*! Holds streaming mode & timestamp */
static struct coines_stream_settings stream_config =
{ .gst_period_us = 0, .ts_mode = COINES_STREAM_NO_TIMESTAMP, .stream_mode = COINES_STREAM_MODE_POLLING };

/*! Holds streaming config */
static union coines_streaming streaming[STREAM_MAX_COUNT_T];

/*! Hold streaming sensor count */
static uint8_t stream_count = 0;

/*! LED timeouts for different condition */
static uint32_t life_led_error_period = 500;
static uint32_t life_led_normal_period = 2500;
static uint32_t life_led_blink_period = 50;

/*! Updated when soft reset is triggered */
static bool soft_reset_triggered = false;

/*! Updated on polling timer0 expire */
static volatile uint8_t poll_stream_triggered = 0;

/*! Updated when interrupt is triggered */
static volatile uint8_t int_stream_triggered = 0;

/*! Holds active interrupt pin number */
static volatile uint32_t active_pins = 0;

/*! Holds the interrupt line state */
static uint8_t stream_feature_line_state[STREAM_MAX_IO_COUNT];

/*! Holds the interrupt timestamp in nanosecond */
static volatile uint64_t timestamp_ns = 0;

/*! Buffer used to hold the read data */
static uint8_t packet[COM_READ_BUFF_SIZE] = { 0 };

/*! Buffer used to hold the write data */
static uint8_t resp_buff[COM_READ_BUFF_SIZE + RSP_OVERHEAD] = { 0 };

//TODO: changed to USB for testing
//static enum coines_comm_intf comm_intf = COINES_COMM_INTF_BLE;
static enum coines_comm_intf comm_intf = COINES_COMM_INTF_USB;


static bool error = false;

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/
static int16_t echo_callback(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t get_board_info_callback(uint8_t cmd,
                                       uint8_t *payload,
                                       uint16_t payload_length,
                                       uint8_t *resp,
                                       uint16_t *resp_length);
static int16_t set_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length);
static int16_t get_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length);
static int16_t set_vdd_vddio(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t spi_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static int16_t spi_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t i2c_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static int16_t i2c_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t i2c_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t i2c_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t spi_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length);
static int16_t spi_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length);
static int16_t poll_streaming_common(uint8_t cmd,
                                     uint8_t *payload,
                                     uint16_t payload_length,
                                     uint8_t *resp,
                                     uint16_t *resp_length);
static int16_t poll_streaming_config(uint8_t cmd,
                                     uint8_t *payload,
                                     uint16_t payload_length,
                                     uint8_t *resp,
                                     uint16_t *resp_length);
static int16_t int_streaming_config(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length);
static int16_t streaming_start_stop(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length);
static int16_t soft_reset(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length);
static void send_streaming_response(void);
static int16_t streaming_start(void);
static int16_t streaming_stop(void);
void polling_timer_handler(void);
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity);
static void read_interrupt_line_state(uint8_t multi_io);
static void send_int_stream_response(void);
static void send_poll_stream_response(void);
static bool is_stream_sensor_configured(uint8_t sensor_id);

int8_t (*sensor_read)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);
int8_t (*sensor_write)(uint8_t, uint8_t, uint8_t, uint8_t*, uint16_t);

/**********************************************************************************/
/* functions */
/**********************************************************************************/
int main(void)
{
    uint16_t packet_length = 0;
    uint16_t resp_length;
    uint32_t bytes_available = 0;
    uint32_t blink_on = 0, blink_off = 0;
    bool led_state = false;
    struct coines_cbt cbt = { 0 };
    enum coines_led led = COINES_LED_GREEN;
    int8_t cmd_resp = COINES_SUCCESS;
    bool intf_connected = false;

    coines_open_comm_intf(comm_intf, NULL); /* Wait here till USB is connected */

    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF); /* Turn off the LED to indicate that the board is connected */
    cbt.cmd_callback[COINES_CMD_ID_ECHO] = (coines_cmd_callback)echo_callback;
    cbt.cmd_callback[COINES_CMD_ID_GET_BOARD_INFO] = (coines_cmd_callback)get_board_info_callback;
    cbt.cmd_callback[COINES_CMD_ID_SET_PIN] = (coines_cmd_callback)set_pin_callback;
    cbt.cmd_callback[COINES_CMD_ID_GET_PIN] = (coines_cmd_callback)get_pin_callback;
    cbt.cmd_callback[COINES_CMD_ID_SET_VDD_VDDIO] = (coines_cmd_callback)set_vdd_vddio;
    cbt.cmd_callback[COINES_CMD_ID_SPI_CONFIG] = (coines_cmd_callback)spi_config;
    cbt.cmd_callback[COINES_CMD_ID_SPI_DECONFIG] = (coines_cmd_callback)spi_deconfig;
    cbt.cmd_callback[COINES_CMD_ID_I2C_CONFIG] = (coines_cmd_callback)i2c_config;
    cbt.cmd_callback[COINES_CMD_ID_I2C_DECONFIG] = (coines_cmd_callback)i2c_deconfig;
    cbt.cmd_callback[COINES_CMD_ID_I2C_WRITE_REG] = (coines_cmd_callback)i2c_write_reg;
    cbt.cmd_callback[COINES_CMD_ID_I2C_READ_REG] = (coines_cmd_callback)i2c_read_reg;
    cbt.cmd_callback[COINES_CMD_ID_SPI_WRITE_REG] = (coines_cmd_callback)spi_write_reg;
    cbt.cmd_callback[COINES_CMD_ID_SPI_READ_REG] = (coines_cmd_callback)spi_read_reg;
    cbt.cmd_callback[COINES_CMD_ID_POLL_STREAM_COMMON] = (coines_cmd_callback)poll_streaming_common;
    cbt.cmd_callback[COINES_CMD_ID_POLL_STREAM_CONFIG] = (coines_cmd_callback)poll_streaming_config;
    cbt.cmd_callback[COINES_CMD_ID_INT_STREAM_CONFIG] = (coines_cmd_callback)int_streaming_config;
    cbt.cmd_callback[COINES_CMD_ID_STREAM_START_STOP] = (coines_cmd_callback)streaming_start_stop;
    cbt.cmd_callback[COINES_CMD_ID_SOFT_RESET] = (coines_cmd_callback)soft_reset;

    while (1)
    {
        if (coines_intf_connected(COINES_COMM_INTF_USB))
        {
            comm_intf = COINES_COMM_INTF_USB;
            led = COINES_LED_GREEN;
            intf_connected = true;
        }
        else if (coines_intf_connected(COINES_COMM_INTF_BLE))
        {
            comm_intf = COINES_COMM_INTF_BLE;
            led = COINES_LED_BLUE;
            intf_connected = true;
        }

        if (intf_connected)
        {
            bytes_available = coines_intf_available(comm_intf);
            if (bytes_available >= 3)
            {
                coines_set_led(led, COINES_LED_STATE_ON); /* Turn on the LED to indicate command is being processed */

                coines_read_intf(comm_intf, packet, 3);
                if (packet[COINES_PROTO_HEADER_POS] == COINES_CMD_HEADER)
                {
                    packet_length = 0;
                    memcpy(&packet_length, &packet[COINES_PROTO_LENGTH_POS], 2);
                    if (packet_length)
                    {
                        while (coines_intf_available(comm_intf) < (packet_length - 3))
                            ;

                        coines_read_intf(comm_intf, &packet[COINES_PROTO_CMD_POS], packet_length - 3);
                        resp_length = 0;

                        if (packet_length >= 3)
                        {
                            cmd_resp = coines_process_packet(packet, packet_length, resp_buff, &resp_length, &cbt);
                            if (cmd_resp != COINES_SUCCESS)
                            {
                                resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_NOK_HEADER;
                                resp_buff[COINES_PROTO_CMD_POS] = packet[COINES_PROTO_CMD_POS];
                                resp_length = 5;
                                memcpy(&resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                                resp_buff[COINES_PROTO_PAYLOAD_POS] = (uint8_t)cmd_resp;
                                error = true;
                                life_led_error_period = 1000;
                            }
                        }

                        if (resp_length)
                        {
                            coines_write_intf(comm_intf, resp_buff, resp_length);
                            resp_length = 0;
                        }
                    }
                }
                else if ((packet[DECODER_BYTEPOS_HEADER] == DECODER_HEADER_VALUE) ||
                         (packet[DECODER_BYTEPOS_HEADER] == UINT8_C(0x55)))
                {
                    coines_set_led(led, COINES_LED_STATE_ON);
                    packet_length = packet[DECODER_BYTEPOS_SIZE];
                    if (packet_length)
                    {
                        while (coines_intf_available(comm_intf) < (packet_length - 3))
                            ;

                        coines_read_intf(comm_intf, &packet[COINES_PROTO_CMD_POS], packet_length - 3);

                        decoder_process_cmds(packet);
                    }

                    coines_set_led(led, COINES_LED_STATE_OFF);
                }
                else
                {
                    error = true;
                    life_led_error_period = 125;
                }

                coines_set_led(led, COINES_LED_STATE_OFF); /* Turn off the LED to indicate processing is done */
            }

            if (blink_on <= coines_get_millis())
            {
                if (error)
                {
                    blink_on = coines_get_millis() + life_led_error_period;
                }
                else
                {
                    blink_on = coines_get_millis() + life_led_normal_period;
                }

                blink_off = coines_get_millis() + life_led_blink_period;
                coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON); /* Turn on the LED to indicate that the board is
                                                                      * connected */
                led_state = true;
            }

            if (led_state && (blink_off <= coines_get_millis()))
            {
                coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF); /* Turn off the LED to indicate processing is done
                                                                      */
                led_state = false;
            }

            send_streaming_response(); /* Look for configured streaming and send the streaming response */
            send_old_protocol_streaming_response(); /* Look for configured streaming in the old-protocol and send the
                                                 * response */

            if (soft_reset_triggered)
            {
                coines_close_comm_intf(comm_intf, NULL);
                coines_soft_reset();
            }
        }
    }
}

/*!
 *  @brief This API is used to transmit the data to PC.
 *
 */
void decoder_write_resp(void *buff, uint16_t len)
{
    coines_write_intf(comm_intf, buff, len);
}

/*!
 *  @brief This API is used to echo the received data.
 *
 */
static int16_t echo_callback(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = payload_length + 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    memcpy(&resp[COINES_PROTO_PAYLOAD_POS], payload, payload_length);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to get the board information.
 *
 */
static int16_t get_board_info_callback(uint8_t cmd,
                                       uint8_t *payload,
                                       uint16_t payload_length,
                                       uint8_t *resp,
                                       uint16_t *resp_length)
{
    (void)payload_length;
    struct coines_board_info board_info = { 0 };
    int16_t rslt;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    rslt = coines_get_board_info(&board_info);
    if (rslt == COINES_SUCCESS)
    {
        memcpy(&resp[COINES_PROTO_PAYLOAD_POS], &board_info.hardware_id, 2);
        memcpy(&resp[COINES_PROTO_PAYLOAD_POS + 2], &board_info.software_id, 2);
        memcpy(&resp[COINES_PROTO_PAYLOAD_POS + 4], &board_info.board, 1);
        memcpy(&resp[COINES_PROTO_PAYLOAD_POS + 5], &board_info.shuttle_id, 2);
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + 7;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return rslt;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 */
static int16_t set_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length)
{
    enum coines_multi_io_pin pin_number;
    enum coines_pin_direction direction;
    enum coines_pin_value pin_value;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 3)
    {
        pin_number = (enum coines_multi_io_pin)payload[0];
        direction = (enum coines_pin_direction)payload[1];
        pin_value = (enum coines_pin_value)payload[2];
        rslt = coines_set_pin_config(pin_number, direction, pin_value);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return rslt;
}

/*!
 *  @brief This API is used to get the pin direction and pin state.
 *
 */
static int16_t get_pin_callback(uint8_t cmd,
                                uint8_t *payload,
                                uint16_t payload_length,
                                uint8_t *resp,
                                uint16_t *resp_length)
{
    enum coines_multi_io_pin pin_number;
    enum coines_pin_direction direction;
    enum coines_pin_value pin_value;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 3)
    {
        pin_number = (enum coines_multi_io_pin)payload[0];
        rslt = coines_get_pin_config(pin_number, &direction, &pin_value);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + 3;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    resp[COINES_PROTO_PAYLOAD_POS] = pin_number;
    resp[COINES_PROTO_PAYLOAD_POS + 1] = direction;
    resp[COINES_PROTO_PAYLOAD_POS + 2] = pin_value;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 */
static int16_t set_vdd_vddio(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint16_t vdd = 0, vddio = 0;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 4)
    {
        memcpy(&vdd, payload, 2);
        memcpy(&vddio, &payload[2], 2);
        rslt = coines_set_shuttleboard_vdd_vddio_config(vdd, vddio);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the spi bus.
 *
 */
static int16_t spi_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    enum coines_spi_bus bus;
    enum coines_spi_speed speed;
    enum coines_spi_mode mode;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 3)
    {
        bus = (enum coines_spi_bus)payload[0];
        speed = (enum coines_spi_speed)payload[1];
        mode = (enum coines_spi_mode)payload[2];
        rslt = coines_config_spi_bus(bus, speed, mode);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to de-configure the spi bus.
 *
 */
static int16_t spi_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 1)
    {
        bus = (enum coines_spi_bus)payload[0];
        rslt = coines_deconfig_spi_bus(bus);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the i2c bus.
 *
 */
static int16_t i2c_config(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    enum coines_i2c_bus bus;
    enum coines_i2c_mode mode;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    if (payload_length >= 2)
    {
        bus = (enum coines_i2c_bus)payload[0];
        mode = (enum coines_i2c_mode)payload[1];
        rslt = coines_config_i2c_bus(bus, mode);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to de-configure the i2c bus.
 *
 */
static int16_t i2c_deconfig(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 1)
    {
        bus = (enum coines_i2c_bus)payload[0];
        rslt = coines_deconfig_i2c_bus(bus);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 */
static int16_t i2c_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_write_i2c(bus, dev_addr, reg_addr, &payload[5], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 */
static int16_t i2c_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_i2c_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_i2c_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_read_i2c(bus, dev_addr, reg_addr, &resp[COINES_PROTO_PAYLOAD_POS], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to write the data in SPI communication.
 *
 */
static int16_t spi_write_reg(uint8_t cmd,
                             uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *resp,
                             uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 6)
    {
        bus = (enum coines_spi_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_write_spi(bus, dev_addr, reg_addr, &payload[5], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 */
static int16_t spi_read_reg(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp,
                            uint16_t *resp_length)
{
    uint8_t dev_addr, reg_addr;
    uint16_t data_length;
    enum coines_spi_bus bus;
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 5)
    {
        bus = (enum coines_spi_bus)payload[0];
        dev_addr = payload[1];
        reg_addr = payload[2];
        memcpy(&data_length, &payload[3], 2);
        rslt = coines_read_spi(bus, dev_addr, reg_addr, &resp[COINES_PROTO_PAYLOAD_POS], data_length);
        if (rslt != COINES_SUCCESS)
        {
            return rslt;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + data_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to trigger the softreset.
 *
 */
static int16_t soft_reset(uint8_t cmd, uint8_t *payload, uint16_t payload_length, uint8_t *resp, uint16_t *resp_length)
{
    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 4)
    {
        soft_reset_triggered = true;
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;

}

/*!
 * @brief This API is used to get and configure polling stream settings.
 *
 */
static int16_t  poll_streaming_config(uint8_t cmd,
                                      uint8_t *payload,
                                      uint16_t payload_length,
                                      uint8_t *resp,
                                      uint16_t *resp_length)
{
    uint8_t read_index = 0;
    uint16_t raw_time;
    union coines_streaming *stream_p = &streaming[stream_count];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 12)
    {
        if (!is_stream_sensor_configured(payload[0]))
        {
            if (stream_count < STREAM_MAX_COUNT_T)
            {
                stream_p->poll_config.sensor_id = payload[read_index++];
                stream_p->poll_config.timestamp = payload[read_index++];
                stream_p->poll_config.intf = (enum coines_sensor_intf)payload[read_index++];
                stream_p->poll_config.intf_bus = (uint8_t)payload[read_index++];
                stream_p->poll_config.intf_addr = (uint8_t)payload[read_index++];
                stream_p->poll_config.sampling_time = (uint16_t)payload[read_index++] << 8; /* sampling_time msb */
                stream_p->poll_config.sampling_time |= (uint16_t)payload[read_index++] & 0xFF; /* sampling_time lsb
                                                                                                  */
                stream_p->poll_config.sampling_units = (enum coines_sampling_unit)payload[read_index++];
                stream_p->poll_config.no_of_blocks = (uint16_t)payload[read_index++] << 8;
                stream_p->poll_config.no_of_blocks |= (uint16_t)payload[read_index++] & 0xFF;
                if (stream_p->poll_config.no_of_blocks > 0)
                {
                    for (uint16_t i = 0 ; i < stream_p->poll_config.no_of_blocks; i++ )
                    {
                        stream_p->poll_config.reg_start_addr[i] = payload[read_index++];
                        stream_p->poll_config.no_of_data_bytes[i] = payload[read_index++];
                    }
                }
                else
                {
                    return COINES_E_STREAM_INVALID_BLOCK_SIZE;
                }

                stream_p->poll_config.spi_type = payload[read_index++];
                stream_p->poll_config.clear_on_write = payload[read_index++];
                if (stream_p->poll_config.clear_on_write)
                {
                    stream_p->poll_config.clear_on_write_config.dummy_byte = payload[read_index++];
                    stream_p->poll_config.clear_on_write_config.startaddress = payload[read_index++];
                    stream_p->poll_config.clear_on_write_config.num_bytes_to_clear = payload[read_index++];
                }

                stream_p->poll_config.intline_count = payload[read_index++];
                if (stream_p->poll_config.intline_count > 0)
                {
                    for (uint8_t i = 0 ; i < stream_p->poll_config.intline_count; i++ )
                    {
                        stream_p->poll_config.intline_info[i] = payload[read_index++];
                    }
                }

                raw_time = stream_p->poll_config.sampling_time;
                if (stream_p->poll_config.sampling_units == (uint8_t)COINES_SAMPLING_TIME_IN_MICRO_SEC)
                {
                    stream_p->poll_config.sampling_period_us = raw_time;
                }
                else
                {
                    stream_p->poll_config.sampling_period_us = raw_time * 1000;
                }

                if (stream_p->poll_config.sampling_period_us <= stream_config.gst_period_us)
                {
                    stream_p->poll_config.gst_multiplier = 1;
                }
                else
                {
                    stream_p->poll_config.gst_multiplier = stream_p->poll_config.sampling_period_us /
                                                           stream_config.gst_period_us;
                }

                stream_p->poll_config.gst_ticks_counter = stream_p->poll_config.gst_multiplier;

                stream_count++;
            }
            else
            {
                return COINES_E_STREAM_CONFIG_MEMORY_FULL;
            }
        }
        else
        {
            return COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    stream_config.stream_mode = COINES_STREAM_MODE_POLLING;
    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to configure global sampling timer period for all polling streaming.
 *
 */
static int16_t poll_streaming_common(uint8_t cmd,
                                     uint8_t *payload,
                                     uint16_t payload_length,
                                     uint8_t *resp,
                                     uint16_t *resp_length)
{
    uint16_t raw_time;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 4)
    {
        raw_time = (payload[1] << 8) | payload[2];
        if (raw_time > 0)
        {
            if (payload[3] == (uint8_t)COINES_SAMPLING_TIME_IN_MICRO_SEC)
            {
                stream_config.gst_period_us = raw_time;
            }
            else if (payload[3] == (uint8_t)COINES_SAMPLING_TIME_IN_MILLI_SEC)
            {
                stream_config.gst_period_us = raw_time * 1000;
            }
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;

}

/*!
 * @brief This API is used to start/stop the streaming.
 *
 */
static int16_t streaming_start_stop(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length)
{
    int16_t rslt = COINES_SUCCESS;

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 1)
    {
        if (stream_count > 0)
        {
            if (payload[0] == 0x00)
            {
                rslt = streaming_stop();
            }
            else if (payload[0] == 0xFF)
            {
                rslt = streaming_start();
            }
        }
        else
        {
            return COINES_E_STREAM_NOT_CONFIGURED;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    if (rslt != COINES_SUCCESS)
    {
        return rslt;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4 + payload_length;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;
    memcpy(&resp[COINES_PROTO_PAYLOAD_POS], payload, payload_length);

    return rslt;
}

/*!
 * @brief This API will be called on polling timer expire.
 *
 */
void polling_timer_handler(void)
{
    if (poll_stream_triggered == 0)
    {
        poll_stream_triggered = 1;
    }
}

/*!
 * @brief This API is used to start the streaming.
 *
 */
static int16_t streaming_start(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    union coines_streaming *stream_p;
    enum coines_pin_interrupt_mode interrupt_mode;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];
        if (stream_config.stream_mode == COINES_STREAM_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
            {
                if (stream_p->poll_config.intline_info[int_line] & COINES_STREAM_INT_PIN_STATE_MASK) /* Active high */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
                }
                else/* Active low */
                {
                    interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
                }

                coines_attach_interrupt((enum coines_multi_io_pin)(stream_p->poll_config.intline_info[int_line] &
                                                                COINES_STREAM_INT_PIN_MASK),
                                        handler_feature_event,
                                        interrupt_mode);
            }
            poll_stream_count++;
        }
        else if (stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT)
        {
            if (stream_p->int_config.hw_pin_state)/* Active high */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_RISING_EDGE;
            }
            else/* Active low */
            {
                interrupt_mode = COINES_PIN_INTERRUPT_FALLING_EDGE;
            }

            rslt = coines_attach_timed_interrupt((enum coines_multi_io_pin)stream_p->int_config.int_pin,
                                        handler_drdy_int_event,
                                        interrupt_mode);
        }
    }
    if (poll_stream_count > 0)
    {
        rslt = coines_timer_config(COINES_TIMER_INSTANCE_0, polling_timer_handler);
        if (rslt == COINES_SUCCESS)
        {
            rslt = coines_timer_start(COINES_TIMER_INSTANCE_0, stream_config.gst_period_us);
        }
    }
    return rslt;
}

/*!
 * @brief This API is used to stop the streaming.
 *
 */
static int16_t streaming_stop(void)
{
    int16_t rslt = COINES_SUCCESS;
    uint8_t poll_stream_count = 0;
    union coines_streaming *stream_p;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_p = &streaming[i];

        if (stream_config.stream_mode == COINES_STREAM_MODE_POLLING)
        {
            for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
            {
                coines_detach_interrupt((enum coines_multi_io_pin)(stream_p->poll_config.intline_info[int_line]));
            }
            poll_stream_count++;
        }
        else if (stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT)
        {
            coines_detach_timed_interrupt((enum coines_multi_io_pin)stream_p->int_config.int_pin);
        }
        else /* COINES_STREAM_MODE_FIFO_POLLING */
        {
            /* TODO */
        }
        /* Clear the streaming configuration */
        memset(&streaming[i],0,sizeof(union coines_streaming));
    }
    if(poll_stream_count > 0)
    {
        rslt = coines_timer_stop(COINES_TIMER_INSTANCE_0);
    }
    stream_count = 0;
    return rslt;
}

/*!
 * @brief This API is used to configure interface type I2C/SPI.
 *
 */
void config_sensor_intf(enum coines_sensor_intf intf_type)
{
    /*lint -e64*/
    if (intf_type == COINES_SENSOR_INTF_I2C)
    {
        sensor_read = &coines_read_i2c;
        sensor_write = &coines_write_i2c;
    }
    else
    {
        sensor_read = &coines_read_spi;
        sensor_write = &coines_write_spi;
    }

    /*lint +e64*/
}

/*!
 * @brief This API is used to send the polling streaming response
 *
 */
static void send_poll_stream_response(void)
{
    uint16_t sensor_data_pos = 0;
    uint16_t feature_int_info_pos = 0;
    uint16_t resp_length;
    union coines_streaming *stream_p;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;

    if (stream_count > 0)
    {
        for (uint8_t i = 0; i < stream_count; i++)
        {
            stream_p = &streaming[i];
            if (stream_p->poll_config.gst_ticks_counter > 0)
            {
                stream_p->poll_config.gst_ticks_counter--;
            }

            if (stream_p->poll_config.gst_ticks_counter == 0)
            {
                /* reload tick-counter */
                stream_p->poll_config.gst_ticks_counter = stream_p->poll_config.gst_multiplier;

                /* frame polling streaming response */
                streaming_resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
                streaming_resp_buff[COINES_PROTO_CMD_POS] = COINES_READ_SENSOR_DATA;
                streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = stream_p->poll_config.sensor_id;
                sensor_data_pos = COINES_PROTO_PAYLOAD_POS + 1;

                config_sensor_intf(stream_p->poll_config.intf);

                if (stream_p->poll_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    read_mask = 0x80;
                }

                if (stream_p->poll_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                      stream_p->poll_config.intf_addr,
                                      stream_p->poll_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->poll_config.clear_on_write_config.data_buf,
                                      (stream_p->poll_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->poll_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->poll_config.no_of_blocks; index_t++)
                {
                    rslt = sensor_read((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                       stream_p->poll_config.intf_addr,
                                       stream_p->poll_config.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[sensor_data_pos],
                                       stream_p->poll_config.no_of_data_bytes[index_t]);

                    sensor_data_pos += stream_p->poll_config.no_of_data_bytes[index_t];
                }

                if ((stream_p->poll_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write((enum coines_i2c_bus)stream_p->poll_config.intf_bus,
                                       stream_p->poll_config.intf_addr,
                                       stream_p->poll_config.clear_on_write_config.startaddress,
                                       &stream_p->poll_config.clear_on_write_config.data_buf[stream_p->poll_config.
                                                                                             clear_on_write_config.
                                                                                             dummy_byte],
                                       stream_p->poll_config.clear_on_write_config.num_bytes_to_clear);
                }

                feature_int_info_pos = sensor_data_pos;

                /* Get the Interrupt line information and update them in the buffer for transmission */
                for (uint8_t int_line = 0; int_line < stream_p->poll_config.intline_count; int_line++)
                {
                    if (stream_feature_line_state[stream_p->poll_config.intline_info[int_line]] == 1)
                    {
                        stream_p->poll_config.DATA_intline[int_line] = 1;
                        stream_feature_line_state[stream_p->poll_config.intline_info[int_line]] = 0;
                    }
                    else
                    {
                        stream_p->poll_config.DATA_intline[int_line] = 0;
                    }
                }

                memcpy(&streaming_resp_buff[feature_int_info_pos],
                       stream_p->poll_config.DATA_intline,
                       stream_p->poll_config.intline_count);
                feature_int_info_pos += stream_p->poll_config.intline_count;

                if (rslt == COINES_SUCCESS)
                {
                    resp_length = feature_int_info_pos;
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
                else
                {
                    /* streaming block read failed */
                    resp_length = 4;
                    streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = 0xFF; /* set invalid sensor id */
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
            }
        }

        poll_stream_triggered = 0;
    }
}

/*!
 * @brief This API is used to check streaming is configured and triggered
 *
 */
static void send_streaming_response(void)
{
    if ((stream_config.stream_mode == COINES_STREAM_MODE_POLLING) && poll_stream_triggered)
    {
        send_poll_stream_response();
    }
    else if ((stream_config.stream_mode == COINES_STREAM_MODE_INTERRUPT) && int_stream_triggered)
    {
        send_int_stream_response();
    }
    else /* COINES_STREAM_MODE_FIFO_POLLING */
    {
        /* TODO - Implements*/
    }
}

/*!
 *  @brief This API will be called on data ready interrupt
 *
 */
static void handler_drdy_int_event(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;

    read_interrupt_line_state((uint8_t)multiio_pin);
    if (int_stream_triggered == 0)
    {
        timestamp_ns = timestamp;
        int_stream_triggered = 1;
    }
}

/*!
 * @brief This function handles feature event
 */
static void handler_feature_event(uint32_t multiio_pin, uint32_t multiio_pin_polarity)
{
    (void)multiio_pin_polarity;

    if (multiio_pin < COINES_SHUTTLE_PIN_MAX)
    {
        stream_feature_line_state[multiio_pin] = 1;

        /* TODO: use polarity */
    }
}

/*!
 * @brief This API is used to set the interrupt streaming response
 */
static void read_interrupt_line_state(uint8_t multi_io)
{
    union coines_streaming *stream_p;
    enum coines_pin_value gpio_pin_val = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction gpio_dir;

    for (uint8_t int_idx = 0; int_idx < stream_count; int_idx++)
    {
        stream_p = &streaming[int_idx];
        if (stream_p->int_config.int_pin == multi_io)
        {
            for (uint8_t int_line = 0; int_line < stream_p->int_config.intline_count; int_line++)
            {
                /* 
                //TODO: Use of multi_io_map seems to be a bug here, as it is the reverse of what is required
                //In any case, this is not available in Z-COINES
                //Same code also exists in stream.c Fix it there too
                //TODO: find a fix for this if it is an issue
                coines_get_pin_config(
                    (enum coines_multi_io_pin)multi_io_map[stream_p->int_config.intline_info[int_line]],
                    &gpio_dir,
                    &gpio_pin_val);*/
                stream_p->int_config.DATA_intline[int_line] = (uint8_t) gpio_pin_val;
            }

            active_pins |= 1 << multi_io;
        }
    }
}

/*!
 * @brief This API is used to send the interrupt streaming response
 *
 */
static void send_int_stream_response(void)
{
    uint8_t sensor_data_pos = 0;
    uint8_t feature_int_info_pos = 0;
    uint16_t resp_length;
    uint8_t packet_no_pos = 0;
    uint8_t timestamp_pos = 0;
    union coines_streaming *stream_p;
    uint64_t packet_no = 0;
    int8_t ret = COINES_SUCCESS;
    int8_t rslt = COINES_SUCCESS;
    uint8_t read_mask = 0;
    uint64_t timestamp_us = 0;

    if (stream_count > 0)
    {
        for (uint8_t i = 0; i < stream_count; i++)
        {
            stream_p = &streaming[i];
            if ((1 << stream_p->int_config.int_pin) & active_pins)
            {
                active_pins &= ~((uint32_t) (1 << stream_p->int_config.int_pin));
                streaming_resp_buff[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
                streaming_resp_buff[COINES_PROTO_CMD_POS] = COINES_READ_SENSOR_DATA;
                streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = stream_p->int_config.sensor_id;
                packet_no_pos = COINES_PROTO_PAYLOAD_POS + 1;

                /* Packet number 4bytes added for compatibility */
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0xff000000) >> 24);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0x00ff0000) >> 16);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) ((packet_no & 0x0000ff00) >> 8);
                streaming_resp_buff[packet_no_pos++] = (uint8_t) (packet_no & 0x000000ff);
                sensor_data_pos = packet_no_pos;

                config_sensor_intf(stream_p->int_config.intf);

                if (stream_p->int_config.intf == COINES_SENSOR_INTF_SPI)
                {
                    read_mask = 0x80;
                }

                if (stream_p->int_config.clear_on_write)
                {
                    /* Dummy byte information also read based on the input */
                    ret = sensor_read((uint8_t)stream_p->int_config.intf_bus,
                                      stream_p->int_config.intf_addr,
                                      stream_p->int_config.clear_on_write_config.startaddress | read_mask,
                                      stream_p->int_config.clear_on_write_config.data_buf,
                                      (stream_p->int_config.clear_on_write_config.num_bytes_to_clear +
                                       stream_p->int_config.clear_on_write_config.dummy_byte));
                }

                for (uint8_t index_t = 0; index_t < stream_p->int_config.no_of_blocks; index_t++)
                {

                    rslt = sensor_read((uint8_t)stream_p->int_config.intf_bus,
                                       stream_p->int_config.intf_addr,
                                       stream_p->int_config.reg_start_addr[index_t] | read_mask,
                                       &streaming_resp_buff[sensor_data_pos],
                                       stream_p->int_config.no_of_data_bytes[index_t]);

                    sensor_data_pos += stream_p->int_config.no_of_data_bytes[index_t];
                }

                if ((stream_p->int_config.clear_on_write) && (COINES_SUCCESS == ret))
                {
                    /* Ignoring dummy byte if present and writing to register for clearing status register */
                    ret = sensor_write((uint8_t)stream_p->int_config.intf_bus,
                                       stream_p->int_config.intf_addr,
                                       stream_p->int_config.clear_on_write_config.startaddress,
                                       &stream_p->int_config.clear_on_write_config.data_buf[stream_p->int_config.
                                                                                            clear_on_write_config.
                                                                                            dummy_byte],
                                       stream_p->int_config.clear_on_write_config.num_bytes_to_clear);
                }

                /* TOD0 - Add timestamp */
                /* Dummy timestamp 6 bytes */
                timestamp_pos = sensor_data_pos;
                if (stream_p->int_config.timestamp)
                {
                    timestamp_us = timestamp_ns / 1000;
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 40);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 32);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 24);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 16);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us >> 8);
                    streaming_resp_buff[timestamp_pos++] = (uint8_t) (timestamp_us);
                }

                timestamp_pos += 6;

                feature_int_info_pos = timestamp_pos;

                /* Get the Interrupt line information and update them in the buffer for transmission */
                for (uint8_t int_line = 0; int_line < stream_p->int_config.intline_count; int_line++)
                {
                    if (stream_feature_line_state[stream_p->int_config.intline_info[int_line]] == 1)
                    {
                        stream_p->int_config.DATA_intline[int_line] = 1;
                        stream_feature_line_state[stream_p->int_config.intline_info[int_line]] = 0;
                    }
                    else
                    {
                        stream_p->int_config.DATA_intline[int_line] = 0;
                    }
                }

                memcpy(&streaming_resp_buff[feature_int_info_pos],
                       stream_p->int_config.DATA_intline,
                       stream_p->int_config.intline_count);
                feature_int_info_pos += stream_p->int_config.intline_count;

                if (rslt == COINES_SUCCESS)
                {
                    resp_length = feature_int_info_pos;
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
                else
                {
                    /* streaming block read failed */
                    resp_length = 4;
                    streaming_resp_buff[COINES_PROTO_PAYLOAD_POS] = 0xFF; /* set invalid sensor id */
                    memcpy(&streaming_resp_buff[COINES_PROTO_LENGTH_POS], &resp_length, 2);
                    coines_write_intf(comm_intf, streaming_resp_buff, resp_length);
                }
            }
        }

        int_stream_triggered = 0;
    }
}

/*!
 * @brief This API is used to get and configure interrupt stream settings
 *
 */
static int16_t int_streaming_config(uint8_t cmd,
                                    uint8_t *payload,
                                    uint16_t payload_length,
                                    uint8_t *resp,
                                    uint16_t *resp_length)
{
    uint8_t read_index = 0;
    uint16_t i;
    union coines_streaming *stream_p = &streaming[stream_count];

    if ((payload == NULL) || (resp == NULL) || (resp_length == NULL))
    {
        return COINES_E_NULL_PTR;
    }

    if (payload_length >= 12)
    {
        if (stream_count < STREAM_MAX_COUNT_T)
        {
            stream_p->int_config.sensor_id = payload[read_index++];
            stream_p->int_config.timestamp = payload[read_index++];
            stream_p->int_config.intf = (enum coines_sensor_intf)payload[read_index++];
            stream_p->int_config.intf_bus = payload[read_index++];
            stream_p->int_config.intf_addr = payload[read_index++];
            stream_p->int_config.int_pin = payload[read_index++];

            stream_p->int_config.no_of_blocks = payload[read_index++] << 8; /* no_of_blocks byte msb */
            stream_p->int_config.no_of_blocks |= payload[read_index++] & 0xFF; /* no_of_blocks byte lsb */
            if (stream_p->int_config.no_of_blocks > 0)
            {
                for (i = 0 ; i < stream_p->int_config.no_of_blocks; i++ )
                {
                    stream_p->int_config.reg_start_addr[i] = payload[read_index++];
                    stream_p->int_config.no_of_data_bytes[i] = payload[read_index++];
                }
            }
            else
            {
                return COINES_E_STREAM_INVALID_BLOCK_SIZE;
            }

            stream_p->int_config.spi_type = payload[read_index++];
            stream_p->int_config.clear_on_write = payload[read_index++];
            stream_p->int_config.hw_pin_state = payload[read_index++] & 0x01;
            if (stream_p->int_config.clear_on_write)
            {
                stream_p->int_config.clear_on_write_config.dummy_byte = payload[read_index++];
                stream_p->int_config.clear_on_write_config.startaddress = payload[read_index++];
                stream_p->int_config.clear_on_write_config.num_bytes_to_clear = payload[read_index++];
            }

            stream_p->int_config.intline_count = payload[read_index++];
            if (stream_p->int_config.intline_count > 0)
            {
                for (i = 0 ; i < stream_p->int_config.intline_count; i++ )
                {
                    stream_p->int_config.intline_info[i] = payload[read_index++];
                }
            }

            stream_config.stream_mode = COINES_STREAM_MODE_INTERRUPT;
            stream_count++;

        }
        else
        {
            return COINES_E_STREAM_CONFIG_MEMORY_FULL;
        }
    }
    else
    {
        return COINES_E_INVALID_PAYLOAD_LEN;
    }

    resp[COINES_PROTO_HEADER_POS] = COINES_RESP_OK_HEADER;
    *resp_length = 4;
    memcpy(&resp[COINES_PROTO_LENGTH_POS], resp_length, 2);
    resp[COINES_PROTO_CMD_POS] = cmd;

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to check whether streming sensor is already configured or not
 *
 */
static bool is_stream_sensor_configured(uint8_t sensor_id)
{
    uint8_t *stream_sensor_id;

    for (uint8_t i = 0; i < stream_count; i++)
    {
        stream_sensor_id = (uint8_t *)&streaming[i];
        if (*stream_sensor_id == sensor_id)
        {
            return true;
        }
    }

    return false;
}
