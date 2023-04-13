/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines_zephy3.c
 * @brief   Implementation of COINES API on Zephyr for 
 *  1. Bosch Application Board 3.0
 *  2. Arduino NICLA
 *  3. Others in future?
 */

/**
 * @brief Ensure that the correct version of COINES for Zephyr is being used for
 * the currently selected board
 */
#if !defined(CONFIG_BOARD_BST_AB3_NRF52840) && !defined(CONFIG_BOARD_BST_ARDUINO_NICLA)
#error This version of COINES for Zephyr is only tested for AB3 and NICLA
#endif

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>

#include <coines.h>
#include "common_services.h"

#define LOG_MODULE_NAME coines
LOG_MODULE_REGISTER(LOG_MODULE_NAME);
/*Map COINES shuttle pin codes to DTS */
static const struct gpio_dt_spec gpio_dt_map[COINES_SHUTTLE_PIN_MAX] = {
                             [COINES_MINI_SHUTTLE_PIN_1_4] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_1_4),gpios),
                             [COINES_MINI_SHUTTLE_PIN_1_5] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_1_5),gpios),
                             [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_1_6),gpios),
                             [COINES_MINI_SHUTTLE_PIN_1_7] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_1_7),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_5] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_5),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_6),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_1] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_1),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_3] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_3),gpios),
                             [COINES_APP30_LED_R] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_red),gpios),
                             [COINES_APP30_LED_G] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_green),gpios),
                             [COINES_APP30_LED_B] = GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue),gpios),
                             [COINES_APP30_BUTTON_1] = GPIO_DT_SPEC_GET(DT_NODELABEL(button_t1),gpios),
                             [COINES_APP30_BUTTON_2] = GPIO_DT_SPEC_GET(DT_NODELABEL(button_t2),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_7] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_7),gpios),
                             [COINES_MINI_SHUTTLE_PIN_2_8] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_pin_2_8),gpios),
                             /*Additionally map AB2.0 pin enums to AB3.0 pins, so that code written using AB2.0
                             pin enums can run on AB3.0*/
                             [COINES_SHUTTLE_PIN_7] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_cs),gpios),     /*< AB2 CS  = AB3 CS*/
                             [COINES_SHUTTLE_PIN_8] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio0),gpios),  /*< AB2 Multi-IO 5 =AB3 GPIO0*/
                             [COINES_SHUTTLE_PIN_9] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio1),gpios),  /*< AB2 Multi-IO 0 = AB3 GPIO1*/
                             [COINES_SHUTTLE_PIN_14] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio4_ocsb),gpios),/*<AB2  Multi-IO 1 = AB3 GPIO4*/
                             [COINES_SHUTTLE_PIN_15] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio5_ascx),gpios),/*<AB2  Multi-IO 2 = AB3 GPIO5*/
                             //[COINES_SHUTTLE_PIN_16] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_sdi_sda),gpios), /*<AB2  Multi-IO 3 = AB3 SDI/SDA TBD????*/
                             [COINES_SHUTTLE_PIN_19] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio6_osdo),gpios),/*<AB2  Multi-IO 8 = AB3 GPIO6*/
                             [COINES_SHUTTLE_PIN_20] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio2_int1),gpios),/*<AB2  Multi-IO 6 = AB3 GPIO2/INT1*/
                             [COINES_SHUTTLE_PIN_21] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio3_int2),gpios),/*<AB2  Multi-IO 7 = GPIO3/INT2*/
                             [COINES_SHUTTLE_PIN_22] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_gpio7_asdx),gpios),/*<AB2  Multi-IO 4 = AB3 GPIO7*/
                             [COINES_SHUTTLE_PIN_SDO] = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_sdo),gpios),
};
/*keep track of shuttle gpio pin directions as a workaround 
for lack of get gpio pin config API in Zephyr*/
static uint32_t gpio_direction_bits =0;

/**
 * @brief BLE SERVICES RELATED APIs 
 * ================================
 */

/*!
 *  @brief This API is used to configure BLE name and power.This API should be called
 *         before calling coines_open_comm_intf().
 *
 *  @param[in] ble_config : structure holding ble name and power details
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t coines_ble_config(struct coines_ble_config *ble_config)
{
    /*
    NOTE: BT Advertising power has to be set in prj.conf. The power element of ble_config
    has no effect on power in COINES for Zephyr as setting of default power via API is not
    supported in Zephyr (it can be done dynamically for each advertisment via bt_hci_cmd_send_sync()
    as described in the Zephyr sample bt_hci_cmd_send_sync, but that is a complex procedure)
    */
   int rc = ble_service_config(ble_config->name,ble_config->tx_power);
   return (rc==0)?(COINES_SUCCESS):COINES_E_FAILURE;
}
/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer not used currently but for future use,
 *                          e.g. to input additional details like COM port, Baud rate, etc.
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
#ifdef COINES_AUTO_TEST
//define timeout for automated tests using JRun
#define TEST_TIMEOUT_PERIOD K_SECONDS(60)
void test_timeout(struct k_timer *dummy)
{
	puts("Test Timed Out - Stopping");
    puts("TEST STOP");
}

K_TIMER_DEFINE(t_test_tout,test_timeout,NULL);
#endif 
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
   ARG_UNUSED(arg);
   int  rc = 0;
    //initialize LEDs and buttons
	coines_set_pin_config(COINES_APP30_LED_R,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
 	coines_set_pin_config(COINES_APP30_LED_G,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
	coines_set_pin_config(COINES_APP30_LED_B,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW); 
	coines_set_pin_config(COINES_APP30_BUTTON_1,COINES_PIN_DIRECTION_IN,COINES_PIN_VALUE_HIGH); 	  
	coines_set_pin_config(COINES_APP30_BUTTON_2,COINES_PIN_DIRECTION_IN,COINES_PIN_VALUE_HIGH);
 
    //enable USB/CDC connection for terminal support
    //rc |= usb_enable(NULL); //done in usb_cdc_init()
    //wait for connection from terminal TODO:CHECK IF REQUIRED
    usb_cdc_init();
	if (intf_type == COINES_COMM_INTF_BLE)
    {
        rc |= ble_service_init();
    }
    
    //Set Blue LED in case of error, set Red LED always (TBD: Is this required?)
    coines_set_led(COINES_LED_RED,COINES_LED_STATE_ON);
    if(rc != 0){
        coines_set_led(COINES_LED_BLUE,COINES_LED_STATE_ON);
    }
	//timed interrupt feature initialization
	timed_interrupt_init();

#ifdef COINES_AUTO_TEST
	//set timeout for JRUN tests to prevent failing tests from blocking the automated script
    k_timer_start(&t_test_tout,TEST_TIMEOUT_PERIOD,K_NO_WAIT);
#endif
    return rc;
}

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer not used currently but for future use,
 *                          e.g. to input additional details like COM port, Baud rate, etc.
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    ARG_UNUSED(arg);
    /*Printing STOP is required by Segger J-Run to close auto-test connection and exit*/
#ifdef COINES_AUTO_TEST
    k_timer_stop(&t_test_tout);
	puts("Ending Auto Test");
    puts("TEST STOP");
#endif
    return (COINES_SUCCESS);
}

/*!
 * @brief Return the number of bytes available in the read buffer of the interface
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return number of bytes in the read buffer
 */
uint16_t coines_intf_available(enum coines_comm_intf intf)
{

    if ((intf == COINES_COMM_INTF_USB) && usb_cdc_connected())
    {
        return (uint16_t)usb_cdc_bytes_available();
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        return (uint16_t)ble_service_nus_bytes_available();
    }

    return 0;
}
/*!
 * @brief Check if the interface is connected
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return true if connected, false otherwise
 */
bool coines_intf_connected(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_BLE)
    {
        return ble_service_nus_connected();
    }
    else if (COINES_COMM_INTF_USB)
    {
        return usb_cdc_connected();
    }
    return false;
}

/*!
 * @brief Read data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer to store the data
 * @param[in] len     : Length of the buffer
 *
 * @return number of bytes read
 */
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    uint16_t bytes_read = 0;

    if ((intf == COINES_COMM_INTF_USB) && usb_cdc_connected())
    {
        bytes_read = (uint16_t)usb_cdc_read(buffer, len);
    }
    else 
    if (intf == COINES_COMM_INTF_BLE)
    {
        bytes_read = (uint16_t)ble_service_nus_read(buffer, len);
    }

    return bytes_read;
}

/*!
 * @brief Write data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer storing the data
 * @param[in] len     : Length of the buffer
 *
 * @return void
 */
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{

    if ((intf == COINES_COMM_INTF_USB) && usb_cdc_connected() )
    {
        (void)usb_cdc_write(buffer,len);
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        (void)ble_service_nus_write(buffer, len);
    }
}
/*!
 * @brief Flush the write buffer
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 *
 * @return void
 */
void coines_flush_intf(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        /* Do nothing */
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
         /* Do nothing */
         //currently, this function is used in non-Zephyr COINES for flushing NUS if accessed
         //via POSIX write, but since POSIX access to NUS is not supported in COINES for Zephyr
         //nothing to do here
    }
}

/*!
 * @brief This API is used to read the battery status .
 *
 * @param[out] bat_status_mv            :  Buffer to retrieve the battery status in millivolt.
 *
 * @param[out] bat_status_percent       :  Buffer to retrieve the battery status in %.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_bat_status(uint16_t *bat_status_mv, uint8_t *bat_status_percent)
{
    const struct device *dev;
	int status = 0;
	struct sensor_value voltage, state_of_charge;

    dev = DEVICE_DT_GET(DT_NODELABEL(bat_level));
	if (!dev) {
		LOG_ERR("Failed to get device battery level sensor device with label bat_level");
		return COINES_E_FAILURE;
	}

    status = sensor_sample_fetch_chan(dev,
                SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
    if (status < 0) {
        LOG_ERR("Unable to fetch battery state of charge: Err %d",status);
        return COINES_E_FAILURE;
    }

    status |= sensor_channel_get(dev,
                    SENSOR_CHAN_GAUGE_STATE_OF_CHARGE,
                    &state_of_charge);
    /*
    //NOTE: In existing implementations of bat level sensor for Nicla and AB3,
    //SOC fetch fetches voltage also, so an additional fetch is redundant and waste cycles
    //The below code may have to be uncommented if a bat_level driver is used in future
    //that is not written like this
    status != sensor_sample_fetch_chan(dev,
                        SENSOR_CHAN_GAUGE_VOLTAGE);
    */
    status |= sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE,
                    &voltage);
    //val1 is integral part in volts, val2 is decimal part, multiplied by million
    if(status == 0)
    {
        *bat_status_mv = (voltage.val1 *1000) + (voltage.val2/1000);
        *bat_status_percent = state_of_charge.val1;
        return COINES_SUCCESS;
    }
    else
    {
        LOG_ERR("Error getting battery status: %x",status);
        return COINES_E_FAILURE;
    }
}

/**
 * @brief TEMPERATURE RELATED APIs 
 * ================================
 */

static const struct device *temp_dev = NULL;
int16_t temp_dev_init(void)
{
	int ret;
	struct sensor_value attr;

    temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp_dev));
	if(temp_dev == NULL)
	{
		LOG_ERR("temperature sensor device temp_dev not defined");
		return COINES_E_DEVICE_NOT_FOUND;
	}
	LOG_INF("temperature device is %p, name is %s", temp_dev,
    		       temp_dev->name);
    //NOTE: following two attribute settings may not be implemented on
    //all drivers, hence no error return on failures
	attr.val1 = 150;
	attr.val2 = 0;
	ret = sensor_attr_set(temp_dev, SENSOR_CHAN_AMBIENT_TEMP,
			      SENSOR_ATTR_FULL_SCALE, &attr);
	if (ret) {
		LOG_ERR("sensor_attr_set SCALE failed. Ret %d", ret);
	}

	attr.val1 = 8;
	attr.val2 = 0;
	ret = sensor_attr_set(temp_dev, SENSOR_CHAN_AMBIENT_TEMP,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
	if (ret) {
		LOG_INF("sensor_attr_set SAMPLE FREQ failed. Err %d", ret);
	}	
    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to read the temperature sensor data.
 *
 * @param[out] temp_data       :  Buffer to retrieve the sensor data in degrees C.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_temp_data(float *temp_data)
{
	int rc;
	struct sensor_value temp_value;
    //initialize sensor on first call
	if(temp_dev == NULL){
        if((rc = temp_dev_init())){
    		return rc;
        }
	}
	rc = sensor_sample_fetch(temp_dev);
	if (rc) {
		LOG_ERR("sensor_sample_fetch failed. Err: %d\n", rc);
		return COINES_E_COMM_IO_ERROR;
	}

	rc = sensor_channel_get(temp_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);

	if (rc) {
		LOG_ERR("sensor_channel_get failed. Err: %d\n", rc);
		return COINES_E_COMM_IO_ERROR;
	}

	LOG_INF("temp is %d.%06d\n", temp_value.val1,
			temp_value.val2);

	*temp_data = (float)sensor_value_to_double(&temp_value);
	return COINES_SUCCESS;
}

/**
 * @brief GPIO RELATED APIs 
 * ========================
 */


int16_t coines_get_board_info(struct coines_board_info *data);
/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 *  @param[in] pin_number : pin to be configured.
 *  @param[in] direction : pin direction information(COINES_PIN_DIRECTION_IN or COINES_PIN_DIRECTION_OUT) *
 *  @param[in] pin_value : pin value information(COINES_PIN_VALUE_LOW or COINES_PIN_VALUE_HIGH)
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{
    int rc;
    gpio_flags_t extra_flags =0;
    struct gpio_dt_spec iospec = gpio_dt_map[pin_number];
    if(iospec.port == NULL)
    {
        LOG_ERR("Invalid gpio pin %d",(int)pin_number);
        return COINES_E_FAILURE;
    }
   
    if (direction == COINES_PIN_DIRECTION_IN)
    {
        extra_flags = GPIO_INPUT;
        extra_flags |= (pin_value == COINES_PIN_VALUE_HIGH)?GPIO_PULL_UP : GPIO_PULL_DOWN;
    }
    else if (direction == COINES_PIN_DIRECTION_OUT)
    {
         extra_flags =(pin_value == COINES_PIN_VALUE_HIGH)?GPIO_OUTPUT_ACTIVE:GPIO_OUTPUT_INACTIVE;
    }
    rc = gpio_pin_configure_dt(&iospec, extra_flags); 
    if( rc !=0)
    {
        LOG_ERR("Error configuring pin %d",pin_number);
        return rc;
    }
    WRITE_BIT(gpio_direction_bits,pin_number,direction);//keep track of direction as Zephyr does not have an API for this
    return COINES_SUCCESS;
}
/*!
 *  @brief This API function is used to get the pin direction and pin state.
 *
 *  @param[in] pin_number : pin number for getting the status.
 *  @param[out] pin_direction : pin direction information(COINES_PIN_DIRECTION_IN or COINES_PIN_DIRECTION_OUT)
 *  @param[out] pin_value : pin value information(COINES_PIN_VALUE_LOW or COINES_PIN_VALUE_HIGH)*
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    int rc;

    struct gpio_dt_spec iospec = gpio_dt_map[pin_number];
    if(iospec.port == NULL)
    {
        LOG_ERR("Invalid gpio pin %d",(int)pin_number);
        return COINES_E_FAILURE;
    }
    *pin_direction = ((gpio_direction_bits & (1 << pin_number)) !=0);//keep track of direction as Zephyr does not have an API for this
    rc = gpio_pin_get_dt(&iospec);
    if(rc>=0){
        *pin_value = rc;
        return COINES_SUCCESS;
    }
    return rc;
}

/*!
 *  @brief This API is used to set led state(on or off).
 *
 *  @param[in] led             : led to which the state has to be set
 *  @param[in] led_state       : state to be set to the given led
 * 
 *  NOTE: This function is implemented using the Zephyr LED API to abstract away the hardware
 *  differences between boards. E.g. in AB3, GPIO LED driver is used, and in NICLA, the 
 *  I2C LED driver is used. The only requirement is that the RGB led device should have the 
 *  "led_rgb:" label applied to it in the DTS. Also, the mapping of led index and color is
 *  assumed to be as follows:
 *      0:Blue 1:Green 2:Red 
 *  This is in line with the allocation on NICLA board, and it also corresponds to the order
 *  in the AB3 DTS. Not that it is the *reverse* of the COINES enum values 
 * 
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state)
{
    int16_t retval;
    uint8_t nled;
    const struct device *leddev = DEVICE_DT_GET(DT_NODELABEL(led_rgb));
	if(leddev == NULL || !device_is_ready(leddev)) {
		printf("LED driver not ready\n");
        return COINES_E_UNABLE_OPEN_DEVICE;
	}
    switch (led)
    {
        case COINES_LED_RED:
            nled=2;
            break;
        case COINES_LED_GREEN:
            nled=1;
            break;
        case COINES_LED_BLUE:
            nled=0;
            break;
        default:
            return COINES_E_NOT_SUPPORTED;
    }
    retval = (led_state == COINES_LED_STATE_ON)?
            led_on(leddev,nled):
            led_off(leddev,nled);
    return (retval)? COINES_E_FAILURE:COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 *  @param[in] vdd_millivolt     : VDD voltage to be set in sensor.
 *  @param[in] vddio_millivolt   : VDDIO voltage to be set in sensor.
 *
 *  @note In APP2.0 board, voltage level of 0 or 3300mV is supported.
 *        In APP3.0 board, voltage levels of 0, 1800mV and 2800mV are supported.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
	struct gpio_dt_spec ios_vdd_sel = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_sel),gpios);
	struct gpio_dt_spec ios_vdd_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vdd_en),gpios);
	struct gpio_dt_spec ios_vddio_en = GPIO_DT_SPEC_GET(DT_NODELABEL(shuttle_vddio_en),gpios);
    if (vdd_millivolt == 0)
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_INACTIVE);
    }
    else if ((vdd_millivolt > 0) && (vdd_millivolt <= 1800))
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_INACTIVE);
    }
    else
    {
        gpio_pin_set_dt(&ios_vdd_en,GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&ios_vdd_sel,GPIO_OUTPUT_ACTIVE);
    }

    if (vddio_millivolt == 0)
        gpio_pin_set_dt(&ios_vddio_en,GPIO_OUTPUT_INACTIVE);
    else
        gpio_pin_set_dt(&ios_vddio_en,GPIO_OUTPUT_ACTIVE);
    return COINES_SUCCESS;
}
/**
 * @brief I2C BUS RELATED APIs 
 * ===========================
 */
/*
NOTE: COINES_I2C_BUS_0 is mapped to shuttle_i2c (primary shuttle I2C port) and COINES_I2C_BUS_1
is mapped to shutle_aux_i2c (auxiliary shuttle I2C port e.g. auxilliary sensor port of some sensors). These
labels are defined as per the DTS Revision used. Each DTS Revision supports a different configuration.
Refer to the comments in the DTS file and the COINES for Zephur User Manuel for more details  
*/

/*!
 *   @brief Helper function for I2C get/set API calls 
 */
static int get_i2c_device(enum coines_i2c_bus bus, struct device **i2c_dev_p)
{
   const struct device *i2c_dev = NULL;// DEVICE_DT_GET(DT_NODELABEL(shuttle_i2c));
     //const struct device *i2c_dev=NULL;
    switch(bus)
    {
         case COINES_I2C_BUS_0:
#if  DT_NODE_EXISTS(DT_NODELABEL(shuttle_i2c))
            i2c_dev = DEVICE_DT_GET(DT_NODELABEL(shuttle_i2c));
#endif
            break;
        case COINES_I2C_BUS_1:
#if  DT_NODE_EXISTS(DT_NODELABEL(shuttle_aux_i2c))
            i2c_dev = DEVICE_DT_GET(DT_NODELABEL(shuttle_aux_i2c));
#endif
            break;
        default:
            i2c_dev = NULL;/*error*/
            break;
    }
    if(i2c_dev ==NULL)
    {
        LOG_ERR("Unknown Shuttle Board I2Cx");
        return COINES_E_DEVICE_NOT_FOUND;
    }
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("Shuttle Board I2C: Device is not ready.\n");
		return COINES_E_I2C_BUS_NOT_ENABLED;
	}
    *i2c_dev_p = (struct device *) i2c_dev;
    return 0;
}


/*!
 *  @brief This API is used to configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *  @param[in] i2c_mode   : i2c_mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode)
{
    int ret =COINES_SUCCESS;
    uint32_t i2c_cfg, i2c_speed;

    int rc =0;
    struct device *i2c_dev = NULL;
    rc = get_i2c_device(bus,&i2c_dev);
    if(rc!=0)
        return rc;

    switch(i2c_mode)
    {
        case COINES_I2C_STANDARD_MODE:   
            i2c_speed = I2C_SPEED_STANDARD;
            break;
        case  COINES_I2C_FAST_MODE:
            i2c_speed = I2C_SPEED_FAST;
            break;
        case COINES_I2C_SPEED_1_7_MHZ:
            i2c_speed = I2C_SPEED_FAST_PLUS;
            /*Actually this is 1Mhz, but closest available in Zephyr*/
            break;
        case COINES_I2C_SPEED_3_4_MHZ:
            i2c_speed = I2C_SPEED_HIGH;
            break;
        default :
		    LOG_ERR("Unknown I2C mode %d\n",(int)i2c_mode);
            return -EINVAL;
    }

	if (!i2c_dev) {
		LOG_ERR("Shuttle I2C device not found!");
		return -EINVAL;
	}
	i2c_cfg = I2C_MODE_CONTROLLER | I2C_SPEED_SET(i2c_speed);
	ret = i2c_configure(i2c_dev, i2c_cfg);
    if(ret!=0){
		LOG_ERR("Error configuring Shuttle I2C %d",ret);
        ret = COINES_E_I2C_CONFIG_FAILED;
    }
    //NOTE: i2c is enabled in the DTC in Zephyr. It cannot be enabled in the config
    //API, as done in old COINES

    return ret;
}
/*!
 *  @brief This API is used to de-configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_deconfig_i2c_bus(enum coines_i2c_bus bus)
{
    //TODO: Disable i2c. 
    //      Is this required? Check. Zephyr does not seem to have an API for this.
    return 0;
}
/*!
 *  @brief This API is used to write 8-bit register data on the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    int rc =0;
    struct device *i2c_dev = NULL;
    rc = get_i2c_device(bus,&i2c_dev);
    if(rc!=0)
        return rc;
    rc = i2c_burst_write(i2c_dev, dev_addr, reg_addr, reg_data, count);
    if(rc!=0)   {
        i2c_recover_bus(i2c_dev);//attempt to recover bus if tx failed for any reason
    }
    return rc;
}

/*!
 *  @brief This API is used to read 8-bit register data from the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr  : Device address for I2C read.
 *  @param[in] reg_addr  : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count     : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    int rc=0;
    struct device *i2c_dev = NULL;
    rc = get_i2c_device(bus,&i2c_dev);
    if(rc!=0)
        return rc;
    rc = i2c_burst_read(i2c_dev, dev_addr, reg_addr, reg_data, count);
    if(rc!=0)   {
        i2c_recover_bus(i2c_dev);//attempt to recover bus if tx failed for any reason
    }
    return rc;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] data     : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
    struct device *i2c_dev = NULL;
    int rc = get_i2c_device(bus,&i2c_dev);
    if(rc!=0)
        return rc;
    rc = i2c_write(i2c_dev, data, count, dev_addr);
    if(rc!=0)   {
        i2c_recover_bus(i2c_dev);//attempt to recover bus if tx failed for any reason
    }
    return rc;
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C read.
 *  @param[out] data    : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
struct device *i2c_dev = NULL;
    int rc = get_i2c_device(bus,&i2c_dev);
    if(rc!=0)
        return rc;
    rc = i2c_read(i2c_dev, data, count, dev_addr);
    if(rc!=0)   {
        i2c_recover_bus(i2c_dev);//attempt to recover bus if tx failed for any reason
    }
    return rc;
}

/**
 * @brief TIME AND DELAY RELATED APIs 
 * ==================================
 */

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t coines_get_millis()
{
    //NOTE:k_uptime_get_32() is called to maintain compatibility with old COINES
    //This can be replaced with 64 bit k_uptime_get() if API is modified to return a
    //64bit value.
    return k_uptime_get_32();
}

/*!
 * @brief This API returns the number of microseconds passed since the program started
 *
 * @return Time in microseconds
 */
uint64_t coines_get_micro_sec()
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

/*!
 * @brief This API is used to introduce delay based on high precision RTC(LFCLK crystal)
 * with the resolution of 30.517 usec
 *
 * @param[in]   : required delay in microseconds
 * @return      : None
 */
void coines_delay_realtime_usec(uint32_t period)
{
    //Zephyr already uses RTC1 for the kernel on Nordic platform, so k_sleep
    //etc. already have the precision provided by RTC. If necessary, RTC0
    //can be used to implement RTC on same lines as old COINES
    //as per the sample in \zephyr\tests\drivers\timer\nrf_rtc_timer
    //and \nrf\samples\debug\ppi_trace
     k_sleep(K_USEC(period));
}

/*!
 * @brief This API is used to get the current counter(RTC) reference time in usec
 *
 * @param[in]   : None
 * @return      : counter(RTC) reference time in usec
 * */
uint32_t coines_get_realtime_usec(void)
{
    //Zephyr already uses RTC1 for the kernel on Nordic platform, so k_sleep
    //etc. already have the precision provided by RTC. If necessary, RTC0
    //can be used to implement RTC on same lines as old COINES
    //as per the sample in \zephyr\tests\drivers\timer\nrf_rtc_timer
    //and \nrf\samples\debug\ppi_trace
    return k_ticks_to_us_floor32(k_uptime_ticks());
}

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 *  @param[in] delay_ms   :  delay in milliseconds.
 *
 *  @return void
 */
void coines_delay_msec(uint32_t delay_ms)
{
	k_sleep(K_MSEC(delay_ms));
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 *
 *  @param[in] delay_us   :  delay in microseconds.
 *
 *  @return void
 */
void coines_delay_usec(uint32_t delay_us)
{
    k_sleep(K_USEC(delay_us));
}

/**
 * @brief SPI Bus related API 
 * 
 */
//default configuration of shuttle board SPI connection. Will be overwritten
//in coines_config_spi_bus()
//COINES_SPI_MODE0 SPI Mode 0: CPOL=0; CPHA=0
#define SPI_OPERATION_MODE0 (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_MSB)
//COINES_SPI_MODE1 SPI Mode 1: CPOL=0; CPHA=1
#define SPI_OPERATION_MODE1 (SPI_OPERATION_MODE0 | SPI_MODE_CPHA )
//COINES_SPI_MODE2 SPI Mode 2: CPOL=1; CPHA=0
#define SPI_OPERATION_MODE2 (SPI_OPERATION_MODE0 | SPI_MODE_CPOL)
//COINES_SPI_MODE3 SPI Mode 3: CPOL=1; CPHA=1
#define SPI_OPERATION_MODE3 (SPI_OPERATION_MODE0 | SPI_MODE_CPOL | SPI_MODE_CPHA )

#define SPI_DELAY 200U
#define SPI_FREQUENCY ((uint32_t)8000000)       /*Default 8 Mhz*/
#define SPI_REG_ADDR 0

/*
NOTE2: COINES_SPI_BUS_0 is mapped to shuttle_spi (primary shuttle SPI port) and COINES_SPI_BUS_1
is mapped to shutle_aux_spi (auxiliary shuttle SPI port e.g. OIS port of some sensors). These
labels are defined as per the DTS Revision used. Each DTS Revision supports a different configuration.
Refer to the comments in the DTS file and the COINES for Zephur User Manuel for more details  
*/

static struct spi_dt_spec spi_specs[COINES_SPI_BUS_MAX] = {
#if DT_NODE_EXISTS(DT_NODELABEL(shuttle_spi))
    [COINES_SPI_BUS_0] = SPI_DT_SPEC_GET(DT_NODELABEL(shuttle_spi),SPI_OPERATION_MODE3,SPI_DELAY),
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(shuttle_aux_spi))
     [COINES_SPI_BUS_1] = SPI_DT_SPEC_GET(DT_NODELABEL(shuttle_aux_spi),SPI_OPERATION_MODE3,SPI_DELAY),
#endif
};
/*helper function for SPI get/set API calls */
static int get_spi_device(enum coines_spi_bus bus, struct spi_dt_spec **spi_spec_p)
{
    struct spi_dt_spec *spi_spec=NULL;
    if(bus >= COINES_SPI_BUS_MAX)
        return COINES_E_SPI_INVALID_BUS_INTF;
    spi_spec = &spi_specs[bus];
    if(spi_spec->bus ==NULL)
    {
        LOG_ERR("Unknown Shuttle Board SPIx");
        return COINES_E_SPI_INVALID_BUS_INTF;
    }
	if (!device_is_ready(spi_spec->bus)) {
		LOG_ERR("Shuttle Board SPI: Device is not ready.\n");
		return COINES_E_SPI_BUS_NOT_ENABLED;
	}
    //override CS in DTS so that CS pin can be manually controlled in COINES API functions below
    //This is because default Zephyr SPI driver CS logic is incompatible with BST sensor chips
    //TODO: is there a way of getting the Zephyr SPI driver to handle CS for COINES?
    spi_spec->config.cs = NULL; 
    *spi_spec_p = spi_spec;
    return 0;
}

/*!
 *  @brief This API is used to configure the SPI bus
 *
 *  @param[in] bus         : bus
 *  @param[in] spi_speed   : SPI speed
 *  @param[in] spi_mode    : SPI mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    struct spi_dt_spec *spi_spec=NULL;
    int rc = get_spi_device(bus,&spi_spec);
    if(rc != 0)
        return rc;
    #define COINES_NRF_SPEED_MAP(coines_spi,spi_hz)  \
        case  COINES_SPI_SPEED_##coines_spi:         \
        spi_spec->config.frequency = spi_hz##000;\
        break \

    switch(spi_speed)
    {
        COINES_NRF_SPEED_MAP(250_KHZ, 250);
        COINES_NRF_SPEED_MAP(300_KHZ,250);
        COINES_NRF_SPEED_MAP(400_KHZ,500);
        COINES_NRF_SPEED_MAP(500_KHZ,500);
        COINES_NRF_SPEED_MAP(600_KHZ,500);
        COINES_NRF_SPEED_MAP(750_KHZ,1000);
        COINES_NRF_SPEED_MAP(1_MHZ,1000);
        COINES_NRF_SPEED_MAP(1_2_MHZ,1000);
        COINES_NRF_SPEED_MAP(1_25_MHZ,1000);
        COINES_NRF_SPEED_MAP(1_5_MHZ,2000);
        COINES_NRF_SPEED_MAP(2_MHZ,2000);
        COINES_NRF_SPEED_MAP(2_5_MHZ,2000);
        COINES_NRF_SPEED_MAP(3_MHZ,4000);
        COINES_NRF_SPEED_MAP(3_75_MHZ,4000);
        COINES_NRF_SPEED_MAP(5_MHZ,4000);
        COINES_NRF_SPEED_MAP(6_MHZ,4000);
        COINES_NRF_SPEED_MAP(7_5_MHZ,4000);
        COINES_NRF_SPEED_MAP(10_MHZ,8000);
        default:
            spi_spec->config.frequency = SPI_FREQUENCY;\
            break;
    }
    switch(spi_mode)
    {
        case COINES_SPI_MODE0: //< SPI Mode 0: CPOL=0; CPHA=0
            spi_spec->config.operation = SPI_OPERATION_MODE0;
            break;
        case COINES_SPI_MODE1: //< SPI Mode 1: CPOL=0; CPHA=1 
            spi_spec->config.operation = SPI_OPERATION_MODE1;
            break;
        case COINES_SPI_MODE2: //< SPI Mode 2: CPOL=1; CPHA=0 
            spi_spec->config.operation = SPI_OPERATION_MODE2;
            break;
        case COINES_SPI_MODE3://< SPI Mode 3: CPOL=1; CPHA=1 
        default:
            spi_spec->config.operation = SPI_OPERATION_MODE3;
            break;
    }
    return 0;
}

/*!
 *  @brief This API is used to de-configure the SPI bus
 *
 *  @param[in] bus         : bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t coines_deconfig_spi_bus(enum coines_spi_bus bus)
{
    //Not supported in Zephyr
    return 0;
}




/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI read.
 *  @param[in] reg_addr : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 * ***************************************************************************************
 *  NOTE: BIT 7 (R/W BIT) of reg_addr should be set by application for this function to
 * work properly with Bosch sensors. 1 = Read, 0 = Write. This is not being done here to be 
 * compatible with previous versions of COINES. Also, the first byte received is a dummy byte
 * which should be ignored by application. This also is not being handled here for the same
 * reason.
 * ***************************************************************************************
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count_)
{
    struct spi_dt_spec *spi_spec=NULL;
    int rc = get_spi_device(bus,&spi_spec);
    if(rc!=0)
        return rc;

    const struct spi_buf tx_buf = {
		.buf = &reg_addr,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
    struct spi_buf rx_buf = {
        .buf = reg_data,
        .len = count_,
	};

	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};
   coines_set_pin_config(dev_addr,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
	rc |= spi_write(spi_spec->bus, &spi_spec->config,&tx);
	rc |= spi_read(spi_spec->bus, &spi_spec->config,&rx);
   coines_set_pin_config(dev_addr,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
    return rc;
}

/*!
 *  @brief This API is used to write 8-bit register data on the SPI device.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    struct spi_dt_spec *spi_spec=NULL;
    int rc = get_spi_device(bus,&spi_spec);
    if(rc!=0)
        return rc;

    uint8_t reg_data_cpy[count];
	const struct spi_buf tx_bufs[] = {
        {
            .buf = &reg_addr,
            .len = 1,
        },
        {
            .buf = reg_data_cpy,
            .len = count,
        }
    };

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 2,
	};
    //make local copy of regdata, as SPI peripheral requires buffer to be in RAM data section
    //and it cannot be certain what type of buffer the user is passing
    memcpy(reg_data_cpy,reg_data,count);
    coines_set_pin_config(dev_addr,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
	rc = spi_write(spi_spec->bus, &spi_spec->config,&tx);
    coines_set_pin_config(dev_addr,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
    return rc;
}

int16_t coines_config_word_spi_bus(enum coines_spi_bus bus,
                                   enum coines_spi_speed spi_speed,
                                   enum coines_spi_mode spi_mode,
                                   enum coines_spi_transfer_bits spi_transfer_bits)
{
    //TODO: Is this reqd? Does not seem to be supported in old COINES for ab3.0
    return COINES_E_NOT_SUPPORTED;
}

int8_t coines_write_16bit_spi(enum coines_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count)
{
    //TODO: Is this reqd? Does not seem to be supported in old COINES for ab3.0
    return COINES_E_NOT_SUPPORTED;
}

int8_t coines_read_16bit_spi(enum coines_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count)
{
    //TODO: Is this reqd? Does not seem to be supported in old COINES for ab3.0
    return COINES_E_NOT_SUPPORTED;
}


/**
 * @brief I2S Bus Related API
 * 
 */
/**
 * @brief This API is used to configure the I2S bus to match the TDM configuration
 *
 */
int16_t coines_config_i2s_bus(uint16_t data_words, coines_tdm_callback callback)
{ 
    //TODO: This API is meant to support a specific sensor which is currently not productionized.
    //It will be implementead at a later date when the sensor is available
    return COINES_E_NOT_SUPPORTED;
}

/**
 * @brief This API is used to stop the I2S/TDM interface from reading data from the sensor
 */
void coines_deconfig_i2s_bus(void)
{
    //TODO
}

/**
 * @brief Timer Related API
 * 
 */
K_TIMER_DEFINE(TIM0,NULL,NULL);
K_TIMER_DEFINE(TIM1,NULL,NULL);
//K_TIMER_DEFINE(TIM2,NULL,NULL); //reduced to 2 timers in COINES2.8 onwards. Changed in ZCOINES also for consistency 

struct k_timer *timer_instance[COINES_TIMER_INSTANCE_MAX] = {
    &TIM0, 
    &TIM1,
    //&TIM2
};
/*Check against future changes in COINES.h*/
#if (COINES_TIMER_INSTANCE_MAX > 2)
    #error "Only two timers are currently Supported in COINES for Zephyr"
#endif
/*!
 * @brief This API is used to configure the hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] handler : callback to be called when timer expires.
 *                      use the function pointer for timer event handler as per the TARGET
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_config(enum coines_timer_instance instance, void* handler)
{
    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        k_timer_init(timer_instance[instance],(k_timer_expiry_t)handler,NULL);
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to start the configured hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] timeout : in microsecond
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_start(enum coines_timer_instance instance, uint32_t timeout)
{
  if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        //duration: first timeout, period: subsequent timeouts
        k_timer_start(timer_instance[instance],K_USEC(timeout),K_USEC(timeout));
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}


/*!
 * @brief This API is used to stop the hardware timer
 *
 * @param[in] instance : timer instance
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_timer_stop(enum coines_timer_instance instance)
{
    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
         k_timer_stop(timer_instance[instance]);
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 *
 * @param[in] tmr_cfg : timer config value
 * @param[in] ts_cfg : timer stamp cfg value
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{
    (void)tmr_cfg;
    (void)ts_cfg;
    //Currently not supported in non-Zephyr COINES, hence not suppported in Zephyr version also
    //to maintain compatibility
    return COINES_E_NOT_SUPPORTED;
}

/**
 * @brief GPIO Interrupt Related API
 * 
 */

/*
    signature of COINES gpio interrupt callback handler:
    void callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
    where both arguments are typecast to uint_32
*/
typedef void (*coines_gpio_callback_handler_t)(uint32_t pin, uint32_t action);
/*array of Zephyr call back structures, one for each shuttle pin*/
struct gpio_callback gpio_callbacks[COINES_SHUTTLE_PIN_MAX];
/*array of corresponding COINES callback function pointers, one for each shuttle pin*/
coines_gpio_callback_handler_t coines_gpio_callbacks[COINES_SHUTTLE_PIN_MAX] ={NULL};
/*Following set of macros can be used to retrieve NRF absolute pin no. from DTS pin label
as explained in below NOTE. EXAMPLE USAGE: int abspin0 = GPIO_ABS_PIN(button0)*/
#define GET_PIN(name, prop, idx) \
	DT_GPIO_PIN_BY_IDX(DT_NODELABEL(name), prop, idx)
#define GET_PORT(name, prop, idx) \
	DT_PROP_BY_PHANDLE_IDX(DT_NODELABEL(name), prop, idx, port)
#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))
#define GPIO_ABS_PIN(pin) NRF_GPIO_PIN_MAP(GPIO_PORT(pin), GPIO_PIN(pin))
/**
 * @brief common gpio callback handler: maps from Zypher callback to COINES callback format
 * NOTE: The pin number passed to COINES callback is the COINES shuttle pin enum, as updated
 * in COINES V2.8 onwards (earlier it was the NRF pin number)
 * NOTE: The second argument is the COINES polarity, as per COINES V2.8 onwards. However
 * as the polarity information is not available in the Zephyr callback, it is deduced
 * from the pin status. This should work in most cases.
 */

void gpio_callback_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    int pin =0, pin_mask;
    while(pins)
    {
        pin_mask = (1U<<pin); 
        if(pins & pin_mask)
        {
            for(int i=0;i<COINES_SHUTTLE_PIN_MAX;i++)
            {
                if(gpio_dt_map[i].port == port && gpio_dt_map[i].pin == pin)
                if(coines_gpio_callbacks[i]!=NULL) {
                    //@ref see NOTE above
                    coines_gpio_callbacks[i](i,
                        (gpio_pin_get(port,pin)==0)?
                            COINES_PIN_INT_POLARITY_HIGH_TO_LOW:
                            COINES_PIN_INT_POLARITY_LOW_TO_HIGH);
                }
            }
            pins ^= pin_mask;
        }
        pin++;
    }
}

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 * @param[in] callback : Name of the function to be called on detection of interrupt
 * @param[in] mode : Trigger modes - change,rising edge,falling edge
 *
 * @return void
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                            void (*callback)(uint32_t, uint32_t),
                             enum coines_pin_interrupt_mode int_mode)
{
    int rc=0;
    gpio_flags_t flags=GPIO_INT_DISABLE;

    struct gpio_dt_spec iospec = gpio_dt_map[pin_number];
    if(iospec.port == NULL)
    {
        LOG_ERR("Invalid gpio pin %d",(int)pin_number);
        return ;
    }
    if (int_mode == COINES_PIN_INTERRUPT_CHANGE)
        flags = GPIO_INT_EDGE_BOTH;
    else if (int_mode == COINES_PIN_INTERRUPT_RISING_EDGE)
        flags = GPIO_INT_EDGE_RISING;
    else if (int_mode == COINES_PIN_INTERRUPT_FALLING_EDGE)
        flags = GPIO_INT_EDGE_FALLING;
    coines_gpio_callbacks[pin_number] = callback;
    rc |= gpio_pin_interrupt_configure_dt(&iospec,flags);
    //initialize the appropriate gpio_callback structure 
    gpio_init_callback(&gpio_callbacks[pin_number], gpio_callback_handler, (1U<<iospec.pin));
    //register the callback structure
    rc |= gpio_add_callback(iospec.port, &gpio_callbacks[pin_number]);
    if(rc!=0)
    {
        LOG_ERR("Error attaching interrupt for pin %d",(int)pin_number);
    }
}
/*!
 * @brief Detaches a interrupt from a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 *
 * @return void
 */
void coines_detach_interrupt(enum coines_multi_io_pin pin_number)
{
    struct gpio_dt_spec iospec = gpio_dt_map[pin_number];
    if(iospec.port == NULL)
    {
        LOG_ERR("Invalid gpio pin %d",(int)pin_number);
        return;
    }
    /* Cleanup: deregister callback request. The COINES callback function is not
    removed from the callback array for safety, in case there is a pending interrupt at
    this time. Normally, it will not be invoked until the next attach interrupt */
    gpio_remove_callback(iospec.port, &gpio_callbacks[pin_number]);
}

/**
 * @brief Miscellaneous API
 * 
 */


/*!
 *
 * @brief       : API to get shuttle ID
 *
 * @param[in]   : None
 * @return      : shuttle id
 */
static uint16_t get_shuttle_id()
{
    //TODO: Implement Zephyr EEPROM driver for DS208E05 
    //so that shuttle ID can be read from shuttle EEPROM.
    //Right now, 0xFFFF is returned as a temporary workaround
    uint16_t shuttle_id = 0xFFFF;
    /* NRFX_IRQ_DISABLE(USBD_IRQn);
    (void)app30_eeprom_read(0x01, (uint8_t *)&shuttle_id, 2);
    NRFX_IRQ_ENABLE(USBD_IRQn);*/
    return shuttle_id;
}


/*!
 *  @brief This API is used to get the board information.
 */
int16_t coines_get_board_info(struct coines_board_info *data)
{

    if (data != NULL)
    {
        data->board = 5;
        data->hardware_id = 0x11;
        data->shuttle_id = get_shuttle_id();
        data->software_id = 0x10;
        return COINES_SUCCESS;
    }
    else
        return COINES_E_NULL_PTR;

}

/*!
 * @brief Get COINES library version
 *
 * @return pointer to version string
 */
const char* coines_get_version()
{
    return COINES_VERSION;
}

/*!
 * @brief Resets the device
 *
 * @note  After reset device jumps to the address specified in makefile (APP_START_ADDRESS).
 *
 * @return void
 */
#define  MAGIC_LOCATION          (0x2003FFF4)
#define  MAGIC_INFO_ADDR         ((int8_t *)(MAGIC_LOCATION))
#define  APP_START_ADDR          (*(uint32_t *)(MAGIC_LOCATION + 4))
#define  APP_SP_VALUE            (*(uint32_t *)APP_START_ADDR)
#define  APP_RESET_HANDLER_ADDR  (*(uint32_t *)(APP_START_ADDR + 4))
void coines_soft_reset(void)
{
#ifndef COINES_AUTO_TEST //Disabled in case of automated testing, as it interferes with it
    //TODO: below line was in previous version of COINES. It is required for
    //the bootloader, as per Kevin. To check if this is required or if it
    //needs to be removed or modified. This address seems to be in an unused 
    //part of the data SRAM
    memcpy((uint32_t *)MAGIC_LOCATION, "COIN", 4); // *MAGIC_LOCATION = 0x4E494F43; // 'N','O','I','C'

    //TODO: below line was in previous version of COINES. It is presumably required
    // to "begin at the beginning" after a NVIC_SystemReset(). The Zephyr equivalent
    //sys_reboot(SYS_REBOOT_COLD) takes care of it, so it is not required. So it is
    //commented out. To verify if this line is still required, and how it needs to be
    //modified, as APP_START_ADDRESS is not available in the current Zephyr build
    /*APP_START_ADDR = APP_START_ADDRESS; // Application start address;*/

    sys_reboot(SYS_REBOOT_COLD);
#endif
}

/**
 * @brief This API can be defined to perform a task when yielded from an ongoing blocking call
 */
void coines_yield(void)
{
    k_yield();
}
/*!
 *  @brief This API is used to execute the function inside critical region.
 *
 *  @param[in] callback   : function to execute
 *  
 *  @return void
 *  NOTE: Zephyr or even nRF Connect SDK does not contain an implementation of CRITICAL_REGION_ENTER
 *  /CRITICAL_REGION_EXIT, unlike the NORDIC Sofdevice SDK used in old COINES. irq_lock/unlock is the 
 *  nearest equivalent in Zephyr. In any case, this is what is done internally by the softdevice macros  
 */
void coines_execute_critical_region(coines_critical_callback callback)
{
    if (callback)
    {
        unsigned int key = irq_lock();
        callback();
        irq_unlock(key);
    }
}

/**
 * @brief COINES specific fatal error handler which overrides the Zephyr fatal error handler
 * and nRF Connect handler. Basically, auto-reset is disabled and the JRUN stop command is
 * pushed to console. This is required for automated batch testing support. See sensor_test
 * project for more details
 */
void k_sys_fatal_error_handler(unsigned int reason,
			       const z_arch_esf_t *esf)
{
	ARG_UNUSED(esf);
	ARG_UNUSED(reason);

	LOG_PANIC();
    /*Printing STOP is required by Segger J-Run to close auto-test connection and exit*/
#ifdef COINES_AUTO_TEST
	puts("Fatal Error: Ending Auto Test");
    puts("TEST STOP");
#endif
		LOG_ERR("Fatal Error: Halting system");
		for (;;) {
			/* Spin endlessly */
	}

	CODE_UNREACHABLE;
}
