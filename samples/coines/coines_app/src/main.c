/*
 * Copyright (c) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * Test program for COINES API for AB3.0 on Zephyr
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <coines.h>

/*Define the number of buttons available on the test board. Default is 2.
Setting it larger than 2 is meaningless as 2 is the maximum used in below tests*/
#ifdef CONFIG_BOARD_BOSCH_APP30
#define TEST_BOARD_BUTTONS 2
#elif CONFIG_BOARD_BOSCH_APP31
#define TEST_BOARD_BUTTONS 2
#elif CONFIG_BOARD_BOSCH_NICLA_SENSE
#define TEST_BOARD_BUTTONS 1
#else
#define TEST_BOARD_BUTTONS 2
#endif
/*
 * GPIO & LED Test
 */
int test_gpio_led(void)
{
	int cnt =0;
	bool pb1_pressed = false, pb1_tog = false;
	bool pb2_pressed = false, pb2_tog = false;
	bool ledg_tog =false;
	enum coines_pin_direction pin_dir;
	enum coines_pin_value pin_val;
	uint32_t oldms = coines_get_millis();
	printf("Press TB1 and TB2 to toggle Red and Blue LEDs (6 times)\n");
	coines_set_led(COINES_LED_RED,COINES_LED_STATE_OFF);
	coines_set_led(COINES_LED_BLUE,COINES_LED_STATE_OFF);
	coines_set_led(COINES_LED_GREEN,COINES_LED_STATE_OFF);
    while (cnt<5)
    {
		if(coines_get_pin_config(COINES_APP30_BUTTON_1,&pin_dir,&pin_val)!=0 || pin_dir != COINES_PIN_DIRECTION_IN)
		{
			printf("Error reading button1 pin\n");
			return -1;
		}
		if(pin_val && ! pb1_pressed)
		{
			pb1_tog = !pb1_tog;
			coines_set_led(COINES_LED_RED,(pb1_tog?COINES_LED_STATE_ON:COINES_LED_STATE_ON));
			cnt++;
		}
		pb1_pressed = pin_val;
		if(coines_get_pin_config(COINES_APP30_BUTTON_2,&pin_dir,&pin_val)!=0  || pin_dir != COINES_PIN_DIRECTION_IN)
		{
			printf("Error reading button2 pin\n");
			return -1;
		}
		if(pin_val && ! pb2_pressed)
		{
			pb2_tog = ! pb2_tog;
			coines_set_led(COINES_LED_BLUE,(pb2_tog?COINES_LED_STATE_ON:COINES_LED_STATE_ON));
			cnt++;
		}
		pb2_pressed = pin_val;
		if(coines_get_millis() > oldms+2000)
		{
			oldms = coines_get_millis();
			ledg_tog =!ledg_tog;
			coines_set_led(COINES_LED_GREEN,(ledg_tog)?COINES_LED_STATE_ON:COINES_LED_STATE_OFF);
		}
	}
	//cleanup before quitting
	coines_set_led(COINES_LED_RED,COINES_LED_STATE_OFF);
	coines_set_led(COINES_LED_BLUE,COINES_LED_STATE_OFF);
	coines_set_led(COINES_LED_GREEN,COINES_LED_STATE_OFF);
	puts("GPIO & LED Test Over");
	return 0;
}

#ifdef CONFIG_BOARD_BOSCH_NICLA_SENSE
#define BHI260_REG_CHIP_ID (0x2B)
#define BHI260_REG_FUSER_ID (0x1C)
#define BHI260_REG_FUSER_VER (0x1D)
#define BHI260_REG_RESET_REQ 0x14
#define BHI260_I2C_ADDR 0x69
#define BHI260_POWER_ON_TIME 500
#define BHI260_CMD_SOFT_RESET (0x01)
#define BHI260_SOFT_RESET_TIME 2000
#define BHI260_REG_HOST_STATUS (0x17)
#define BHI260_REG_BOOT_STATUS (0x25)
/*
 * Test SPI (for Nicla)
 */
#define BHI260_SPI_REG_READ_CMD (0x80)
#define BHI260_SPI_REG_WRITE_CMD (0x00)
//in case of SPI, Bit7 of Register Address is the R/W bit. It should be set to 1 for 
//read operations and 0 for Write operations. In I2C, the RW bit is part of the slave
//device address, and is taken care of by the protocol
int test_spi(void)
{
	int cnt =0;
	//pull CS low to put into SPI mode 
	//coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
	coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
	coines_delay_usec(800);
    while (cnt++<10)
    {
		uint8_t chip_id;
		uint8_t soft_reset_cmd;
		uint8_t status;
		int rc;
		printf("Testing SPI API\n");
		coines_delay_usec(BHI260_POWER_ON_TIME);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_CHIP_ID|BHI260_SPI_REG_READ_CMD, &chip_id, 1);
		printf("chipid:%x rc%d\n",chip_id,rc);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_FUSER_ID|BHI260_SPI_REG_READ_CMD, &chip_id, 1);
		printf("fuserid:%x rc%d\n",chip_id,rc);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_FUSER_VER|BHI260_SPI_REG_READ_CMD, &chip_id, 1);
		printf("fuserver:%x rc%d\n",chip_id,rc);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_HOST_STATUS|BHI260_SPI_REG_READ_CMD, &status, 1);		
		printf("host status:%x rc%d\n",status,rc);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_BOOT_STATUS|BHI260_SPI_REG_READ_CMD, &status, 1);		
		printf("boot status before reset:%x rc%d\n",status,rc);
		soft_reset_cmd = BHI260_CMD_SOFT_RESET;
		rc=coines_write_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1,BHI260_REG_RESET_REQ|BHI260_SPI_REG_WRITE_CMD, &soft_reset_cmd, 1);
		if (rc != 0) {
			printf("error doing BHI260 reset: rc%d\n",rc);
		}
		coines_delay_usec(BHI260_SOFT_RESET_TIME);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BHI260_REG_BOOT_STATUS|BHI260_SPI_REG_READ_CMD, &status, 1);		
		printf("boot status after reset:%x rc%d\n",status,rc);

		coines_delay_msec(3000);
	}
	puts("SPI test over");
    return 0;	
}
/*dummy tests to satisfy build in case of unsupported board*/
int test_i2c(void)
{
	puts("test_i2c() is not available for this board");
	return 0;
}
int test_aux_spi(void)
{
	puts("test_aux_spi() is not available for this board");
	return 0;
}
#else /*CONFIG_BOARD_BOSCH_NICLA_SENSE: below tests not valid for Nicla*/
/*
 * I2C API Test
 */
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_CMD 0x7E
#define BMI270_I2C_ADDR 0x69
#define BMI270_POWER_ON_TIME 500
#define BMI270_CMD_SOFT_RESET 0xB6
#define BMI270_SOFT_RESET_TIME 2000
#define BMI270_REG_INTERNAL_STATUS 0x21

int test_i2c(void)
{
	int ret =0;
	int cnt =0;
	//CS should be High for I2C
	//SP2.1 = CS
	ret |= coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
	//SDO High: I2C Address is 0x69
	//SDO Low: I2C Address is 0x68
	//SP2.3 = SDO
 	ret |= coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_3,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH); 
	if(ret!=0)
	{
		printf("Error setting cs and sdo pins %d\n",ret);
		return -1;
	}
    while (cnt++ <10)
    {
		uint8_t chip_id;
		uint8_t soft_reset_cmd;
		uint8_t status;
		int rc;
		printf("Testing I2C Read+Write API\n");
		coines_delay_usec(BMI270_POWER_ON_TIME);
		rc=coines_read_i2c(COINES_I2C_BUS_0,BMI270_I2C_ADDR, BMI270_REG_CHIP_ID, &chip_id, 1);
		printf("chipid:%x rc%d\n",chip_id,rc);

		soft_reset_cmd = BMI270_CMD_SOFT_RESET;
		rc=coines_write_i2c(COINES_I2C_BUS_0,BMI270_I2C_ADDR,BMI270_REG_CMD, &soft_reset_cmd, 1);
		if (rc != 0) {
			printf("error doing bmi270 reset: rc%d\n",rc);
		}
		coines_delay_usec(BMI270_SOFT_RESET_TIME);
		rc=coines_read_i2c(COINES_I2C_BUS_0,BMI270_I2C_ADDR, BMI270_REG_INTERNAL_STATUS, &status, 1);		
		printf("internal status after reset:%x rc%d\n",status,rc);

		printf("Testing I2C Read-Only/Write-Only API\n");
		coines_delay_usec(BMI270_POWER_ON_TIME);
		uint8_t reg = BMI270_REG_CHIP_ID;
		rc=coines_i2c_set(COINES_I2C_BUS_0,BMI270_I2C_ADDR,&reg, 1);
		rc|=coines_i2c_get(COINES_I2C_BUS_0,BMI270_I2C_ADDR,&chip_id, 1);
		printf("chipid:%x rc%d\n",chip_id,rc);
		coines_delay_msec(3000);
	}
	puts("I2C Test over");
    return 0;	
}

/*
 * Test SPI
 */
#define BMI270_SPI_REG_READ_CMD 0x80
#define BMI270_SPI_REG_WRITE_CMD 0x00
//in case of SPI, Bit7 of Register Address is the R/W bit. It should be set to 1 for 
//read operations and 0 for Write operations. In I2C, the RW bit is part of the slave
//device address, and is taken care of by the protocol
//Also, the device returns a dummy byte before the data bytes, so this has to be removed.
//Hence chip_id and status are passed as 2 byte arrays in below code
int test_spi(void)
{
	int cnt =0;
	//generate Rx Edge on CS pin to put into SPI mode 
	coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_LOW);
	coines_delay_usec(800);
	coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
	coines_delay_usec(800);
    while (cnt++<10)
    {
		uint8_t chip_id[2];
		uint8_t soft_reset_cmd;
		uint8_t status[2];
		int rc;
		printf("Testing SPI API\n");
		coines_delay_usec(BMI270_POWER_ON_TIME);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BMI270_REG_CHIP_ID|BMI270_SPI_REG_READ_CMD, chip_id, 2);
		printf("dummy:%x chipid:%x rc%d\n",chip_id[0],chip_id[1],rc);

		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BMI270_REG_INTERNAL_STATUS|BMI270_SPI_REG_READ_CMD, status, 2);		
		printf("dummy %x internal status before reset:%x rc%d\n",status[0],status[1],rc);

		soft_reset_cmd = BMI270_CMD_SOFT_RESET;
		rc=coines_write_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1,BMI270_REG_CMD|BMI270_SPI_REG_WRITE_CMD, &soft_reset_cmd, 1);
		if (rc != 0) {
			printf("error doing bmi270 reset: rc%d\n",rc);
		}
		coines_delay_usec(BMI270_SOFT_RESET_TIME);
		rc=coines_read_spi(COINES_SPI_BUS_0,COINES_MINI_SHUTTLE_PIN_2_1, BMI270_REG_INTERNAL_STATUS|BMI270_SPI_REG_READ_CMD, status, 2);		
		printf("dummy %x internal status after reset:%x rc%d\n",status[0],status[1],rc);

		coines_delay_msec(3000);
	}
	puts("SPI test over");
    return 0;	
}
/*
 * Test Auxiliary SPI (OIS interface on BMI270)
 * Requires BMI270 shuttle board, requires the test app to be built using AB3 Board Revision 2
 * Uses primary I2C interface to first activate the Auxiliary OIS SPI interface
 */
#define BMI2_OIS_ADDR UINT8_C(0x0C)
#define BMI2_IF_CONF_ADDR  UINT8_C(0x6B)
#define BMI2_OIS_CONFIG_ADDR                  UINT8_C(0x40)
#define BMI2_OIS_IF_EN_MASK                       UINT8_C(0x10)
#define BMI2_AUX_IF_EN_MASK                       UINT8_C(0x20)
#define BMI2_OIS_IF_EN_POS                        UINT8_C(0x04)
#define BMI2_AUX_IF_EN_POS                        UINT8_C(0x05)
#define BMI2_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     ((data << bitname##_POS) & bitname##_MASK))
#define BMI2_SET_BIT_VAL0(reg_data, bitname)      (reg_data & ~(bitname##_MASK))
int test_aux_spi(void)
{
	int rc=0,i=0;
	uint8_t reg_data=0;
	uint8_t ois_data[13] ={0};
	//first activate OIS using the primary I2C
	//set CS and SDO pins for I2C enable and address
	coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_1,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH);
 	coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_3,COINES_PIN_DIRECTION_OUT,COINES_PIN_VALUE_HIGH); 
	coines_delay_usec(BMI270_POWER_ON_TIME);
	rc=coines_read_i2c(COINES_I2C_BUS_0,BMI270_I2C_ADDR, BMI2_IF_CONF_ADDR, &reg_data, 1);
	printf("REG IFCONF:%x rc%d\n",reg_data,rc);
	reg_data = BMI2_SET_BITS(reg_data, BMI2_OIS_IF_EN, 1);
	reg_data = BMI2_SET_BIT_VAL0(reg_data, BMI2_AUX_IF_EN);
	rc=coines_write_i2c(COINES_I2C_BUS_0,BMI270_I2C_ADDR,BMI2_IF_CONF_ADDR, &reg_data, 1);
	printf("REG IFCONF SET:%x rc%d\n",reg_data,rc);
	//config OIS SPI interface with CS pin
	coines_config_spi_bus(COINES_SPI_BUS_1, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE3);
    coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_2_5, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
	//ois set config -set both acc and gyr bits
	reg_data=0xC0;
	rc=coines_write_spi(COINES_SPI_BUS_1,COINES_MINI_SHUTTLE_PIN_2_5, BMI2_OIS_CONFIG_ADDR|BMI270_SPI_REG_WRITE_CMD, &reg_data, 1);
	reg_data =0;
	rc=coines_read_spi(COINES_SPI_BUS_1,COINES_MINI_SHUTTLE_PIN_2_5, BMI2_OIS_CONFIG_ADDR|BMI270_SPI_REG_READ_CMD, &reg_data, 1);
	printf("REG OIS CONF:%x rc%d\n",reg_data,rc);
	while(i++ <100)
	{
		coines_delay_msec(5000);
		rc=coines_read_spi(COINES_SPI_BUS_1,COINES_MINI_SHUTTLE_PIN_2_5, BMI2_OIS_ADDR|BMI270_SPI_REG_READ_CMD, ois_data, 13);
		printf("dummy %x ois_data1:%x %x %x %x %x %x\n",ois_data[0],ois_data[1],ois_data[2],ois_data[3],ois_data[4],ois_data[5],ois_data[6]);
		printf("ois_data2:%x %x %x %x %x %x  rc%d\n",ois_data[7],ois_data[8],ois_data[9],ois_data[10],ois_data[11],ois_data[12],rc);
	}
	puts("Aux SPI test over");
    return 0;	
}
#endif /*BOARD_BST_ARDUINO_NICLA*/

/*
 * Bluetooth BLE/BAS/BTS API test
 */
int test_ble(void)
{
	uint8_t buffer[100];
	uint8_t str[100];
	int len, istr=0;
	bool testover=false;
    struct coines_ble_config bleconfig = {
			.name = "APP3.0_BLE_INPUT",
			.tx_power = COINES_TX_POWER_0_DBM
	};
    coines_ble_config(&bleconfig);
	puts("Connect with nrf Toolkit or similar BLE console client.");
    coines_open_comm_intf(COINES_COMM_INTF_BLE,NULL);
	puts("Enter a text to echo in the BLE console terminated by ENTER,\n it will be echoed back to BLE console and printed here.\n Type quit[ENTER] to stop test");
    while (1)
    {
		len = coines_intf_available(COINES_COMM_INTF_BLE);
		if(len >0)
		{
			printf("%d bytes available on nus\n",len);
			len = coines_read_intf(COINES_COMM_INTF_BLE, buffer, sizeof(buffer));
			coines_write_intf(COINES_COMM_INTF_BLE, buffer, len);//echo back
			buffer[(len>=sizeof(buffer))?(sizeof(buffer)-1):len]=0;
			printf("%d bytes read: %s\n",len, buffer);
			for(int i =0;i<len;i++)
			{
				if(buffer[i]== '\r' || buffer[i]== '\n' || istr >= (sizeof(str)-1))
				{
					str[istr]=0;
					printf("input string:%s\n",str);
					istr =0;
					if(strcmp(str,"quit")  == 0)
						testover =true;
					break;
				}
				str[istr++] = buffer[i];
			}
			if(testover)
			break;
		}
		coines_delay_msec(100);//yield to allow BLE write thread to run
    }
	puts("BLE test over");
    coines_close_comm_intf(COINES_COMM_INTF_BLE,NULL);
    return 0;	
}
/*
 * USB CDC ACM API test
 */

int test_usb_cdc(void)
{
	uint8_t buffer[100];
	int len;

	puts("Enter a text to echo, type quit to stop test");
    while (1)
    {
		len = coines_intf_available(COINES_COMM_INTF_USB);
		if(len >0)
		{
			printf("%d bytes available on uart\n",len);
			len = coines_read_intf(COINES_COMM_INTF_USB, buffer, sizeof(buffer));
			coines_write_intf(COINES_COMM_INTF_USB, buffer, len);//echo back
			buffer[(len>=sizeof(buffer))?(sizeof(buffer)-1):len]=0;
			printf("%d bytes read: %s\n",len, buffer);
			if(strncmp(buffer,"quit",4)  == 0)
				break;
		}
    }
    return 0;	
}
/*
 * Battery & Temperature measurement API Test
 */
int test_bat_temp(void)
{
	int cnt =0;
	int err;
	uint16_t bat_status_mv;
	uint8_t bat_status_percent;
	float temp =0;
    while (cnt++ <10 )
    {
		err = coines_read_bat_status(&bat_status_mv, &bat_status_percent);
		if(err ==0)
			printf("Battery mv: %d Percent: %d\n",(int)bat_status_mv, (int)bat_status_percent);
		else 
			printf("Error %d reading battery level\n",err);
		err = coines_read_temp_data(&temp);
		if(err ==0)
		{
			int t1 = (int)temp;
			int t2 = (int)((temp-(float)t1)*100.0f);
			printf("Temperature: %d.%02dC\n",t1,t2);
		}
		else 
			printf("Error %d reading temperature\n",err);
		coines_delay_msec(2000);
    }
	puts("Test over");
    return 0;	
}
/*
 * GPIO Interrupt Handling API Test
 */
int ncb_1=0,ncb_2=0; 
void cb_1(uint32_t pin, uint32_t polarity)
{
	ncb_1++;
}
void cb_2(uint32_t pin, uint32_t polarity)
{
	ncb_2++;
}

int test_gpio_int()
{
	int ncb_1_o=0,ncb_2_o=0;
	ncb_1=0;ncb_2=0;
	coines_set_pin_config(COINES_APP30_BUTTON_1,COINES_PIN_DIRECTION_IN,COINES_PIN_VALUE_HIGH); 	  
	coines_attach_interrupt(COINES_APP30_BUTTON_1,cb_1,COINES_PIN_INTERRUPT_RISING_EDGE);
#if TEST_BOARD_BUTTONS >= 2
	coines_set_pin_config(COINES_APP30_BUTTON_2,COINES_PIN_DIRECTION_IN,COINES_PIN_VALUE_HIGH); 
	coines_attach_interrupt(COINES_APP30_BUTTON_2,cb_2,COINES_PIN_INTERRUPT_FALLING_EDGE);
#endif
	puts("Press Button 1 or 2 to trigger interrupts (5 times)");
	while(1)
	{
		if(ncb_1 != ncb_1_o)
		{
			printf("Button 1 pressed %d \n",ncb_1);
			ncb_1_o = ncb_1;
		}
		if(ncb_2 != ncb_2_o)
		{
			printf("Button 2 pressed %d \n",ncb_2);
			ncb_2_o = ncb_2;
		}
		if((ncb_1 + ncb_2) >5)
		{
			puts("GPIO test over");
			break;
		}
	}
	coines_detach_interrupt(COINES_APP30_BUTTON_1);
	coines_detach_interrupt(COINES_APP30_BUTTON_2);
	return 0;
}

/*
 * Timed GPIO Interrupt API Test (using PPI)
 */
uint64_t ns_p=0;
uint32_t pin_p=0;
void cb_3(uint64_t ns,uint32_t pin, uint32_t polarity)
{
	ncb_1++;
	ns_p = ns;
	pin_p = pin;
}
void cb_4(uint64_t ns,uint32_t pin, uint32_t polarity)
{
	ncb_2++;
	ns_p = ns;
	pin_p = pin;
}

int test_timed_gpio_int()
{
	int ncb_1_o=0,ncb_2_o=0;
	int err1=0,err2=0;
	ncb_1=0;ncb_2=0;
	ns_p=0;	  
	err1 = coines_attach_timed_interrupt(COINES_APP30_BUTTON_1,cb_3,COINES_PIN_INTERRUPT_RISING_EDGE);
#if TEST_BOARD_BUTTONS >= 2
	err2 = coines_attach_timed_interrupt(COINES_APP30_BUTTON_2,cb_4,COINES_PIN_INTERRUPT_FALLING_EDGE);	
#endif
	if(err1!=0 || err2 != 0)
	{
		printf("Error attaching interrupt %d %d\n",err1,err2);
		return 1;
	}
	puts("Press Button 1 and 2 to trigger interrupts (5 times)");
	while(1)
	{
		if(ncb_1 != ncb_1_o)
		{
			printf("Button 1 pressed at ns %lx%08lx pinid%d \n",((unsigned long) (ns_p>>32)), ((unsigned long)ns_p),pin_p);
			ncb_1_o = ncb_1;
		}
		if(ncb_2 != ncb_2_o)
		{
			printf("Button 2 pressed at ns %lx%08lx pinid%d\n",((unsigned long) (ns_p>>32)), ((unsigned long)ns_p),pin_p);
			ncb_2_o = ncb_2;
		}
		if((ncb_1 + ncb_2) >5)
		{
			puts("Timed Interrupt test over");
			break;
		}
	}
	coines_detach_timed_interrupt(COINES_APP30_BUTTON_1);
#if TEST_BOARD_BUTTONS >= 2
	coines_detach_timed_interrupt(COINES_APP30_BUTTON_2);
#endif
	return 0;
}
/*
 * Timer API Test
 */
int timer_cb_cnt=0;
void tim_cb(void)
{
	timer_cb_cnt++;
}
int test_timer_interrupt()
{
	int old_timer_cb_cnt=0;
	timer_cb_cnt=0;
	coines_timer_config(COINES_TIMER_INSTANCE_1,tim_cb);
	puts("Starting Timer test");
	coines_timer_start(COINES_TIMER_INSTANCE_1,3000000);// 3 sec
	while(1){
		if(timer_cb_cnt > old_timer_cb_cnt)
		{
			printf("Timer callback: %d\n",timer_cb_cnt);
			old_timer_cb_cnt = timer_cb_cnt;
		}
		if(timer_cb_cnt >10){
			coines_timer_stop(COINES_TIMER_INSTANCE_1);
			puts("stopping timer");
			break;
		}
		coines_delay_msec(1);
	}
	puts("Timer test over");
	return 0;
}

/*
 * System Commands API Test
 */
int test_sys_cmds(void)
{
	int counter=10;
	struct coines_board_info data;
	printf("COINES VERSION:%s \n",coines_get_version());
	coines_get_board_info(&data);
	printf("Board-id:%d HW-id:%d Shuttle-id:%d SW-id:%d\n",data.board,data.hardware_id,data.shuttle_id,data.software_id);
	puts("starting countdown");
	while(1)
	{
		printf("%d...\n",counter);
		coines_delay_msec(3000);
		if(--counter <0)
		{
			puts("Rebooting...");
			coines_soft_reset();
		}
	}
}
/*
 * File System API Test
 */
#if defined(CONFIG_FILE_SYSTEM_LITTLEFS)
#define FS_DRIVE "/lfs1"
#elif defined(CONFIG_FILE_SYSTEM_FLOGFS)
#define FS_DRIVE "/flog0"
#else
#define FS_DRIVE ""
#endif
/* Format-Remount function*/
void fs_format_remount(void);

#if defined(CONFIG_FILE_SYSTEM_LITTLEFS)
#include <zephyr/fs/littlefs.h>
#include <lfs.h>
#define PARTITION_NODE DT_NODELABEL(lfs1)
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);
void fs_format_remount(void)
{
	struct fs_mount_t *mp =	&FS_FSTAB_ENTRY(PARTITION_NODE);
	struct fs_littlefs *fs = mp->fs_data;
	int rc = lfs_format(&fs->lfs, &fs->cfg);
	printf("format done. Ret:%d\n",rc);
	lfs_mount(&fs->lfs, &fs->cfg);
	puts("Do warm reboot");
}

#elif defined(CONFIG_FILE_SYSTEM_FLOGFS)
#include <fs/flogfs_fs.h>
void fs_format_remount(void)
{
	int rc = flogfs_format_remount();
	printf("format done. Ret:%d\n",rc);
	puts("Do warm reboot");
}
#else
void fs_format_remount(void)
{
	printf("format remount not available\n");
}
#endif

static struct fs_dirent ent = { 0 };
struct fs_dir_t dir;
int print_dir(void)
{
	int rc;

	fs_dir_t_init(&dir);
	rc = fs_opendir(&dir, FS_DRIVE);
	printf("%s opendir: %d\n", FS_DRIVE, rc);

	while (rc >= 0) {

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			break;
		}
		if (ent.name[0] == 0) {
			printf("End of files\n");
			break;
		}
		printf("  %c %d %s\n",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       (int)ent.size,
		       ent.name);
	}
	(void)fs_closedir(&dir);
	return rc;
}

int clean_dir(void)
{
	int rc;
	char fpath[500];
	fs_dir_t_init(&dir);
	rc = fs_opendir(&dir, FS_DRIVE);

	while (rc >= 0) {

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			break;
		}
		if (ent.name[0] == 0) {
			break;
		}
		if(ent.type == FS_DIR_ENTRY_FILE)
		{
			sprintf(fpath,"%s/%s",FS_DRIVE,ent.name);
			fs_unlink(fpath);
			printf("%s deleted\n",fpath);
		}
	}
	(void)fs_closedir(&dir);
	return rc;
}

int print_dir_posix(void)
{
	DIR *dp;
  struct dirent *ep;     
  dp = opendir (FS_DRIVE);
  if (dp != NULL)
  {
    while ((ep = readdir (dp)) != NULL && ep->d_name[0]!=0)
      puts (ep->d_name);
          
    (void) closedir (dp);
    return 0;
  }
  else
  {
    perror ("Couldn't open the directory");
    return -1;
  }
}
static char rbuf[3000]={0};

void test_fs()
{
	FILE *f;
	int rc;
	puts("\nStarting POSIX FS Test\n");
	print_dir();
	f = fopen(FS_DRIVE "/posix3.txt","a");
	if(f== NULL)
	{
		puts("error opening file for writing");
	}
	else
	{
		int nbytes=0;
		for(int i=0;i<10;i++){
			rc = fprintf(f,"Hello posix %03d\n",i);
			if(rc <16) {
				printf("write error at %d bytes rc=%d\n",nbytes,rc);
				break;
			}
			else
				nbytes +=rc;
		}
		if(rc > 0)
				printf("%d bytes written without errors\n",nbytes);

		fputs("byebye posix 100",f);
		rc=fclose(f);
		printf("fclose after writing p1: %d\n",rc);
		f = fopen(FS_DRIVE "/posix3.txt","r");
		if(f== NULL)
		{
			puts("error opening file for reading");
		}
		else
		{
			char str1[10],str2[10];
			int d;
			while(fscanf(f,"%s %s %d",str1,str2,&d) == 3)
				printf("%s %s %d\n",str1,str2,d);
			rc=fclose(f);
			printf("fclose after reading p1: %d\n",rc);
		}
		
	}
	
	f = fopen(FS_DRIVE "/posix4.txt","a");
	if(f== NULL)
	{
		puts("error opening file2 for writing");
	}
	else
	{
		char *msg = "Hello again Posix 1\nHello again Posix 2\nHello again Posix 3\n";
		fwrite(msg,1,strlen(msg),f);
		rc = fclose(f);
		printf("fclose after writing p2: %d\n",rc);
		f = fopen(FS_DRIVE "/posix4.txt","r");
		if(f== NULL)
		{
			puts("error opening file2 for reading");
		}
		else
		{
			fread(rbuf,1,sizeof(rbuf),f);
			puts(rbuf);
			rc = fclose(f);
			printf("fclose after reading p2: %d\n",rc);

		} 
	}
	
	print_dir();
}
int shell_main(void);

void wait_loop()
{
	int len;
	char buf[100];
	puts("Press ENTER to continue");
		/*wait for user to press something*/
		while (1)
		{
			len = coines_intf_available(COINES_COMM_INTF_USB);
			if(len >0)
			{
				/*reject user input and continue test shell or batch*/
				coines_read_intf(COINES_COMM_INTF_USB, buf, sizeof(buf));
				break;
			}
			coines_delay_msec(1);
		}
}

int main()
{
	/*INTF USB is enabled for all tests (batch or shell), as it is
	required for console interaction*/
	coines_open_comm_intf(COINES_COMM_INTF_USB,NULL); 
	wait_loop();

	/*Uncomment to run test-drivers in an interactive console shell.*/
	shell_main();

	/*Uncomment one or more tests to run as a batch.*/
	// test_usb_cdc();			
	// test_ble();
	// test_i2c();		//Not valid for Nicla	
	// test_spi();		//different implementations for AB3 and Nicla
	// test_aux_spi();	//Not valid for Nicla, requires build with board revision 2	for AB3
	// test_bat_temp();	
	// test_gpio_led();			
	// test_gpio_int();
	// test_timed_gpio_int();
	// test_timer_interrupt();	
	// test_sys_cmds();			
	// test_fs();
	while(1)
	{
	}
}

/*Interactive test driver. The test is selected via console*/
int shell_main(void)
{
	int tn=0, len=0;
	char cmd[100];


	while(1)
	{
		puts(
			"1. test_usb_cdc\n"
			"2. test_ble\n"
			"3. test_i2c\n"
			"4. test_spi\n"
			"5. test_bat_temp\n"
			"6. test_gpio_led\n"
			"7. test_gpio_int\n"
			"8. test_timed_gpio_int\n"
			"9. test_timer_interrupt\n"
			"10. test_sys_cmds (with warm reboot)\n"
			"11. test_fs\n"
			"12. fs_format_remount\n"
			"13. print_dir\n"
			"14. clean_dir\n"
			"100. Quit TEST Application\n"
			"Enter Test no:"
			);
		tn =0;
		//wait for input from user
		while (1)
		{
			len = coines_intf_available(COINES_COMM_INTF_USB);
			if(len >0)
			{
				coines_read_intf(COINES_COMM_INTF_USB, cmd, sizeof(cmd));
				if(sscanf(cmd,"%d",&tn) >0)
					break;
				else
					printf("Unexpected input %s\n",cmd);
					
			}
			coines_delay_msec(1);
		}
   
		switch(tn)
		{
			case 1:
				test_usb_cdc();
				break;
			case 2:
				test_ble();
				break;
			case 3:
				test_i2c();
				break;
			case 4:
				test_spi();
				break;
			case 5:
				test_bat_temp();
				break;
			case 6:
				test_gpio_led();
				break;
			case 7:
				test_gpio_int();
				break;
			case 8:
				test_timed_gpio_int();
				break;
			case 9:
				test_timer_interrupt();
				break;
			case 10:
				test_sys_cmds();
				break;
			case 11:
				test_fs();
				break;
			case 12:
				fs_format_remount();
				break;
			case 13:
				print_dir();
				break;
			case 14:
				clean_dir();
				break;
			case 100:
				puts("Quitting Test app");
				return 0;
			default:
				printf("Unknown test number %d\n",tn);
				break;
		}
		 wait_loop();
	}
	return 0;
}
