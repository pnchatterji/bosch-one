/*
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

/*TIP: This template file is actually working code. It shows a minimal support for
BMI270 (acclerometer data only). Some of the code inside various functions pertains
to BMI270, and needs to be replaced for BXX */

#define DT_DRV_COMPAT bosch_bxx
#include <devicetree.h>
#include <init.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <pm/device.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <stdio.h>
LOG_MODULE_REGISTER(bxx, CONFIG_SENSOR_LOG_LEVEL);

/*TIP: include the header file with BXX specific type declarations and constants here.
If there are only a small number of declarations, the declarations 
placed directly in bxx.c and bxx.h can be removed*/
#include "bxx.h"
/*TIP: If bxx.h contains a Raw API definition, for accessing extra features not covered by
sensor.h, it needs to be placed in zephyr/include/sensor so that it is publicly visible.
In that case, it needs to be included here using angle brackets, like this:
#include <bxx.h>
*/
/*TIP: In this example, we are directly placing BXX specific type declarations and 
constants below, avoiding the need for a separate bxx.h */
enum bxx_err_codes {
	BXX_OK=0,
	BXX_E_IO,
	BXX_E_TIMEOUT,
	BXX_E_NULL_PTR,
	BXX_E_INVALID_PARAM,
};
#define BXX_PRODUCT_ID 0x24 	/*TIP:Change this to the actual device product ID. This is the BMI270 ID */
/*Register IDs*/
/*TIP: Change below codes to the corresponding BXX device codes. The below
codes have been taken from BMI270 for testing the template. Add any 
additional codes required*/
#define BXX_SPI_REG_READ_CMD 0x80
#define BXX_SPI_REG_WRITE_CMD 0x00
#define BXX_REG_PRODUCT_ID 0x00
#define BXX_REG_CMD		   0x7E
#define BXX_CMD_SOFT_RESET 0xB6
#define BXX_REG_ACC		   0x0C

/*COINES_SPI_MODE0 SPI Mode 0: CPOL=0; CPHA=0*/
#define SPI_OPERATION_MODE0 (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_TRANSFER_MSB)
/*COINES_SPI_MODE1 SPI Mode 1: CPOL=0; CPHA=1*/
#define SPI_OPERATION_MODE1 (SPI_OPERATION_MODE0 | SPI_MODE_CPHA )
/*COINES_SPI_MODE2 SPI Mode 2: CPOL=1; CPHA=0*/
#define SPI_OPERATION_MODE2 (SPI_OPERATION_MODE0 | SPI_MODE_CPOL)
/*COINES_SPI_MODE3 SPI Mode 3: CPOL=1; CPHA=1*/
#define SPI_OPERATION_MODE3 (SPI_OPERATION_MODE0 | SPI_MODE_CPOL | SPI_MODE_CPHA )

#define SPI_DELAY 200U
#define SPI_OPERATION_MODE SPI_OPERATION_MODE0

/*bxx data and config structure definitions. This is required by the Zephyr driver
framework. The elements can be assigned on need basis*/

struct bxx_data {
	/*TIP: add BXX specific driver variable data elements.
	If the baremetal driver has an instance data structure, 
	that is a good candidate for inserting here. 
	e.g.
	struct bxx_dev bxxdev;
	It can then be accessed
    in the API functions via dev->data.bxxdev	
	Other variables required on instance basis (one copy per
	device instance) should also be inserted here*/
	int16_t ax, ay, az; /*TIP: example variables, to be removed*/
	uint8_t acc_range; /*TIP: example variables, to be removed*/
	/*TIP: This structure is most useful as temporary storage for fetched values
	and attribute settings
	Here, acceleration values and range are being stored for BMI270 which is 
	being used as an example. Refer the sensor_fetch,channel_get and attr_set
	Sensor API functions 
	*/
	/*TIP: the data items required for trigger interrupt are also stored
	here. Remove if BXX does not support interrupts*/
#if defined(CONFIG_BXX_TRIGGER)
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t trig_handler;
	enum sensor_trigger_type trig_type;
	struct k_work work;
#endif /* CONFIG_BXX_TRIGGER */
};

struct bxx_cfg {
	bool use_spi;						/*Use SPI or I2C flag*/
	struct spi_dt_spec bus_s;			/*SPI device used by BXX*/
	struct i2c_dt_spec bus_i;			/*I2C device used by BXX*/
	struct gpio_dt_spec cs_gpios;		/*gpio pin connected to cs pin of BXX*/
	struct gpio_dt_spec hirq_gpios;		/*gpio pin connected to interrupt pin of BXX*/
	struct gpio_dt_spec addr_gpios;		/*gpio pin connected to SDO pin (I2C Address Selector) of BXX*/
	uint8_t addr_state;					/*desired state of address selector pin*/
	/*TIP: add BXX specific driver config elements. They can be initialized in the 
	driver definition macro at the end of the file, either as constants or as
	values taken defined in the DTS*/

};

/*TIP: convert baremetal driver error code (if any) to Zypher Error code
for convenience, assuming code from a baremetal version of the driver is being
reused*/
const int bxxerr_to_errno(int8_t error_code)
{
    switch (error_code)
    {
        case BXX_OK:
			return 0;
        case BXX_E_IO:
            return -EIO;
		case BXX_E_TIMEOUT:
			return -ETIMEDOUT;
        case BXX_E_NULL_PTR:
        case BXX_E_INVALID_PARAM:
            return -EINVAL;
        default:
            return -EFAULT;
    }
}
/*TIP: The communication functions below perform i2c/spi communication with the BXX device,
 as configured in the DTS. They should be used in the driver code for communicating with BXX.
 If you are copy-pasting existing baremetal code, the communication function in it should
 be replaced with reg_read, reg_write defined below.
If the BXX device supports only I2C or only SPI, the communication code for the unused protocol
can be deleted to avoid confusion. The unnecessary elements in struct bxxcfg should also be
removed
*/
static int reg_read_i2c(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
	const struct bxx_cfg *cfg = dev->config;

	int rc = i2c_burst_read_dt(&cfg->bus_i, reg, data, length);
	if(rc!=0)   {
        i2c_recover_bus(cfg->bus_i.bus);//attempt to recover bus if tx failed for any reason
    }
	return rc;
}

static int reg_write_i2c(const struct device *dev, uint8_t reg, const uint8_t *data, uint16_t length)
{
	const struct bxx_cfg *cfg = dev->config;

	int rc = i2c_burst_write_dt(&cfg->bus_i, reg, data, length);
	if(rc!=0)   {
        i2c_recover_bus(cfg->bus_i.bus);//attempt to recover bus if tx failed for any reason
    }
	return rc;
}

/*TIP: IMPORTANT ISSUE !
In most BST sensor devices, the reg address bit 8 has to be forced to 1
for SPI read operations. Also, the first received byte is a dummy byte. The 
following function automatically takes care of this. In case this is also being
taken care of at a higher level in the bare-metal driver, the correction code 
needs to be removed from there, or from here. Otherwise it will lead to an error.
This is not the case for I2C 
*/

static int reg_read_spi(const struct device *dev,uint8_t reg_addr_raw, uint8_t *reg_data, uint32_t length)
{
	const struct bxx_cfg *cfg = dev->config;
	const struct spi_dt_spec *spidev = &cfg->bus_s;
	uint8_t reg_data2[length+1];
	/*Add SPI read-bit to register address*/
	uint8_t reg_addr = reg_addr_raw | BXX_SPI_REG_READ_CMD;
	int rc1 =0,rc2=0;
    const struct spi_buf tx_buf = {
		.buf = &reg_addr,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
    struct spi_buf rx_buf = {
        .buf = reg_data2,
        .len = length+1,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};
	/*manual control of CS as auto-control does not work here*/
	gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_INACTIVE); 
	rc1 = spi_write_dt(spidev, &tx);
	rc2 = spi_read_dt(spidev, &rx);
	/*separate read and write are required here. spi_transcive_dt does not work*/
	gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_ACTIVE); 
	if(rc1 || rc2)
	{
		LOG_ERR("SPI communication error write:%d read%d\n",rc1,rc2);
		return -EIO;
	}
	else
	{
		/*throw away dummy first byte*/
		memcpy(reg_data,&reg_data2[1],length);
	}
	return 0;
}

static int reg_write_spi(const struct device *dev,uint8_t reg_addr_raw, const uint8_t *reg_data, uint32_t length)
{
	const struct bxx_cfg *cfg = dev->config;
	const struct spi_dt_spec *spidev = &cfg->bus_s;
	uint8_t reg_addr = reg_addr_raw | BXX_SPI_REG_WRITE_CMD;
    uint8_t reg_data_cpy[length];
	int rc=0;
	const struct spi_buf tx_bufs[] = {
        {
            .buf = &reg_addr,
            .len = 1,
        },
        {
            .buf = reg_data_cpy,
            .len = length,
        }
    };

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 2,
	};

    /*make local copy of regdata, as SPI peripheral requires buffer to be in RAM data section
      and it cannot be certain what type of buffer the user is passing*/
    memcpy(reg_data_cpy,reg_data,length);
	gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_INACTIVE); 
	rc = spi_write_dt(spidev, &tx);
	gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_ACTIVE);
	if(rc)
	{
		LOG_ERR("SPI communication error write:%d",rc);
		return -EIO;
	}
    return 0;
}

static inline int reg_write(const struct device *dev, uint8_t reg, const uint8_t *data, uint16_t length)
{
	int ret = 0;
	const struct bxx_cfg *cfg = dev->config;
	if(cfg->use_spi)
		ret = reg_write_spi(dev, reg, data, length);
	else 
		ret = reg_write_i2c(dev, reg, data, length);
	return ret;
}

static inline int reg_write_with_delay(const struct device *dev, uint8_t reg, const uint8_t *data,
				uint16_t length, uint32_t delay_us)
{
	int ret = 0;

	ret = reg_write(dev, reg, data, length);
	if (ret == 0) {
		k_usleep(delay_us);
	}
	return ret;
}
static inline int reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
	int ret = 0;
	const struct bxx_cfg *cfg = dev->config;
	if(cfg->use_spi)
		ret = reg_read_spi(dev, reg, data, length);
	else 
		ret = reg_read_i2c(dev, reg, data, length);
	return ret;
}
/**
 * @brief poll BXX interrupt pin GPIO
 * TIP: if the interrupt pin of the BXX device needs to be polled, this
 * function can be used. If the interrupt pin is configured for hardware
 * interrupt (see trigger code), or is not not available/not desired, the
 * below function can be deleted.
 */
static bool inline bxx_get_interrupt_status(const struct device *dev)
{
	const struct bxx_cfg *cfg = dev->config;
	return (cfg->hirq_gpios.port ==NULL)? false:(gpio_pin_get_dt(&cfg->hirq_gpios)==1);
}

#if defined(CONFIG_BXX_TRIGGER)
/**
 * @brief callback of interrupt pin, if trigger is enabled by CONFIG_BXX_TRIGGER 
 */
static void bxx_hirq_callback(const struct device *dev,
				 struct gpio_callback *cb, uint32_t pins)
{
	struct bxx_data *bxxdata =
		CONTAINER_OF(cb, struct bxx_data, gpio_cb);
	const struct bxx_cfg *cfg = bxxdata->dev->config;

	if ((pins & BIT(cfg->hirq_gpios.pin)) == 0U) {
		return;
	}

	gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_DISABLE);
	/*delegate actual trigger handling to kernel main thread*/
	k_work_submit(&bxxdata->work);
}
/**
 * @brief callback from kernel main thread if trigger interrupt is handled 
 */
static void bxx_work_cb(struct k_work *work)
{
	struct bxx_data *bxxdata =
		CONTAINER_OF(work, struct bxx_data, work);
	const struct device * dev = bxxdata->dev;
	const struct bxx_cfg *cfg = dev->config;
	struct sensor_trigger bxx_trig = {
		.type = bxxdata->trig_type,
		.chan = SENSOR_CHAN_ALL,
	};
	/*call handler registered by application using sensor_trigger_set*/
	if (bxxdata->trig_handler) {
		bxxdata->trig_handler(dev, &bxx_trig);
	}
	gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_EDGE_TO_ACTIVE);
}
/**
 * @brief do all initialization necessary to enable trigger interrupts,
 * in case it is activated. This function assumes int pin is previously
 * initialized.
 */
static int bxx_init_interrupt(const struct device *dev)
{
	const struct bxx_cfg *cfg = dev->config;
	struct bxx_data *bxxdata = dev->data;
	bxxdata->dev = dev;
	bxxdata->work.handler = bxx_work_cb;
	gpio_init_callback(&bxxdata->gpio_cb,
						bxx_hirq_callback,	
						BIT(cfg->hirq_gpios.pin));
	gpio_add_callback(cfg->hirq_gpios.port, 
					&bxxdata->gpio_cb);
	int rc = gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_EDGE_TO_ACTIVE);
	if(rc == 0)
	{
		/*TIP: Add any necessary code to configure and initialize the interrupt
		generation on BXX device. The above code only initializes the host CPU
		interrupt handling (i.e. the interrupt consumption)*/
	}
	return rc;
}
#endif

static int bxx_init_pins(const struct device *dev)
{
	int rc1=0,rc2=0,rc3=0;
	const struct bxx_cfg *cfg = dev->config;

	if(cfg->use_spi) /*SPI Settings*/
	{
		struct spi_dt_spec *spidev = (struct spi_dt_spec *)&cfg->bus_s;
		if(!spi_is_ready(spidev))
		{
			LOG_ERR("SPI is not ready\n");
			return -EIO;
		}
		/*remove const to allow manipulation*/
		struct spi_cs_control *cs= (struct spi_cs_control *)(spidev->config.cs);
		/*inhibit CS auto-control in order to control the CS pin directly,
		as auto-control algorithm does not appear to work with BST devices*/
		cs->gpio.port =NULL;
		/*create rising edge on CS pin to force BXX into SPI mode*/ 
		rc1 = gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_INACTIVE);
		k_msleep(3);
		rc1 = gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_ACTIVE);
		/*BXX switches to SPI a delay after the rising edge*/
		k_msleep(3);
	}
	else /*I2c settings*/
	{
		/*If CS pin (csm_gpios) is defined for I2C, pull it high to force
		BXX into I2C mode. Otherwise it is assumed the CS pin of BXX is
		wired to VDD on the board to be always in I2C mode*/
		if(cfg->cs_gpios.port !=NULL)
			rc1 = gpio_pin_configure_dt(&cfg->cs_gpios, GPIO_OUTPUT_ACTIVE);
		/*If I2C address pin (aka SDO of SPI) is defined, pull it high or low
		as specified in the DTS. Otherwise it is assumed that the SDO pin
		is hardwired to VDD or GND, as desired. The address of I2C device 
		specified in the DTS should correspond to the SDO pin setting*/
		if(cfg->addr_gpios.port !=NULL)
			rc2 = gpio_pin_configure_dt(&cfg->addr_gpios, 
					((cfg->addr_state)?GPIO_OUTPUT_ACTIVE:GPIO_OUTPUT_INACTIVE));
		/*give some time for BXX power on delay*/
		k_msleep(3);
	}
	/* If the interrupt pin is defined, configure hirq as a pull-down. BXX 
	operates the interrupt pin as an active high, level, push-pull by default */
	if(cfg->hirq_gpios.port !=NULL) 
	{
	    rc3 = gpio_pin_configure_dt(&cfg->hirq_gpios,(GPIO_INPUT|GPIO_PULL_DOWN)); 
	}
	if(rc1 || rc2 || rc3)
	{
		LOG_ERR("Error initializing BXX pins");
		return -EINVAL;
	}
	return 0;
}

/**
 * @brief sample fetch
 * Fetches sensor data from BXX sensor and stores in temporary
 * storage until read by the application using sensor_channel_get
 * @param dev 
 * @param chan 
 * @return errno
 */
static int bxx_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct bxx_data *drv_dev = dev->data;
	uint8_t data[6];
	int ret;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	ret = reg_read(dev, BXX_REG_ACC, data, 6);
	if (ret == 0) {
		drv_dev->ax = (int16_t)sys_get_le16(&data[0]);
		drv_dev->ay = (int16_t)sys_get_le16(&data[2]);
		drv_dev->az = (int16_t)sys_get_le16(&data[4]);
	} else {
		drv_dev->ax = 0;
		drv_dev->ay = 0;
		drv_dev->az = 0;
	}

	return ret;
}

/*TIP: Data conversion function required by the BMI270 example. Something
similar may be required for BXX*/
static void channel_accel_convert(struct sensor_value *val, int64_t raw_val,
				  uint8_t range)
{
	/* 16 bit accelerometer. 2^15 bits represent the range in G */
	/* Converting from G to m/s^2 */
	raw_val = (raw_val * SENSOR_G * (int64_t) range) / INT16_MAX;

	val->val1 = raw_val / 1000000LL;
	val->val2 = raw_val % 1000000LL;

	/* Normalize val to make sure val->val2 is positive */
	if (val->val2 < 0) {
		val->val1 -= 1LL;
		val->val2 += 1000000LL;
	}
}
/**
 * @brief sensor value get
 * Takes temporary raw data stored by fetch, and returns it to application
 * after appropriate conversion.
 * TIP: In this example, raw acceleration data from BMI270 is converted into the
 * form specified by Zephyr Sensor API. Modify this code as appropriate for BXX 
 * @return errno
 */
static int bxx_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct bxx_data *drv_dev = dev->data;

	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		channel_accel_convert(&val[0], drv_dev->ax,
				      drv_dev->acc_range);
		channel_accel_convert(&val[1], drv_dev->ay,
				      drv_dev->acc_range);
		channel_accel_convert(&val[2], drv_dev->az,
				      drv_dev->acc_range);
	} else {
		return -ENOTSUP;
	}
	return 0;
}
/**
 * @brief Set the range of bxx device
 * 
 * @param dev 
 * @param range 
 * @return errno
 */
static int set_bxx_range(const struct device *dev, const struct sensor_value *range)
{
	/*TIP: dummy function. Usually, this will be implemented by making the
	appropriate reg_write register calls to set sensor range  (or by calling 
	the appropriate baremetal driver functions which will do this)*/
	return 0;
}

/**
 * @brief API function for setting BXX device attributes
 * TIP: only one attribute is shown as an example. Usually, there will be many
 * in a real driver
 * @param dev 
 * @param chan 
 * @param attr 
 * @param val 
 * @return errno
 */
int bxx_attr_set(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	int ret = -ENOTSUP;

	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		switch (attr) {
		case SENSOR_ATTR_FULL_SCALE:
			ret = set_bxx_range(dev, val);
			break;
		default:
			ret = -ENOTSUP;
		} 
	} else {
		ret = -ENOTSUP;
	}
	return ret;
}

int bxx_attr_get(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 struct sensor_value *val)
{
	/*TIP: not implemented. attributes can either be cached in BXX data structure
	when attr_set is done, or retrieved from the BXX device by appropriate commands*/
	return -ENOTSUP;
}

int bxx_enable_data_ready_intr()
{
	/*TIP: This is a placeholder function. If BXX supports hardware interrupts on data 
	ready, making appropriate BXX register settings here for enabling this feature on
	BXX. Similarly, for other sensor events that can trigger the hardware interrupt.*/
	return 0;
}

/**
 * @brief Set trigger callbacks
 * TIP: If BXX supports hardware interrupts on sensor events, implement
 * the below case statement to support the available event types.
 * An example for the "Data Ready" event is provided below.
 * Remove the example code within ifdef/endif if BXX does not support 
 * hardware interrupts
 * @param dev 
 * @param trig 
 * @param handler 
 * @return errno 
 */
int bxx_trigger_set(const struct device *dev,
				    const struct sensor_trigger *trig,
				    sensor_trigger_handler_t handler)
{	
	struct bxx_data *bxxdata = dev->data;
	const struct bxx_cfg *cfg = dev->config;
	if (!cfg->hirq_gpios.port) {
		return -ENOTSUP;
	}

	switch (trig->type) {
#if defined(CONFIG_BXX_TRIGGER)
	case SENSOR_TRIG_DATA_READY:
		bxxdata->trig_handler = handler;
		bxxdata->trig_type = SENSOR_TRIG_DATA_READY;
		int rc = bxx_enable_data_ready_intr();
		return rc;
#endif
	default:
		LOG_ERR("Unsupported sensor trigger");
		return -ENOTSUP;
	}
}

/*TIP: BXX internal functions. This can be adapted from
existing baremetal drivers by making use of above defined
reg read/write functions for communication*/


/**
 *@brief get BXX device product ID
 */
static int  bxx_get_product_id(const struct device *dev,uint8_t *product_id){
	return reg_read(dev, BXX_REG_PRODUCT_ID, product_id, 1);
}

/**
 *@brief soft reset of BXX device
 */
static int bxx_soft_reset(const struct device *dev) {
	uint8_t soft_reset_cmd = BXX_CMD_SOFT_RESET;
	return reg_write_with_delay(dev, BXX_REG_CMD, &soft_reset_cmd, 1,10);
}

/**
 * @brief Initialize BXX
 * TIP: This function is automatically called by the Zephyr OS when the driver
 * is initialized after Kernel Boot, but before the application starts.
 * It can be used to perform any initialization required by the BXX device(s)
 */

int bxx_init(const struct device *dev)
{
    uint8_t product_id = 0;
	
	if(bxx_init_pins(dev))
	{
		LOG_ERR("Unable to initialize CS, HIRQ and Reset pins");
		return -EINVAL;
	}
#if defined(CONFIG_BXX_TRIGGER)
	if(bxx_init_interrupt(dev))
	{
		LOG_ERR("Unable to initialize trigger interrupt");
		/*non-fatal error, continue initialization*/
	}
#endif		
    bxx_get_product_id(dev,&product_id);
    if (product_id != BXX_PRODUCT_ID)
    {
        LOG_ERR("Aborting init: Unexpected Product ID %x. Expected %x", product_id, BXX_PRODUCT_ID);
		return -EFAULT;
    }
	bxx_soft_reset(dev);
	/*TIP: optimize sleep time after reset for each device*/
	k_msleep(3);/*pause until BXX reset*/
	/*TIP: Do any other device specific initialization here. If necessary, define
	additional DTS initialization parameters to pass to the BXX device.
	To add an init parameter, it has to be added to: 
	 1. The binding yaml files dts/binding/bxx-i2c/bosch,bxx-*.yaml
	 2. the sample DTS overlay files sample/board/<board>.overlay
	 3. in the device config structure bxx_config
	 4. in the below init macro BXX_INIT*/
	LOG_INF("Device Bxx initialized");
	return 0;
}

static const struct sensor_driver_api bxx_api = {
	.sample_fetch = bxx_sample_fetch,
	.channel_get = bxx_channel_get,
	.attr_set = bxx_attr_set,
	.attr_get = bxx_attr_get,
	.trigger_set = bxx_trigger_set,
};

/*TIP: The below macro initializes all instances of a device that are defined
in the DTS and creates an instance of data and cfg structure for each device 
instance. Modify the macro to add initialization for any additional
bxx_cfg or bxx_data structure elements that have been added above*/

#define BXX_SPI(inst)  (                               					\
	.bus_s = SPI_DT_SPEC_INST_GET(inst, SPI_OPERATION_MODE, SPI_DELAY),	\
 	.cs_gpios = SPI_CS_GPIOS_DT_SPEC_INST_GET(inst),) 

#define BXX_I2C(inst) (													\
	.bus_i = I2C_DT_SPEC_INST_GET(inst), 								\
 	.cs_gpios = GPIO_DT_SPEC_INST_GET_OR(inst,csm_gpios, { 0 }), 		\
 	.addr_gpios = GPIO_DT_SPEC_INST_GET_OR(inst,addr_gpios, { 0 }), 	\
	.addr_state = DT_INST_PROP_OR(inst,addr_state,0),					\
	) 

#define BXX_INIT(index)													\
																		\
static struct bxx_data bxx_data_##index = {0};							\
																		\
const static struct bxx_cfg bxx_cfg_##index = {							\
	COND_CODE_1(DT_INST_ON_BUS(index, spi), BXX_SPI(index), ())			\
	COND_CODE_1(DT_INST_ON_BUS(index, i2c), BXX_I2C(index), ())			\
 	.use_spi = DT_INST_ON_BUS(index,spi),								\
 	.hirq_gpios = GPIO_DT_SPEC_INST_GET_OR(index, hirq_gpios,{0}),		\
};																		\
																		\
DEVICE_DT_INST_DEFINE(index, &bxx_init, NULL, 							\
		      &bxx_data_##index,&bxx_cfg_##index, 						\
			  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,					\
		      &bxx_api);

DT_INST_FOREACH_STATUS_OKAY(BXX_INIT)