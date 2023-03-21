/*
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

/* Bosch Sensortec BHY2 Sensor API includes*/
#include <bhy2.h>
#define BHY2_RD_WR_LEN          256    /* MCU maximum read write length */

/*Zephyr includes*/
#define DT_DRV_COMPAT bosch_bhix60

#include <drivers/spi.h>
#include <init.h>
#include <drivers/sensor.h>
#include <pm/device.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <stdio.h>
LOG_MODULE_REGISTER(bhix60, CONFIG_SENSOR_LOG_LEVEL);

/*This driver's extended (public) API*/
#include "bhix60.h"
/*This driver's internal interface*/
#include "bhix60_int.h"

/*Work buffer required by the FIFO parsing engine*/
#define BHIX60_WORK_BUFFER_SIZE  2048
/*Max Read write lengths in SPI communication*/
#define BHY2_RD_WR_LEN          256 

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

#if defined(CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD) || defined(CONFIG_BHIX60_FIFO_INT_FIFO_THREAD)
	#define BHIX60_FIFO_INT 1
#endif 

static int bhix60_poll_fifo(const struct device *dev);
static int bhix60_proc_fifo(const struct device *dev);

struct bhix60_data_slot {
	uint8_t sensor_id;
	uint8_t size;
	uint8_t *data;
	uint64_t time_stamp;
	uint16_t range;
	struct k_mutex slock;
	/*trigger related elements*/
	struct sensor_trigger trigs[CONFIG_BHIX60_MAX_SLOT_TRIGGERS];
	sensor_trigger_handler_t thandlers[CONFIG_BHIX60_MAX_SLOT_TRIGGERS];
	int32_t trig_threshold_upper;
	int32_t trig_threshold_lower;
	bool trig_threshold_fired;
	int ntrigs;
};

struct bhix60_data {
	struct bhy2_dev bhy2;
	struct bhix60_data_slot slots[CONFIG_BHIX60_VIRTUAL_SENSORS_USED];
	uint16_t nslots;
	uint8_t work_buffer[BHIX60_WORK_BUFFER_SIZE];
#if defined(BHIX60_FIFO_INT)
	const struct device *dev;
	struct gpio_callback gpio_cb;
#if defined(CONFIG_BHIX60_FIFO_INT_FIFO_THREAD)
	K_KERNEL_STACK_MEMBER(fifo_thread_stack, CONFIG_BHIX60_FIFO_THREAD_STACK_SIZE);
	struct k_thread fifo_thread;
	struct k_sem fifo_sem;
#elif defined(CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD)
	struct k_work work;
#endif /* CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD */
#endif /* BHIX60_FIFO_INT */
};

struct bhix60_cfg {
	struct spi_dt_spec bus;				/*SPI device used by BHIX60*/
	struct gpio_dt_spec cs_gpios;		/*gpio pin connected to cs pin of BHIX60*/
	struct gpio_dt_spec reset_gpios;	/*gpio pin connected to RESET pin of BHIX60*/
	struct gpio_dt_spec hirq_gpios;		/*gpio pin connected to HIRQ pin of BHIX60*/
};

const int bhix60_api_to_errno(int8_t error_code)
{
    switch (error_code)
    {
        case BHY2_OK:
			return 0;
        case BHY2_E_IO:
            return -EIO;
		case BHY2_E_TIMEOUT:
			return -ETIMEDOUT;
        case BHY2_E_NULL_PTR:
        case BHY2_E_INVALID_PARAM:
        case BHY2_E_MAGIC:
        case BHY2_E_BUFFER:
        case BHY2_E_INVALID_FIFO_TYPE:
        case BHY2_E_INVALID_EVENT_SIZE:
        case BHY2_E_PARAM_NOT_SET:
            return -EINVAL;
        default:
            return -EFAULT;
    }
}

struct bhy2_dev *bhix60_get_bhy2_dev(const struct device *dev)
{
    return &((struct bhix60_data *)(dev->data))->bhy2;
}

int16_t  bhix60_chan_to_sid(enum sensor_channel chan)
{

	if((int)chan > (int)SENSOR_CHAN_BHIX60_START &&
      (int)chan < (int)SENSOR_CHAN_BHIX60_END)
	{
		/*BHY2 extended channel range, standard, special or custom*/
		return ((int16_t)chan - (int16_t)SENSOR_CHAN_PRIV_START);
	}
	else
	{
		/*Zephyr standard sensor range: attempt mapping*/
		/*typecast to int as both standard and extension channels are 
		being processed, to avoid warnings*/
		switch((int)chan)
		{
			case SENSOR_CHAN_ACCEL_XYZ: 
				return BHY2_SENSOR_ID_ACC;
			case SENSOR_CHAN_GYRO_XYZ: 
				return BHY2_SENSOR_ID_GYRO;	
			case SENSOR_CHAN_MAGN_XYZ: 
				return BHY2_SENSOR_ID_MAG;
			case SENSOR_CHAN_AMBIENT_TEMP: 
				return BHY2_SENSOR_ID_TEMP;
			case SENSOR_CHAN_PROX: 
				return BHY2_SENSOR_ID_PROX;
			case SENSOR_CHAN_HUMIDITY: 
				return BHY2_SENSOR_ID_HUM;
			case SENSOR_CHAN_LIGHT: 
				return BHY2_SENSOR_ID_LIGHT;
			case SENSOR_CHAN_GAS_RES: 
				return BHY2_SENSOR_ID_GAS;
			case SENSOR_CHAN_PRESS:
				return BHY2_SENSOR_ID_BARO;
			case SENSOR_CHAN_ROTATION_QTR:
				return BHY2_SENSOR_ID_RV;
			case SENSOR_CHAN_GAME_ROTATION_QTR:
				return BHY2_SENSOR_ID_GAMERV;
			case SENSOR_CHAN_GEOMAG_ROTATION_QTR:
				return BHY2_SENSOR_ID_GEORV;
			case SENSOR_CHAN_ORIENTATION_HPR:
				return BHY2_SENSOR_ID_ORI;
			case SENSOR_CHAN_GRAVITY_XYZ:
				return BHY2_SENSOR_ID_GRA;
			case SENSOR_CHAN_LINEAR_ACCEL_XYZ:
				return BHY2_SENSOR_ID_LACC;
			case SENSOR_CHAN_STEP_CNT:
				return BHY2_SENSOR_ID_STC;
  			case SENSOR_CHAN_DEVICE_ORIENTATION:
				return BHY2_SENSOR_ID_ORI;
			case SENSOR_CHAN_CAMERA_SHUTTER:
				return BHY2_SENSOR_ID_EXCAMERA;
			/*Event type channels*/
			case SENSOR_CHAN_SIG_MOTION:
				return BHY2_SENSOR_ID_SIG;
			case SENSOR_CHAN_STEP:
				return BHY2_SENSOR_ID_STD;
			case SENSOR_CHAN_TILT:
				return BHY2_SENSOR_ID_TILT_DETECTOR;
			case SENSOR_CHAN_WAKE_GEST:
				return BHY2_SENSOR_ID_WAKE_GESTURE;
			case SENSOR_CHAN_GLANCE_GEST:
				return BHY2_SENSOR_ID_GLANCE_GESTURE;
			case SENSOR_CHAN_PICKUP_GEST:
				return BHY2_SENSOR_ID_PICKUP_GESTURE;
			case SENSOR_CHAN_WRIST_TILT_GEST:
				return BHY2_SENSOR_ID_WRIST_TILT_GESTURE;
			case SENSOR_CHAN_STATIONARY:
				return BHY2_SENSOR_ID_STATIONARY_DET;
			case SENSOR_CHAN_MOTION:
				return BHY2_SENSOR_ID_MOTION_DET;
			default: 
				return -ENOTSUP;			/*mapping not supported*/
		}
	}
}

/*find the slot of a registered sensor*/
static inline struct bhix60_data_slot *bhix60_get_slot(struct bhix60_data *devdata,uint16_t sensor_id)
{
	for(int i=0;i<devdata->nslots;i++)
	{
		if(devdata->slots[i].sensor_id == sensor_id) {
			return &devdata->slots[i];
		}
	}
	return NULL;
}

int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	const struct device *bhidev =intf_ptr;
	const struct bhix60_cfg *bhicfg = bhidev->config;
	const struct spi_dt_spec *spidev = &bhicfg->bus;
	int rc1 =0, rc2=0;
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
        .len = length,
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};
	gpio_pin_configure_dt(&bhicfg->cs_gpios, GPIO_OUTPUT_INACTIVE); 
	rc1 = spi_write_dt(spidev, &tx);
	rc2 = spi_read_dt(spidev, &rx);
	gpio_pin_configure_dt(&bhicfg->cs_gpios, GPIO_OUTPUT_ACTIVE); 
	LOG_DBG("reg %x data:%x",reg_addr,(uint8_t)(*reg_data));
	if(rc1 || rc2)
	{
		LOG_ERR("SPI communication error write:%d read%d",rc1,rc2);
		return BHY2_E_IO;
	}	
	return BHY2_INTF_RET_SUCCESS;
}

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	const struct device *bhidev =intf_ptr;
	const struct bhix60_cfg *bhicfg = bhidev->config;
	const struct spi_dt_spec *spidev = &bhicfg->bus;

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
	gpio_pin_configure_dt(&bhicfg->cs_gpios, GPIO_OUTPUT_INACTIVE); 
	rc = spi_write_dt(spidev, &tx);
	gpio_pin_configure_dt(&bhicfg->cs_gpios, GPIO_OUTPUT_ACTIVE);
	if(rc)
	{
		LOG_ERR("SPI communication error write:%d",rc);
		return BHY2_E_IO;
	}
    return BHY2_INTF_RET_SUCCESS;
}

void bhy2_delay_us(uint32_t us, void *private_data)
{
	(void)private_data;
	k_usleep(us);
}


#if defined(BHIX60_FIFO_INT)
int bhix60_proc_fifo(const struct device *dev);
/**
 * @brief callback of interrupt pin, if hw interrupt is enabled for FIFO processing 
 */
static void bhix60_hirq_callback(const struct device *dev,
				 struct gpio_callback *cb, uint32_t pins)
{
	struct bhix60_data *devdata =
		CONTAINER_OF(cb, struct bhix60_data, gpio_cb);
	const struct bhix60_cfg *cfg = devdata->dev->config;

	if ((pins & BIT(cfg->hirq_gpios.pin)) == 0U) {
		return;
	}

	gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_DISABLE);
#if defined(CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD)
	/*delegate actual trigger handling to kernel main thread*/
	k_work_submit(&devdata->work);
#elif defined(CONFIG_BHIX60_FIFO_INT_FIFO_THREAD)
	k_sem_give(&devdata->fifo_sem);
#endif
}

static void bhix60_handle_intr(struct bhix60_data *devdata)
{
	const struct device * dev = devdata->dev;
	const struct bhix60_cfg *cfg = dev->config;
	/*call the FIFO processor.*/
	bhix60_proc_fifo(dev);
	gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_LEVEL_ACTIVE);
}

#if defined (CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD)
/**
 * @brief callback from kernel main thread if trigger interrupt is handled 
 */
static void bhix60_work_cb(struct k_work *work)
{
	struct bhix60_data *devdata =
		CONTAINER_OF(work, struct bhix60_data, work);
	bhix60_handle_intr(devdata);
}
#endif

#if defined (CONFIG_BHIX60_FIFO_INT_FIFO_THREAD)
static void fifo_thread(struct bhix60_data *devdata)
{
	while (1) {
		k_sem_take(&devdata->fifo_sem, K_FOREVER);
		bhix60_handle_intr(devdata);
	}
}
#endif

/**
 * @brief do all initialization necessary to enable trigger interrupts,
 * in case it is activated. This function assumes int pin is previously
 * initialized.
 */
static int bhix60_init_interrupt(const struct device *dev)
{
	const struct bhix60_cfg *cfg = dev->config;
	struct bhix60_data *devdata = dev->data;
	devdata->dev = dev;
	
#if defined(CONFIG_BHIX60_FIFO_INT_FIFO_THREAD)
	k_sem_init(&devdata->fifo_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&devdata->fifo_thread, devdata->fifo_thread_stack,
		       CONFIG_BHIX60_FIFO_THREAD_STACK_SIZE,
		       (k_thread_entry_t)fifo_thread, devdata,
		       0, NULL, K_PRIO_COOP(CONFIG_BHIX60_FIFO_THREAD_PRIORITY),
		       0, K_NO_WAIT);
#elif defined(CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD)
	devdata->work.handler = bhix60_work_cb;
#endif /* CONFIG_BHIX60_FIFO_INT_GLOBAL_THREAD */

	gpio_init_callback(&devdata->gpio_cb,
						bhix60_hirq_callback,	
						BIT(cfg->hirq_gpios.pin));
	gpio_add_callback(cfg->hirq_gpios.port, 
					&devdata->gpio_cb);
	int rc = gpio_pin_interrupt_configure_dt(&cfg->hirq_gpios, GPIO_INT_LEVEL_ACTIVE);
	return rc;
}
#endif /*BHIX60_FIFO_INT*/

/**
 * @brief poll BHIx60 interrupt pin GPIO (for POLL mode)
 */
static bool inline bhix60_get_interrupt_status(const struct device *dev)
{
	const struct bhix60_cfg *bhicfg = dev->config;
	return (gpio_pin_get_dt(&bhicfg->hirq_gpios)==1);
}

static int bhix60_init_pins(const struct device *dev)
{
	int rc1=0,rc2=0,rc3=0;
	const struct bhix60_cfg *bhicfg = dev->config;
	const struct spi_dt_spec *spidev = &bhicfg->bus;
	/*remove const to allow manipulation*/
	struct spi_cs_control *cs= (struct spi_cs_control *)(spidev->config.cs);
	/*inhibit CS auto-control in order to control the CS pin directly,
	as auto-control algorithm does not appear to work with BHIx60*/
	cs->gpio.port =NULL; 
	rc1 = gpio_pin_configure_dt(&bhicfg->cs_gpios, GPIO_OUTPUT_ACTIVE);
	/* Configure hirq as a pull-down. The BHy260 operates the interrupt pin as an 
	active high, level, push-pull by default */
    rc2 = gpio_pin_configure_dt(&bhicfg->hirq_gpios,(GPIO_INPUT|GPIO_PULL_DOWN)); 
    rc3 = gpio_pin_configure_dt(&bhicfg->reset_gpios, GPIO_OUTPUT_INACTIVE); 
	return (rc1 || rc2 || rc3)?-EINVAL:0;
}

static int bhix60_channel_raw_get(const struct device *dev,
			       enum sensor_channel chan,
			       uint8_t **data,
				   uint32_t *data_size,
				   uint16_t *range,
				   struct k_mutex **slock,
				   uint64_t *time_stamp)
{
	struct bhix60_data *devdata = dev->data;
	
	int16_t sensor_id = bhix60_chan_to_sid(chan);
	if(sensor_id <0)
	{
		LOG_ERR("Invalid sensor channel %d",chan);
		return -EINVAL;
	}
	struct bhix60_data_slot *p_slot = bhix60_get_slot(devdata,sensor_id);
	if(p_slot ==NULL)
	{
		LOG_ERR("Virtual sensor %s not registered", bhix60_get_sensor_name(sensor_id));
		return -EINVAL;
	}
	*data = p_slot->data;
	*data_size = p_slot->size;
	*time_stamp = p_slot->time_stamp;
	*range = p_slot->range;
	*slock = &p_slot->slock;
	return 0;
}

int bhix60_channel_parse_get(const struct device *dev,
			    enum sensor_channel chan,
			    bhix60_parser_t parse_func,
                void *parsed_data,
				uint64_t *timestamp)
{
    uint8_t *raw_data=NULL;
    uint32_t data_size =0;
    uint16_t range;
	struct k_mutex *slock;
    int ret = bhix60_channel_raw_get(dev,chan,&raw_data,&data_size,&range,&slock,timestamp);
    if(ret ==0)
	{
		if(k_mutex_lock(slock,K_MSEC(10)) == 0)
		{ 
			parse_func(raw_data,parsed_data);
			k_mutex_unlock(slock);
		}
		else{
			ret = -EBUSY;
		}
	}
    return ret;
}



/**
 * @brief sensor value get
 * @return -ENOTSUP for unsupported channels
 */
static int bhix60_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	uint8_t *data=NULL;
	uint32_t data_size = 0;
	uint64_t time_stamp;
	uint16_t range;
	struct k_mutex *slock;
	int ret = bhix60_channel_raw_get(dev, chan,&data,&data_size,&range,&slock,&time_stamp);
	/*convert raw data to sensor_value*/
	return bhix60_data_conv(chan,data,range,val);
    if(ret ==0)
	{
		if(k_mutex_lock(slock,K_MSEC(10)) == 0)
		{ 
			ret = bhix60_data_conv(chan,data,range,val);
			k_mutex_unlock(slock);
		}
		else{
			ret = -EBUSY;
		}
	}
    return ret;
}

static int bhix60_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	(void)chan;
	return bhix60_poll_fifo(dev);
}

int bhix60_attr_set(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 const struct sensor_value *val)
{
	struct bhy2_dev *p_bhy2 = bhix60_get_bhy2_dev(dev);
	int16_t sensor_id = bhix60_chan_to_sid(chan);
	int8_t ret = BHY2_OK;
	if(sensor_id <0)
	{
		LOG_ERR("Invalid sensor channel %d",chan);
		return -EINVAL;
	}
	
	if(attr == SENSOR_ATTR_CONFIGURATION)
	{
		/*register the virtual sensor, if not already registered*/
		ret = bhix60_register_virtual_sensor(dev, chan);
		if(ret){
			return ret; /*Could not be registered: fail*/
		}
		/*val1: sample rate in Hz: val2: latency in ms*/
		float sample_rate = (float) val->val1;
		uint32_t latency = val->val2;
		ret = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, latency ,p_bhy2);
	}
	else
	{	
		/*below attributes require slot to already have been registered
		by a previous call to SENSOR_ATTR_CONFIGURATION*/
		struct bhix60_data *devdata = dev->data;
		struct bhix60_data_slot *pslot =  bhix60_get_slot(devdata,sensor_id);
		if(pslot == NULL) {
			LOG_ERR("Slot not found for sensor %s", bhix60_get_sensor_name(sensor_id));
			return -EINVAL;
		}
		if(attr == SENSOR_ATTR_FULL_SCALE)
		{
			uint16_t range = 0;

			ret = bhix60_range_conv(chan,&range,val);
			if(ret){
				return ret; /*channel does not support range setting*/
			}
			ret = bhy2_set_virt_sensor_range(sensor_id, range,p_bhy2);
			if(ret == BHY2_OK) {
				pslot->range = range;
			}
		}
		else if(attr == SENSOR_ATTR_SAMPLING_FREQUENCY)
		{
			/*change sampling freq and latency for previously registered sensors*/
			float sample_rate = (float) val->val1;
			uint32_t latency = val->val2;
			ret = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, latency ,p_bhy2);
		}
		else if(attr == SENSOR_ATTR_UPPER_THRESH)
		{
			bhix60_thresh_conv(chan,val,&pslot->trig_threshold_upper,pslot->range);
			LOG_INF("Upper Threshold set for SID:%d at FIFO val:%d for range:%d",
				sensor_id, pslot->trig_threshold_upper,pslot->range);
		}
		else if( attr == SENSOR_ATTR_LOWER_THRESH)
		{
			bhix60_thresh_conv(chan,val,&pslot->trig_threshold_lower,pslot->range);
			LOG_INF("Lower Threshold set for SID:%d at FIFO val:%d for range:%d",
				sensor_id, pslot->trig_threshold_lower,pslot->range);
		}
		else
		{
			return -ENOTSUP;
		}
	}
	if(ret != BHY2_OK)
	{
		LOG_ERR("Error setting attribute %d for %s :%s",attr,
			bhix60_get_sensor_name(sensor_id),
			bhix60_get_api_error(ret));
		return bhix60_api_to_errno(ret);
	}

	return 0;
}

int bhix60_attr_get(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 struct sensor_value *val)
{
	return -ENOTSUP;
}

int bhix60_trigger_set(const struct device *dev,
				    const struct sensor_trigger *trig,
				    sensor_trigger_handler_t handler)
{
	struct bhix60_data *devdata = dev->data;
	
	enum sensor_channel chan = trig->chan;
	int16_t sensor_id = bhix60_chan_to_sid(chan);
	if(sensor_id <0)
	{
		LOG_ERR("Invalid sensor channel %d",chan);
		return -EINVAL;
	}
	struct bhix60_data_slot *p_slot = bhix60_get_slot(devdata,sensor_id);
	if(p_slot ==NULL)
	{
		LOG_ERR("Virtual sensor %s not registered", bhix60_get_sensor_name(sensor_id));
		return -EINVAL;
	}
	if(p_slot->ntrigs < CONFIG_BHIX60_MAX_SLOT_TRIGGERS)
	{
		p_slot->trigs[p_slot->ntrigs].type = trig->type;
		p_slot->trigs[p_slot->ntrigs].chan = trig->chan;
		p_slot->thandlers[p_slot->ntrigs] = handler;
		p_slot->ntrigs ++;
	}
	else
	{
		LOG_ERR("Too many triggers for Virtual sensor %s", bhix60_get_sensor_name(sensor_id));
		return -EINVAL;
	}
	return 0;
}

void bhix60_parse_data(const struct bhy2_fifo_parse_data_info *fifo_data, void *arg);

int bhix60_register_virtual_sensor(const struct device *dev, enum sensor_channel chan)
{
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	int i;
	int16_t sensor_id = bhix60_chan_to_sid(chan);
	if(sensor_id <0)
	{
		LOG_ERR("Invalid sensor channel %d",chan);
		return -EINVAL;
	}
	/*event size includes 1 byte for sensor id. In the below line, it is removed. Note that data_size
	0 is valid, as some events have no data.*/
	uint16_t data_size = p_bhy2->event_size[sensor_id]-1;
	if(data_size < 0)
	{
		LOG_ERR("Unsupported BHY2 Virtual Sensor %d (sensor channel %d)",sensor_id,chan);
	}
	/*locate first empty slot*/
	for(i=0;i<CONFIG_BHIX60_VIRTUAL_SENSORS_USED;i++)
	{
		if(devdata->slots[i].sensor_id == sensor_id)
			return 0;/*already registered, nothing to do*/
		else if (devdata->slots[i].sensor_id == 0)
			break;		
	}
	if(i==CONFIG_BHIX60_VIRTUAL_SENSORS_USED)
	{
		LOG_ERR("Too many virtual sensors used. Increase CONFIG_BHIX60_VIRTUAL_SENSORS_USED");
		return -E2BIG;
	}
	int rslt = bhy2_register_fifo_parse_callback(sensor_id, bhix60_parse_data, (void *)dev, p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("BHIX60 error registering fifo parser for %s :%s",
			bhix60_get_sensor_name(sensor_id),
			bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
	devdata->slots[i].sensor_id = sensor_id;
	devdata->slots[i].size = data_size;
	devdata->slots[i].trig_threshold_upper = INT32_MAX;
	devdata->slots[i].trig_threshold_lower = INT32_MIN;
	devdata->slots[i].trig_threshold_fired = false;
	k_mutex_init(&devdata->slots[i].slock);
	/*allocate data buffer to store received data until fetched*/
	devdata->slots[i].data = k_calloc(1,data_size);
	if(devdata->slots[i].data == NULL)
	{
		LOG_ERR("Insufficient heap error allocating slot buffer");
		devdata->slots[i].sensor_id = 0;
		bhy2_deregister_fifo_parse_callback(sensor_id, p_bhy2);
		return -ENOMEM;
	}
	struct bhy2_virt_sensor_conf vcfg ={0};
	bhy2_get_virt_sensor_cfg(sensor_id,&vcfg,p_bhy2);
	devdata->slots[i].range = vcfg.range;
	devdata->nslots++;
	LOG_INF("Sensor ID %d registered. Current Range: %d (SI units)",sensor_id, vcfg.range);
	return 0;
}

int bhix60_config_virtual_sensor(const struct device *dev, enum sensor_channel chan, 
	float sample_rate, uint32_t report_latency_ms)
{
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	int16_t sensor_id = bhix60_chan_to_sid(chan);
	if(sensor_id <0)
	{
		LOG_ERR("Invalid sensor channel %d",chan);
		return -EINVAL;
	}
	/*Configure sensor to start receiving data in FIFO*/
	int rslt = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Error configuring virtual sensor %s :%s",
			bhix60_get_sensor_name(sensor_id),
			bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
	return 0;
}

/**
 * @brief process FIFO once
 * This is called either by the FIFO poller in poll mode (via the app fetch loop)
 * Or by the interrupt handler thread in in one of the interrupt modes
 */
static int bhix60_proc_fifo(const struct device *dev)
{
	int rslt = BHY2_OK;
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	/* Data from the FIFO is read and the relevant callbacks 
	if registered are called */
	rslt = bhy2_get_and_process_fifo(devdata->work_buffer, 
			BHIX60_WORK_BUFFER_SIZE, 
			p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Error processing FIFO:%s",bhix60_get_api_error(rslt));
		return -EIO;
	}
	return 0;
}

/**
 * @brief FIFO poller for polling the FIFO of a particular BHI60 device
 * If POLL mode is active, it is called from fetch, which should be called 
 * cyclically by the application at least as fast as the fastest sampling 
 * rate that has been set 
 */
static int bhix60_poll_fifo(const struct device *dev)
{
#ifdef CONFIG_BHIX60_FIFO_POLL
	if (bhix60_get_interrupt_status(dev))
	{
		return bhix60_proc_fifo(dev);
	}
#endif
	return 0;
}

/*TODO: hysteresis support in this macro.*/
/*slot thresholds are set to INT_MAX/MIN values in slot initialization, so below
code should work even if one or the other threshold has not been set using attr_set */
#define THRESHOLD_CROSSED(DATAVAL) (((int32_t)(DATAVAL) >= pslot->trig_threshold_upper) | \
								   ((int32_t)(DATAVAL) <= pslot->trig_threshold_lower))
static bool threshold_check(struct bhix60_data_slot *pslot)
{
	uint8_t *data = pslot->data;
	bool xthresh = false;
	switch (pslot->size) {
	case 10:	/*Quaternion: apply threshold to W wlement only*/
	    xthresh = THRESHOLD_CROSSED(BHY2_LE2S16(data + 6));
		break;
	case 6:	/*3DVectors and Euler: apply to ALL of the elements individually*/
	    xthresh = THRESHOLD_CROSSED(BHY2_LE2S16(data)) |
				  THRESHOLD_CROSSED(BHY2_LE2S16(data + 2)) |
				  THRESHOLD_CROSSED(BHY2_LE2S16(data + 4));
		break;
	case 4:
	    xthresh = THRESHOLD_CROSSED(BHY2_LE2S32(data));
		break;
	case 3:
	    xthresh = THRESHOLD_CROSSED(BHY2_LE2S24(data));
		break;
	case 1:
	    xthresh = THRESHOLD_CROSSED(data[0]);
		break;
	default: /*threshold check not supported for other data types*/
		return false;
	}
	/*compute rising edge on threshold flag*/
	bool thresh_rx = xthresh && !pslot->trig_threshold_fired;
	pslot->trig_threshold_fired = xthresh;
	return thresh_rx;
}

void bhix60_parse_data(const struct bhy2_fifo_parse_data_info *fifo_data, void *arg)
{
	const struct device *dev = arg;
	struct bhix60_data *devdata = dev->data;
	
	uint16_t sensor_id =  fifo_data->sensor_id;
	/*data_size in fifo_data is actually event size (data + 1 byte sensor id header), but the
	data pointer points to the data, excluding the header. This anomaly is corrected here.
	The below line may have to be updated if this anomaly is removed from a future version of
	the BHY2 API. Note that slot size is correctly set to data size in bhix60_register_virtual_sensor()*/
	uint16_t data_size_in = fifo_data->data_size-1;
	struct bhix60_data_slot *pslot =bhix60_get_slot(devdata,sensor_id);
	if(pslot  == NULL) /*fifo event sensor_id not registered with driver*/
		return;
	if(pslot->size != data_size_in)
	{
		LOG_ERR("Unexpected event data size %d in fifo for sensor id %d. Expected %d.",
			data_size_in,sensor_id,pslot->size);
		return;		
	}
	if(k_mutex_lock(&pslot->slock,K_MSEC(10)) == 0)
	{ 
		memcpy(pslot->data, fifo_data->data_ptr, data_size_in);
		pslot->time_stamp = *fifo_data->time_stamp;
		k_mutex_unlock(&pslot->slock);
	}
	else{
		return;
	}
	LOG_DBG("Sensor Event:%d size:%d value--",fifo_data->sensor_id,fifo_data->data_size);
	LOG_HEXDUMP_DBG(fifo_data->data_ptr, data_size_in,";");
	for(int i = 0; i<pslot->ntrigs;i++)
	{
		if(
			(pslot->trigs[i].type == SENSOR_TRIG_DATA_READY) 
							||
			(bhix60_is_event_trigger(pslot->trigs[i].type)
				&& data_size_in == 0) 
							|| 
			(pslot->trigs[i].type == SENSOR_TRIG_THRESHOLD 
				&& threshold_check(pslot))
		)
		{
			pslot->thandlers[i](dev,&pslot->trigs[i]);
		}
	}
}

/**
 * @brief Upload Firmware to BHIX60 FLASH or RAM and boot
 */
static int bhix60_upload_firmware_and_boot(const struct device *dev,uint8_t boot_stat, struct bhy2_dev *p_bhy2)
{
    uint8_t sensor_error=0;
    int8_t rslt = BHY2_OK;
	/*Upload Firmware to RAM or FLASH, if required*/
	unsigned char *fw=NULL;
	unsigned int fw_sz=0;
	/*call application-defined function to get application-specific firmware*/
	if(bhix60_get_firmware(dev, &fw, &fw_sz) || fw==NULL || fw_sz ==0)
	{
		LOG_ERR("Application-specific firmware not available");
		return -ENOTSUP;
	}

#if defined(CONFIG_BHIX60_UPLOAD_FW_TO_FLASH)

    if (boot_stat & BHY2_BST_FLASH_DETECTED)
    {
        uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
        uint32_t end_addr = start_addr + fw_sz;
        LOG_DBG("BHIX60 Flash detected. Erasing flash to upload firmware");

        rslt = bhy2_erase_flash(start_addr, end_addr, p_bhy2);
        if(rslt!= BHY2_OK)
		{
			LOG_ERR("BHIX60 Flash erase error:%s",bhix60_get_api_error(rslt));
			return bhix60_api_to_errno(rslt);
		}
    }
    else
    {
        LOG_ERR("BHIX60 Flash not detected");
		return -EIO;
    }

    LOG_DBG("Loading firmware into BHIX60 FLASH");
    rslt = bhy2_upload_firmware_to_flash(fw, fw_sz, p_bhy2);
#elif defined(CONFIG_BHIX60_UPLOAD_FW_TO_RAM)
    LOG_DBG("Loading firmware into BHIX60 RAM");
    rslt = bhy2_upload_firmware_to_ram(fw,fw_sz, p_bhy2);
#endif
	sensor_error=0;
    bhy2_get_error_value(&sensor_error, p_bhy2);
    if (sensor_error)
    {
        LOG_ERR("BHIX60 Error Register:%s",bhix60_get_sensor_error_text(sensor_error));
    }
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Firmware upload error:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
	/*Boot from RAM or FLASH*/
#if defined(CONFIG_BHIX60_UPLOAD_FW_TO_FLASH)
    LOG_DBG("Booting from FLASH");
    rslt = bhy2_boot_from_flash(p_bhy2);
#else
    LOG_DBG("Booting from RAM");
    rslt = bhy2_boot_from_ram(p_bhy2);
#endif
	sensor_error =0;
    bhy2_get_error_value(&sensor_error, p_bhy2);
    if (sensor_error)
    {
        LOG_ERR("BHIX60 Error Register:%s",bhix60_get_sensor_error_text(sensor_error));
    }
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Firmware boot error:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
	return 0;
}

/**
 * @brief Initialize BHIX60
 */

static int bhix60_init(const struct device *dev)
{
    uint8_t product_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t work_buffer[BHIX60_WORK_BUFFER_SIZE];
    uint8_t hintr_ctrl=0, hif_ctrl=0, boot_status=0;
	
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	if(bhix60_init_pins(dev))
	{
		LOG_ERR("Unable to initialize CS, HIRQ and Reset pins");
		return -EINVAL;
	}

	rslt = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, (void*)dev, p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("API Init Error:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
	bhy2_soft_reset(p_bhy2);
    bhy2_get_product_id(&product_id, p_bhy2);
    if (product_id != BHY2_PRODUCT_ID)
    {
        LOG_ERR("Aborting init: Unexpected Product ID %x. Expected %x", product_id, BHY2_PRODUCT_ID);
		return -EFAULT;
    }
#ifndef CONFIG_BHIX60_STATUS_DEBUG
	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	bhy2_set_host_interrupt_ctrl(hintr_ctrl, p_bhy2);
#endif
    bhy2_get_host_interrupt_ctrl(&hintr_ctrl, p_bhy2);
	LOG_INF("Host interrupt control code: %x",hintr_ctrl);

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Host intf ctrl error:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}

    /* Check if the sensor is ready to load firmware */
    bhy2_get_boot_status(&boot_status, p_bhy2);
	LOG_INF("Boot status (preboot):%x",boot_status);
#if defined(BHIX60_FLASH_AUTO_BOOT)
	/*Auto-Boot from flash configured*/
	rslt = bhy2_boot_from_flash(p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Error booting from flash:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}
#else
	/*Upload FW to Flash or RAM before boot configured*/
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        rslt = bhix60_upload_firmware_and_boot(dev,boot_status, p_bhy2);
		if(rslt != 0)
		{
			LOG_ERR("Error uploading and booting FW. Aborting BHIX60 Init");
			return rslt;
		}
    }
    else
    {
		LOG_ERR("Host interface not ready. Abort boot.");
        return -ETIMEDOUT;
    }
#endif
    bhy2_get_boot_status(&boot_status, p_bhy2);
	LOG_INF("Boot status (postboot):%x\n",boot_status);

	bhy2_get_kernel_version(&version, p_bhy2);
	LOG_INF("Kernel version %u\n", version);

	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, bhix60_parse_meta_event, NULL, p_bhy2);
	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, bhix60_parse_meta_event, NULL, p_bhy2);
  	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, bhix60_parse_debug_message, NULL, p_bhy2);

	rslt = bhy2_get_and_process_fifo(work_buffer, BHIX60_WORK_BUFFER_SIZE, p_bhy2);
	if(rslt!= BHY2_OK)
	{
		LOG_ERR("Error processing FIFO:%s",bhix60_get_api_error(rslt));
		return bhix60_api_to_errno(rslt);
	}

	/*update callback table*/
	bhy2_update_virtual_sensor_list(p_bhy2);
	/*store virtual sensor list in buffer in bhy2 struct*/
	bhy2_get_virt_sensor_list(p_bhy2);
	bhix60_print_sensors(p_bhy2);
	/*process FIFO once to clear any pending events, to avoid
	triggering FIFO interrupt at this stage*/
	bhix60_proc_fifo(dev);
#if defined(BHIX60_FIFO_INT)
	if(bhix60_init_interrupt(dev))
	{
		LOG_ERR("Unable to initialize FIFO interrupt handler");
		return -EINVAL;
	}
#endif	
	return 0;
}
/**
 * @brief Zephyr-compatible wrappers for miscellaneous useful BHY2 commands
 * 
 */
int bhix60_host_standby(const struct device *dev, bool status)
{
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	uint8_t hintf_ctrl = (status)?0x1F:0;
	int8_t ret = bhy2_set_host_intf_ctrl(hintf_ctrl, p_bhy2);
	return (ret == BHY2_OK)?0:-EIO;
}

int bhix60_soft_reset(const struct device *dev)
{
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	int8_t ret = bhy2_soft_reset(p_bhy2);
	return (ret == BHY2_OK)?0:-EIO;
}

int bhix60_flush_fifo(const struct device *dev)
{
	struct bhix60_data *devdata = dev->data;
	struct bhy2_dev *p_bhy2 = &devdata->bhy2;
	/*0xFE => Flush all FIFOs, discarding data*/
	int8_t ret = bhy2_flush_fifo(0xFE,p_bhy2);
	return (ret == BHY2_OK)?0:-EIO;
}


static const struct sensor_driver_api bhix60_api = {
	.sample_fetch = bhix60_sample_fetch,
	.channel_get = bhix60_channel_get,
	.attr_set = bhix60_attr_set,
	.attr_get = bhix60_attr_get,
	.trigger_set = bhix60_trigger_set,
};

#define BHIX60_INIT(index)						\
									\
static struct bhix60_data bhix60_data_##index = {0};			\
									\
const static struct bhix60_cfg bhix60_cfg_##index = {	\
	.bus = SPI_DT_SPEC_INST_GET(index,SPI_OPERATION_MODE,SPI_DELAY),     \
	.cs_gpios = SPI_CS_GPIOS_DT_SPEC_INST_GET(index), \
	.reset_gpios = GPIO_DT_SPEC_INST_GET(index, reset_gpios),\
	.hirq_gpios = GPIO_DT_SPEC_INST_GET(index, hirq_gpios),\
};			\
		\
DEVICE_DT_INST_DEFINE(index, &bhix60_init, NULL, \
		      &bhix60_data_##index,&bhix60_cfg_##index, \
			  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,\
		      &bhix60_api);

DT_INST_FOREACH_STATUS_OKAY(BHIX60_INIT)