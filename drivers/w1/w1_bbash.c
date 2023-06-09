/*
 * Copyright (c) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT w1_bbash

/**
 * @brief 1-Wire Bus Master driver using Bit Bashing of a GPIO pin.
 *
 * This driver implements the 1-Wire interface using bit bashing of a GPIO 
 * pin. This is an option where the Zephyr_wi_serial driver using UART
 * cannot be used, e.g. because a UART module is not available, or because
 * it has to be implemented on an existing board which is pre-wired for
 * bit-bashing
 * NOTE: This driver does not seem to be working with the Zephyr w1 sample
 * apppplication "search". In general, with the slave search feature.
 * It has only been tested with the SKIP ROM command, which assumes there 
 * is only a single slave on the bus 
 */

#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>
#include <hal/nrf_gpio.h>
#include <nrfx_timer.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

LOG_MODULE_REGISTER(w1_bbash, CONFIG_W1_LOG_LEVEL);

#define TICK_PER_US 					UINT8_C(16)

struct w1_bbash_config {
	int nrf_timer_num;
    nrfx_timer_t nrf_timer_inst;
    void (*nrf_timer_hdlr)(const void *) ;
    unsigned int nrf_timer_irq;
	struct gpio_dt_spec w1_gpios;
    int nrf_pin_num;
	/** w1 master cfg, common to all drivers */
	struct w1_master_config master_config;
};

struct w1_bbash_data {
	/*interrupt routine flags*/
	bool running;
	int sample;
    int nrf_pin_num;
	/** w1 master data, common to all drivers */
	struct w1_master_data master_data;
};

static void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
	if(p_context == NULL)
	{
		return;
	}
	struct w1_bbash_data *data = p_context;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
			nrf_gpio_pin_clear(data->nrf_pin_num);
			break;
		case NRF_TIMER_EVENT_COMPARE1:
			nrf_gpio_pin_set(data->nrf_pin_num);
			break;
		case NRF_TIMER_EVENT_COMPARE2:
			data->sample = nrf_gpio_pin_read(data->nrf_pin_num);
			break;
		case NRF_TIMER_EVENT_COMPARE3:
			data->running = false;
			break;
        default:
            /*Do nothing. */
            break;
    }
}

static int w1_bbash_reset_bus(const struct device *dev)
{
	const struct w1_bbash_config *cfg = dev->config;
	struct w1_bbash_data *data = dev->data;
	nrfx_timer_t timer_instance = cfg->nrf_timer_inst;
	data->sample = 1;
	data->running = true;
	nrfx_timer_disable(&timer_instance);
	nrfx_timer_clear(&timer_instance);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL0,1,true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL1,(60*TICK_PER_US),true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL2,((60+9)*TICK_PER_US),true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL3,((60+60)*TICK_PER_US),true);
	nrfx_timer_enable(&timer_instance);
	while(data->running);

	if (data->sample == 0)
		return 1;

	return 0;
}


/**
 * Send 1 bit of communication to the 1-Wire Net and return the
 * result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
 * least significant bit is used and the least significant bit
 * of the result is the return bit.
 *
 * Returns: 0:   0 bit read from sendbit
 * 			1:   1 bit read from sendbit
 */
static bool w1_bbash_touch_bit(const struct device *dev, uint8_t sendbit)
{
	const struct w1_bbash_config *cfg = dev->config;
	struct w1_bbash_data *data = dev->data;
	nrfx_timer_t timer_instance = cfg->nrf_timer_inst;
	data->running = true;
	uint32_t cc1_val= (sendbit&0x01)?(1*TICK_PER_US)+1:10*TICK_PER_US;
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL0,1,true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL1,cc1_val,true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL2,((2+1)*TICK_PER_US),true);
	nrfx_timer_compare(&timer_instance,NRF_TIMER_CC_CHANNEL3,((10+10)*TICK_PER_US),true);
	nrfx_timer_enable(&timer_instance);	

	while(data->running);

	return data->sample;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
 * least significant 8 bits are used and the least significant 8 bits
 * of the result is the return byte.
 *
 * 'sendbyte'   - 8 bits to send (least significant byte)
 *
 * Return:  	- 8 bytes read from sendbyte
 */
static uint8_t w1_bbash_touchbyte(const struct device *dev, uint8_t sendbyte)
{
	uint8_t i;
	uint8_t receivedbyte=0;

	for (i=0; i<8; i++)
	{
		receivedbyte >>= 1;
		receivedbyte |= w1_bbash_touch_bit(dev, sendbyte)?0x80:0;
		sendbyte >>= 1;
	}

	return receivedbyte;
}

static int w1_bbash_read_bit(const struct device *dev)
{
	return w1_bbash_touch_bit(dev, 0xFF);
}

static int w1_bbash_write_bit(const struct device *dev, const bool sendbit)
{
	bool sentbit =	w1_bbash_touch_bit(dev, ((sendbit)?0xFF:0x00));
	return (sentbit == sendbit)?0:-EIO;
}

/*
 *  Send 8 bits of read communication to the 1-Wire Net and and return the
 *  result 8 bits read from the 1-Wire Net.
 *
 *  Returns:  8 bytes read from 1-Wire Net
 */
static int w1_bbash_read_byte(const struct device *dev)
{
	return w1_bbash_touchbyte(dev, 0xFF);
}

/*
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte'   - 8 bits to send (least significant byte)
 *
 * Returns:	0: bytes written and echo was the same
 *			-EIO: echo was not the same
 */
static int w1_bbash_write_byte(const struct device *dev, const uint8_t sendbyte)
{
	return (w1_bbash_touchbyte(dev, sendbyte) == sendbyte) ? 0 : -EIO;
}

static int w1_bbash_configure(const struct device *dev,
			       enum w1_settings_type type, uint32_t value)
{
	//const struct w1_bbash_config *cfg = dev->config;
	//struct w1_bbash_data *data = dev->data;
	int ret = 0;

	switch (type) {
	case W1_SETTING_SPEED:
		//TODO. Support this in future?
		ret = -ENOTSUP;
		break;
	default:
		ret = -ENOTSUP;
	}
	return ret;
}

static int w1_bbash_init(const struct device *dev)
{
	const struct w1_bbash_config *cfg = dev->config;
	struct w1_bbash_data *data = dev->data;
	nrfx_timer_t timer_instance = cfg->nrf_timer_inst;

	nrf_gpio_cfg(cfg->nrf_pin_num,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0D1,
		NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_pin_set(cfg->nrf_pin_num);
	data->running = false;
	data->sample=0;
	data->nrf_pin_num = cfg->nrf_pin_num;
	/**
	 * CC0 sets OWPIN to 0
	 * CC1 sets OWPIN to 1
	 * CC2 Sampling of the GPIO
	 * CC3 stops the timer and clear a flag (interrupt routine defined in owlnk.c)
	 */
    nrfx_timer_config_t timer_config = {
        .frequency = NRF_TIMER_FREQ_16MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 6, .p_context = data, 
    };
	
	/* Configure hardware timer for capturing timestamp */
    if (NRFX_SUCCESS == nrfx_timer_init(&timer_instance, &timer_config, (nrfx_timer_event_handler_t)timer_handler))
    {
		/* Compare 3 shorts to stop */
        nrfx_timer_extended_compare(&timer_instance,
                                    NRF_TIMER_CC_CHANNEL3,
                                    0,
                                    NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK|NRF_TIMER_SHORT_COMPARE3_STOP_MASK,
                                    (bool)false);/*interrupt enabled at time of timer enable*/
        irq_connect_dynamic(cfg->nrf_timer_irq, 1, cfg->nrf_timer_hdlr,NULL,0);
        irq_enable(cfg->nrf_timer_irq);
        nrfx_timer_disable(&timer_instance);
        nrfx_timer_clear(&timer_instance);
    }


	LOG_DBG("w1-bbash initialized, with %d slave devices",
		cfg->master_config.slave_count);
	return 0;
}

static const struct w1_driver_api w1_bbash_driver_api = {
	.reset_bus = w1_bbash_reset_bus,
	.read_bit = w1_bbash_read_bit,
	.write_bit = w1_bbash_write_bit,
	.read_byte = w1_bbash_read_byte,
	.write_byte = w1_bbash_write_byte,
	.configure = w1_bbash_configure,
};


#define GPIO_PORT(inst)    DT_PROP_BY_PHANDLE_IDX(DT_DRV_INST(inst), w1_gpios, 0, port)
#define GPIO_PIN(inst)     DT_GPIO_PIN_BY_IDX(DT_DRV_INST(inst), w1_gpios, 0)
#define GPIO_ABS_PIN(inst) NRF_GPIO_PIN_MAP(GPIO_PORT(inst), GPIO_PIN(inst))
#define TIMER_IRQ(t_num)      NRFX_CONCAT_3(TIMER,t_num,_IRQn)
#define TIMER_IRQ_HDLR(t_num) (void (*)(const void *))NRFX_CONCAT_3(nrfx_timer_, t_num, _irq_handler )

#define W1_BBASH_INIT(inst)							\
	static const struct w1_bbash_config w1_bbash_config_##inst = { \
		.nrf_timer_inst = NRFX_TIMER_INSTANCE(DT_INST_PROP(inst, nrf_timer_num)), \
		.nrf_timer_hdlr = TIMER_IRQ_HDLR(DT_INST_PROP(inst, nrf_timer_num)), \
		.nrf_timer_irq = TIMER_IRQ(DT_INST_PROP(inst, nrf_timer_num)), \
		.w1_gpios =	GPIO_DT_SPEC_INST_GET(inst, w1_gpios),	\
		.nrf_pin_num = GPIO_ABS_PIN(inst), \
		.master_config.slave_count = W1_INST_SLAVE_COUNT(inst)		   \
	};									\
	static struct w1_bbash_data w1_bbash_data_##inst = {0};		\
	DEVICE_DT_INST_DEFINE(inst, &w1_bbash_init, NULL, \
				  &w1_bbash_data_##inst,		\
			      &w1_bbash_config_##inst,			\
			      POST_KERNEL, CONFIG_W1_INIT_PRIORITY,		\
			      &w1_bbash_driver_api);

DT_INST_FOREACH_STATUS_OKAY(W1_BBASH_INIT)

  
