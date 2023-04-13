/*
 * Copyright (c) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB CDC ACM interface functions for COINES
 *
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "common_services.h"
#include <zephyr/sys/ring_buffer.h>
LOG_MODULE_REGISTER(usb_cdc_bst, LOG_LEVEL_INF);
#define RING_BUF_SIZE 1024
uint8_t ring_buffer_rx[RING_BUF_SIZE];
uint8_t ring_buffer_tx[RING_BUF_SIZE];

struct ring_buf ringbufrx;
struct ring_buf ringbuftx;
bool connected =false;

/*For boards that support UDC CDC e.g. Application Board 3*/
#if defined(CONFIG_USB_CDC_ACM)

#include <zephyr/usb/usb_device.h>

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbufrx),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbufrx, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbufrx %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuftx, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuftx -> tty fifo %d bytes", send_len);
		}
	}
}
int usb_cdc_init(void)
{
	const struct device *dev;
	uint32_t baudrate;
	int ret;

	dev = DEVICE_DT_GET(DT_NODELABEL(uart_dev));
	if (!device_is_ready(dev)) {
		LOG_ERR("UART device not ready");
		return -EIO;
	}
#ifdef CONFIG_USB_CDC_ACM
	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return -EIO;
	}
#endif
	ring_buf_init(&ringbufrx, sizeof(ring_buffer_rx), ring_buffer_rx);
	ring_buf_init(&ringbuftx, sizeof(ring_buffer_tx), ring_buffer_tx);
	/*DO NOT wait for DTR as per new COINES concept*/
	connected=true;
	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);
	return 0;
}

uint16_t usb_cdc_bytes_available(void)
{
	return ring_buf_size_get(&ringbufrx);
}

uint16_t usb_cdc_read(void *buffer, uint16_t len)
{
	return ring_buf_get(&ringbufrx, buffer, len);
}
int usb_cdc_write(uint8_t *data,int len)
{
	return ring_buf_put(&ringbuftx, data, len);
}

bool usb_cdc_connected()
{
	return connected;
}

#else /*CONFIG_USB_CDC_ACM not defined*/

/*
For boards that do not need USB CDC e.g. NICLA
where USB console is via a gateway USB chip that connects to CPU UART
pins. Async UART is not enabled in this case, to avoid complicating
debug message logging which also uses the same mechanism. In case of
AB3, Segger is used for logging debug messages, so it is not an issue 
To support the coines_intf_available API, a simple lookahead buffer
is implemented using polling 
NOTE: Perhaps this approach can be used for AB3 also, to avoid 
complications with async method 
*/

#include <unistd.h>
#define UART_LOOKAHEAD_BUF_SIZE 100

int usb_cdc_write(uint8_t *data,int len)
{
	return write(1,data,len);
}

int usb_cdc_init(void)
{
	const struct device *dev;
	ring_buf_init(&ringbufrx, sizeof(ring_buffer_rx), ring_buffer_rx);
	dev = DEVICE_DT_GET(DT_NODELABEL(uart_dev));
	if (!device_is_ready(dev)) {
		LOG_ERR("UART Device not found/not ready");
		return -EIO;
	}
	connected=true;
	return 0;
}

uint16_t usb_cdc_bytes_available(void)
{
	const struct device *dev;
	unsigned char p_char;
	dev = DEVICE_DT_GET(DT_NODELABEL(uart_dev));
	if (!device_is_ready(dev)) {
		return 0;
	}
	while(ring_buf_space_get(&ringbufrx) >0)
	{
		int n =	uart_poll_in(dev, &p_char);
		if(n ==-1)
		{
			//retry once more after small pause
			//as there is an occasional delay
			k_usleep(100); 
			n =	uart_poll_in(dev, &p_char);
		}
		if(n==0)
			ring_buf_put(&ringbufrx,&p_char,1);
		else
			break;
	}
	return ring_buf_size_get(&ringbufrx);
}

uint16_t usb_cdc_read(void *buffer, uint16_t len)
{	
	const struct device *dev;
	unsigned char p_char;
	dev = DEVICE_DT_GET(DT_NODELABEL(uart_dev));
	if (!device_is_ready(dev)) {
		return 0;
	}
	//get data buffered in a preceding usb_cdc_bytes_available() call
	int i = ring_buf_get(&ringbufrx, buffer, len);
	//direct read, in case usb_cdc_read() is called 
	//without a preceeding call to usb_cdc_bytes_available()
	while (i < len)
	{
		int n =	uart_poll_in(dev, &p_char);
		if(n ==-1)
		{
			//retry once more after small pause
			//as there is an occasional delay
			k_usleep(100); 
			n =	uart_poll_in(dev, &p_char);
		}
		if(n==0)
		 	((unsigned char*)buffer)[i++] = p_char;
		else
			break;
	}
	return i;
}

bool usb_cdc_connected()
{
	return connected;
} 
#endif