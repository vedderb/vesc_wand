/*
	Copyright 2017 - 2021 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "deca_port.h"

#include <zephyr.h>
#include <device.h>
#include <hal/nrf_gpio.h>
#include <drivers/spi.h>

/*
 * Connections
 *
 * P0.19: DW_IRQ
 * P0.16: DW_SCK
 * P0.20: DW_MOSI
 * P0.18: DW_MISO
 * P0.17: DW_SPI_CS
 * P0.24: DW_RST
 *
 */

#define DW_PIN_IRQ		NRF_GPIO_PIN_MAP(0, 19)
#define DW_PIN_SCK		NRF_GPIO_PIN_MAP(0, 16)
#define DW_PIN_MOSI		NRF_GPIO_PIN_MAP(0, 20)
#define DW_PIN_MISO		NRF_GPIO_PIN_MAP(0, 18)
#define DW_PIN_CS		NRF_GPIO_PIN_MAP(0, 17)
#define DW_PIN_RST		NRF_GPIO_PIN_MAP(0, 24)

static const struct spi_config spi_cfg_slow = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.frequency = 2000000,
	.slave = 0,
};

static const struct spi_config spi_cfg_fast = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.frequency = 8000000,
	.slave = 0,
};

// Private variables
static volatile bool isr_enabled = true;
static const struct device *spi_dev;
static const struct device *gpio_dev;
static const struct spi_config *spi_cfg_now = &spi_cfg_slow;
static struct gpio_callback isr_cb_data;
static struct k_work isr_work;

// Private functions
static void isr_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	(void)dev;
	(void)cb;
	(void)pins;

	if (!isr_enabled) {
		return;
	}

	k_work_submit(&isr_work);
}

void handle_isr(struct k_work *item) {
	(void)item;

	while (nrf_gpio_pin_read(DW_PIN_IRQ)) {
		dwt_isr();
	}
}

void deca_port_init(void) {
	nrf_gpio_cfg_output(DW_PIN_RST);
	nrf_gpio_cfg_output(DW_PIN_CS);

	spi_dev = device_get_binding("SPI_1");

	if (spi_dev == NULL) {
		printk("Could not get %s device\n", "SPI_1");
		return;
	}

	gpio_dev = device_get_binding("GPIO_0");
	if (gpio_dev == NULL) {
		printk("Error: didn't find %s device\n", "GPIO_0");
		return;
	}

	k_work_init(&isr_work, handle_isr);

	gpio_pin_configure(gpio_dev, DW_PIN_IRQ, GPIO_INPUT);
	gpio_pin_interrupt_configure(gpio_dev, DW_PIN_IRQ, GPIO_INT_EDGE_RISING);
	gpio_init_callback(&isr_cb_data, isr_cb, BIT(DW_PIN_IRQ));
	gpio_add_callback(gpio_dev, &isr_cb_data);

	// Reset and wake up
	deca_port_reset();
	deca_port_set_spi_slow();
	nrf_gpio_pin_clear(DW_PIN_CS);
	k_msleep(600);
	nrf_gpio_pin_set(DW_PIN_CS);
}

void deca_port_reset(void) {
	nrf_gpio_cfg_output(DW_PIN_RST);
	nrf_gpio_pin_clear(DW_PIN_RST);
	k_msleep(2);
	nrf_gpio_cfg_input(DW_PIN_RST, NRF_GPIO_PIN_NOPULL);
	k_msleep(2);
}

void deca_port_go_to_sleep(void) {
	dwt_configuresleep(DWT_PRESRV_SLEEP, DWT_SLP_EN | DWT_WAKE_CS);
	dwt_entersleep();
//	nrf_gpio_cfg_default(DW_PIN_IRQ);
//	nrf_gpio_cfg_default(DW_PIN_SCK);
//	nrf_gpio_cfg_default(DW_PIN_MOSI);
//	nrf_gpio_cfg_default(DW_PIN_MISO);
//	nrf_gpio_cfg_default(DW_PIN_CS);
//	nrf_gpio_cfg_default(DW_PIN_RST);
}

void deca_port_set_spi_slow(void) {
	spi_cfg_now = &spi_cfg_slow;
}

void deca_port_set_spi_fast(void) {
	spi_cfg_now = &spi_cfg_fast;
}

int deca_port_writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
	nrf_gpio_pin_clear(DW_PIN_CS);

	const struct spi_buf header_buf = {
			.buf = (void*)headerBuffer,
			.len = headerLength
	};
	const struct spi_buf_set header_buf_set = {
			.buffers = &header_buf,
			.count = 1
	};
	spi_write(spi_dev, spi_cfg_now, &header_buf_set);

	const struct spi_buf body_buf = {
			.buf = (void*)bodyBuffer,
			.len = bodylength
	};
	const struct spi_buf_set body_buf_set = {
			.buffers = &body_buf,
			.count = 1
	};
	spi_write(spi_dev, spi_cfg_now, &body_buf_set);

	nrf_gpio_pin_set(DW_PIN_CS);

	return 0;
}

int deca_port_readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
	nrf_gpio_pin_clear(DW_PIN_CS);

	const struct spi_buf header_buf = {
			.buf = (void*)headerBuffer,
			.len = headerLength
	};
	const struct spi_buf_set header_buf_set = {
			.buffers = &header_buf,
			.count = 1
	};
	spi_write(spi_dev, spi_cfg_now, &header_buf_set);

	struct spi_buf body_buf = {
			.buf = readBuffer,
			.len = readlength
	};
	const struct spi_buf_set body_buf_set = {
			.buffers = &body_buf,
			.count = 1
	};
	spi_read(spi_dev, spi_cfg_now, &body_buf_set);

	nrf_gpio_pin_set(DW_PIN_CS);

	return 0;
}

void deca_port_sleep(unsigned int time_ms) {
	k_msleep(time_ms);
}

decaIrqStatus_t decamutexon(void) {
	decaIrqStatus_t ret = isr_enabled;
	isr_enabled = false;
	return ret;
}

void decamutexoff(decaIrqStatus_t s) {
	if (s) {
		isr_enabled = true;
	}
}
