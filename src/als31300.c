/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "als31300.h"
#include <string.h>

// Private functions
static uint8_t crc8(const uint8_t *data, uint8_t len);
static void init_reg(i2c_bb_state *i2c, uint16_t addr);

bool als31300_read_mag_xyz(i2c_bb_state *i2c, uint16_t addr, float *xyz) {
	uint8_t txbuf[2] = {0x28, 0x29};
	uint8_t r28[5], r29[5];

	i2c_bb_tx_rx(i2c, addr, txbuf, 1, r28, 5);
	i2c_bb_tx_rx(i2c, addr, txbuf + 1, 1, r29, 5);

	uint8_t crc_data[6] = {0x28, (addr << 1) + 1, 0, 0, 0, 0};
	memcpy(crc_data + 2, r28, 4);
	uint8_t crc1 = crc8(crc_data, 6);
	crc_data[0] = 0x29;
	memcpy(crc_data + 2, r29, 4);
	uint8_t crc2 = crc8(crc_data, 6);

	if (crc1 != r28[4] || crc2 != r29[4]) {
		init_reg(i2c, addr);
		return false;
	}

	int16_t x = (int16_t)((uint16_t)r28[0] << 8 | (((uint16_t)r29[1] << 4) & 0xF0)) / 16;
	int16_t y = (int16_t)((uint16_t)r28[1] << 8 | (((uint16_t)r29[2] << 0) & 0xF0)) / 16;
	int16_t z = (int16_t)((uint16_t)r28[2] << 8 | (((uint16_t)r29[2] << 4) & 0xF0)) / 16;

	/*
	 * 500G-version: 4
	 * 1000G version: 2
	 * 2000G-version: 1
	 */
	float scale = 1.0;

	xyz[0] = (float)x / scale;
	xyz[1] = (float)y / scale;
	xyz[2] = (float)z / scale;

	return true;
}

/**
 * Enter sleep mode.
 *
 * Sleep:
 * 0: Active mode
 * 1: Sleep mode
 * 2: Low-power duty-cycle mode
 */
bool als31300_sleep(i2c_bb_state *i2c, uint16_t addr, int sleep) {
	uint8_t txbuf[5] = {0x27, 0, 0, 0, sleep & 0x03};
	return i2c_bb_tx_rx(i2c, addr, txbuf, 5, 0, 0);
}

static void init_reg(i2c_bb_state *i2c, uint16_t addr) {
	uint8_t txbuf[1] = {0x02};
	uint8_t rxbuf[4];

	i2c_bb_tx_rx(i2c, addr, txbuf, 1, rxbuf, 4);

	// Send access code
	uint8_t txbuf2[5] = {0x35, 0x2c, 0x41, 0x35, 0x34};
	i2c_bb_tx_rx(i2c, addr, txbuf2, 5, 0, 0);

	txbuf2[0] = 0x02;

	rxbuf[1] |= (1 << 2); // Enable CRC

	rxbuf[1] &= ~(1 << 3); // Single ended hall mode
	rxbuf[1] &= ~(1 << 4); // Single ended hall mode

	rxbuf[1] &= ~(1 << 5); // Lowest bandwidth
	rxbuf[1] &= ~(1 << 6); // Lowest bandwidth
	rxbuf[1] &= ~(1 << 7); // Lowest bandwidth

	rxbuf[2] |= (1 << 0); // Enable CH_Z
	rxbuf[2] |= (1 << 1); // 1.8V I2C Mode
	rxbuf[3] |= (1 << 6); // Enable CH_X
	rxbuf[3] |= (1 << 7); // Enable CH_Y

	memcpy(txbuf2 + 1, rxbuf, 4);
	i2c_bb_tx_rx(i2c, addr, txbuf2, 5, 0, 0);
}

static uint8_t crc8(const uint8_t *data, uint8_t len)  {
	const uint8_t poly = 0x07;
	uint8_t crc = 0x00;

	for (uint8_t j = 0;j < len;j++) {
		crc ^= *data++;

		for (uint8_t i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
		}
	}

	return crc;
}
