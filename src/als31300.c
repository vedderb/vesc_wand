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

bool als31300_read_mag_xyz(i2c_bb_state *i2c, uint16_t addr, float *xyz) {
	uint8_t txbuf[1] = {0x28};
	uint8_t rxbuf[8];

	bool ok = i2c_bb_tx_rx(i2c, addr, txbuf, 1, rxbuf, 8);

	int16_t x = (int16_t)((uint16_t)rxbuf[0] << 8 | (((uint16_t)rxbuf[5] << 4) & 0xF0)) / 16;
	int16_t y = (int16_t)((uint16_t)rxbuf[1] << 8 | (((uint16_t)rxbuf[6] << 0) & 0xF0)) / 16;
	int16_t z = (int16_t)((uint16_t)rxbuf[2] << 8 | (((uint16_t)rxbuf[6] << 4) & 0xF0)) / 16;

	/*
	 * 500G-version: 4
	 * 1000G version: 2
	 * 2000G-version: 1
	 */
	float scale = 1.0;

	xyz[0] = (float)x / scale;
	xyz[1] = (float)y / scale;
	xyz[2] = (float)z / scale;

	return ok;
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

