/*
	Copyright 2021 Benjamin Vedder	benjamin@vedder.se

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

#include "lis2dh12.h"

bool lis2dh12_init(i2c_bb_state *i2c, uint16_t addr) {
	// Power on
	uint8_t txbuf0[2] = {0x20, 0b01110111};
	i2c_bb_tx_rx(i2c, addr, txbuf0, 2, 0, 0);

	// HR Mode and +-16G
	uint8_t txbuf1[2] = {0x23, 0b00111000};
	i2c_bb_tx_rx(i2c, addr, txbuf1, 2, 0, 0);

	// Read reference to reset filters
	uint8_t txbuf[1] = {0x26};
	uint8_t rxbuf[1];
	return i2c_bb_tx_rx(i2c, addr, txbuf, 1, rxbuf, 1);
}

bool lis2dh12_read_acc_xyz(i2c_bb_state *i2c, uint16_t addr, float *xyz) {
	uint8_t txbuf[1] = {0x28 | 0x80};
	uint8_t rxbuf[6];
	bool okacc = i2c_bb_tx_rx(i2c, addr, txbuf, 1, rxbuf, 6);

	if (okacc) {
		xyz[0] = ((float)((int16_t)((uint16_t)rxbuf[0] | (uint16_t)rxbuf[1] << 8) >> 4)) * 12.0 / 1000.0;
		xyz[1] = ((float)((int16_t)((uint16_t)rxbuf[2] | (uint16_t)rxbuf[3] << 8) >> 4)) * 12.0 / 1000.0;
		xyz[2] = ((float)((int16_t)((uint16_t)rxbuf[4] | (uint16_t)rxbuf[5] << 8) >> 4)) * 12.0 / 1000.0;
	}

	return okacc;
}

bool lis2dh12_sleep(i2c_bb_state *i2c, uint16_t addr) {
	uint8_t txbuf0[2] = {0x20, 0b00000000};
	return i2c_bb_tx_rx(i2c, addr, txbuf0, 2, 0, 0);
}


