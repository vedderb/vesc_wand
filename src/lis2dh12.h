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

#ifndef SRC_LIS2DH12_H_
#define SRC_LIS2DH12_H_

#include <stdint.h>
#include <stdbool.h>

#include "i2c_bb.h"

/**
 * Address in the DMW1001 is 0x19
 */

bool lis2dh12_init(i2c_bb_state *i2c, uint16_t addr);
bool lis2dh12_read_acc_xyz(i2c_bb_state *i2c, uint16_t addr, float *xyz);
bool lis2dh12_sleep(i2c_bb_state *i2c, uint16_t addr);


#endif /* SRC_LIS2DH12_H_ */
