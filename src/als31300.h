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

#ifndef SRC_MAG_THROTTLE_ALS31300_H_
#define SRC_MAG_THROTTLE_ALS31300_H_

#include <stdint.h>
#include <stdbool.h>

#include "i2c_bb.h"

/*
 * ADR0 is GND on the boards and ADR1 is 0 or 1. This gives
 * the addresses 96 and 108 (decimal)
 */

bool als31300_read_mag_xyz(i2c_bb_state *i2c, uint16_t addr, float *xyz);
bool als31300_sleep(i2c_bb_state *i2c, uint16_t addr, int sleep);

#endif /* SRC_MAG_THROTTLE_ALS31300_H_ */
