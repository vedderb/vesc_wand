/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;
} ATTITUDE_INFO;

typedef enum {
	PACKET_PRINT = 0,
	PACKET_RB_STATUS,
	PACKET_IMU_SAMPLES,
	PACKET_TIME_ADJUST,
	PACKET_TIME_NOW,
	PACKET_BATTERY
} PACKET_COMMAND;

typedef struct {
	uint32_t time_ms_today;

	int16_t accX;
	int16_t accY;
	int16_t accZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
} imu_samples;

// Datatypes
typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE,
	MOTE_PACKET_FILL_RX_BUFFER,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
	MOTE_PACKET_PAIRING_INFO
} MOTE_PACKET;

typedef struct {
	int rx_cnt;
	float battery_level;
	float speed;
	float distance;
	float temp_fet;
	float temp_motor;
	float wh_left;
	float wh_used;
	float wh_charged;
	float curr_prop;
} vesc_values_t;

#endif /* DATATYPES_H_ */
