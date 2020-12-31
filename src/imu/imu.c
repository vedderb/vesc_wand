/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "imu.h"
#include "bmi160.h"
#include "ahrs.h"

#include <math.h>
#include <zephyr.h>
#include <device.h>
#include <hal/nrf_gpio.h>
#include <sys/time_units.h>

#include "i2c_bb.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define IMU_FLIP

extern volatile bool going_to_sleep;

// Private variables
static ATTITUDE_INFO m_att;
static float m_accel[3], m_gyro[3];
static struct bmi160_dev sensor;
static i2c_bb_state s_i2c;

// Function pointers
static void(*imu_samples_ready)(imu_samples s) = 0;

// Private functions
static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
static void user_delay_ms(uint32_t ms);
static void imu_sleep_sense(void);
static void imu_sleep(void);

void imu_init(int scl_pin, int sda_pin) {
	s_i2c.scl_pin = scl_pin;
	s_i2c.sda_pin = sda_pin;

	i2c_bb_init(&s_i2c);

	ahrs_init_attitude_info(&m_att);

	sensor.id = BMI160_I2C_ADDR;
	sensor.interface = BMI160_I2C_INTF;
	sensor.read = user_i2c_read;
	sensor.write = user_i2c_write;
	sensor.delay_ms = user_delay_ms;

	bmi160_init(&sensor);

	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	bmi160_set_sens_conf(&sensor);
}

float imu_get_roll(void) {
	return ahrs_get_roll(&m_att);
}

float imu_get_pitch(void) {
	return ahrs_get_pitch(&m_att);
}

float imu_get_yaw(void) {
	return ahrs_get_yaw(&m_att);
}

void imu_get_rpy(float *rpy) {
	ahrs_get_roll_pitch_yaw(rpy, &m_att);
}

void imu_get_accel(float *accel) {
	memcpy(accel, m_accel, sizeof(m_accel));
}

void imu_get_gyro(float *gyro) {
	memcpy(gyro, m_gyro, sizeof(m_gyro));
}

void imu_get_accel_derotated(float *accel) {
	float rpy[3];
	imu_get_rpy(rpy);

	const float ax = m_accel[0];
	const float ay = m_accel[1];
	const float az = m_accel[2];

	const float sr = sinf(rpy[0]);
	const float cr = -cosf(rpy[0]);
	const float sp = sinf(rpy[1]);
	const float cp = -cosf(rpy[1]);
	const float sy = sinf(rpy[2]);
	const float cy = cosf(rpy[2]);

	float c_ax = ax * cp + ay * sp * sr + az * sp * cr;
	float c_ay = ay * cr - az * sr;
	float c_az = -ax * sp + ay * cp * sr + az * cp * cr;
	float c_ax2 = cy * c_ax + sy * c_ay;
	float c_ay2 = sy * c_ax - cy * c_ay;

	accel[0] = c_ax2;
	accel[1] = c_ay2;
	accel[2] = c_az;
}

void imu_get_quaternions(float *q) {
	q[0] = m_att.q0;
	q[1] = m_att.q1;
	q[2] = m_att.q2;
	q[3] = m_att.q3;
}

void imu_set_raw_data_callback(void(*cb)(imu_samples s)) {
	imu_samples_ready = cb;
}

static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t txbuf[1];
	txbuf[0] = reg_addr;
	return i2c_bb_tx_rx(&s_i2c, dev_addr, txbuf, 1, data, len) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	uint8_t txbuf[len + 1];
	txbuf[0] = reg_addr;
	memcpy(txbuf + 1, data, len);
	return i2c_bb_tx_rx(&s_i2c, dev_addr, txbuf, len + 1, 0, 0) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static void user_delay_ms(uint32_t ms) {
	k_msleep(ms);
}

static void imu_sleep_sense(void) {
	sensor.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
	bmi160_set_power_mode(&sensor);

	// Activate interrupt on INT2 on any motion. This is used to wake up the CPU on
	// movement of the device.

	struct bmi160_int_settg int_config;

	int_config.int_channel = BMI160_INT_CHANNEL_2;
	int_config.int_type = BMI160_ACC_ANY_MOTION_INT;
	int_config.int_pin_settg.output_en = BMI160_ENABLE;
	int_config.int_pin_settg.output_mode = BMI160_DISABLE;
	int_config.int_pin_settg.output_type = BMI160_DISABLE;
	int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
	int_config.int_pin_settg.input_en = BMI160_DISABLE;
	int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_10_MILLI_SEC;

	int_config.int_type_cfg.acc_any_motion_int.anymotion_en = BMI160_ENABLE;
	int_config.int_type_cfg.acc_any_motion_int.anymotion_x = BMI160_ENABLE;
	int_config.int_type_cfg.acc_any_motion_int.anymotion_y = BMI160_ENABLE;
	int_config.int_type_cfg.acc_any_motion_int.anymotion_z = BMI160_ENABLE;
	int_config.int_type_cfg.acc_any_motion_int.anymotion_dur = 0;
	// (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg,
	// (8-g range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg
	int_config.int_type_cfg.acc_any_motion_int.anymotion_thr = 10;

	bmi160_set_int_config(&int_config, &sensor);
}

static void imu_sleep(void) {
	sensor.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
	bmi160_set_power_mode(&sensor);
}

static void imu_sample_thd(void) {
	imu_init(NRF_GPIO_PIN_MAP(1, 2), NRF_GPIO_PIN_MAP(1, 3));

	uint32_t start = k_cycle_get_32();

	for (;;) {
		if (going_to_sleep) {
			imu_sleep();
			(void)imu_sleep_sense;

			nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(1, 2));
			nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(1, 3));

			for(;;) {
				k_msleep(100);
			}
		}

		float dt = ((float)k_cyc_to_ns_floor64(k_cycle_get_32() - start) / 1e9);
		start = k_cycle_get_32();

		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;

		int8_t res = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);

		if (res != BMI160_OK) {
			k_msleep(10);
			continue;
		}

		float tmp_accel[3], tmp_gyro[3];

		tmp_accel[0] = (float)accel.x * 16.0 / 32768.0;
		tmp_accel[1] = (float)accel.y * 16.0 / 32768.0;
		tmp_accel[2] = (float)accel.z * 16.0 / 32768.0;

		tmp_gyro[0] = (float)gyro.x * 2000.0 / 32768.0;
		tmp_gyro[1] = (float)gyro.y * 2000.0 / 32768.0;
		tmp_gyro[2] = (float)gyro.z * 2000.0 / 32768.0;

		if (imu_samples_ready) {
			imu_samples s;
			s.time_ms_today = k_uptime_get_32();
			s.accX = accel.x;
			s.accY = accel.y;
			s.accZ = accel.z;

			s.gyroX = gyro.x;
			s.gyroY = gyro.y;
			s.gyroZ = gyro.z;

			imu_samples_ready(s);
		}

#ifdef IMU_FLIP
		m_accel[0] = -tmp_accel[0];
		m_accel[1] = tmp_accel[1];
		m_accel[2] = -tmp_accel[2];

		m_gyro[0] = -tmp_gyro[0];
		m_gyro[1] = tmp_gyro[1];
		m_gyro[2] = -tmp_gyro[2];
#else
		m_accel[0] = tmp_accel[0];
		m_accel[1] = tmp_accel[1];
		m_accel[2] = tmp_accel[2];

		m_gyro[0] = tmp_gyro[0];
		m_gyro[1] = tmp_gyro[1];
		m_gyro[2] = tmp_gyro[2];
#endif

		float gyro_rad[3];
		gyro_rad[0] = m_gyro[0] * M_PI / 180.0;
		gyro_rad[1] = m_gyro[1] * M_PI / 180.0;
		gyro_rad[2] = m_gyro[2] * M_PI / 180.0;

		ahrs_update_madgwick_imu(gyro_rad, m_accel, dt, (ATTITUDE_INFO*)&m_att);

		k_msleep(10);
	}
}

K_THREAD_DEFINE(imu_id, 2048, imu_sample_thd, NULL, NULL, NULL, 7, 0, 10);
