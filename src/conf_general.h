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

#ifndef SRC_CONF_GENERAL_H_
#define SRC_CONF_GENERAL_H_

// Firmware version
#define FW_MAJOR		1
#define FW_MINOR		3

// Pin definitions
#ifdef WAND_DW

// Push buttons
#define SW1_PIN			NRF_GPIO_PIN_MAP(0, 4)
#define SW1_PIN2		NRF_GPIO_PIN_MAP(0, 4)
#define SW2_PIN			NRF_GPIO_PIN_MAP(0, 3)

// Magnetic field sensor joystick
#define PIN_ALS_SDA		NRF_GPIO_PIN_MAP(0, 8)
#define PIN_ALS_SCL		NRF_GPIO_PIN_MAP(0, 7)

// LIS2DH12 Accelerometer (in DWM1001)
#define PIN_LIS_SDA		NRF_GPIO_PIN_MAP(0, 29)
#define PIN_LIS_SCL		NRF_GPIO_PIN_MAP(0, 28)
#define LIS_I2C_ADDR	0x19

// IMU pins (wrong on rev0)
#define PIN_IMU_SCL		NRF_GPIO_PIN_MAP(0, 28)
#define PIN_IMU_SDA		NRF_GPIO_PIN_MAP(0, 29)

// Oled pins
#define OLED_PIN_RES	NRF_GPIO_PIN_MAP(0, 11)
#define OLED_PIN_CS		NRF_GPIO_PIN_MAP(0, 5)
#define OLED_PIN_D0		NRF_GPIO_PIN_MAP(0, 13)
#define OLED_PIN_D1		NRF_GPIO_PIN_MAP(0, 23)
#define OLED_PIN_DC		NRF_GPIO_PIN_MAP(0, 27)
#define OLED_PIN_BOOST	NRF_GPIO_PIN_MAP(0, 9) // REV0: This is a wire bridge

// ADC Channels
#define ADC_CH_BATT		1

#elif defined(SABRE)

// Push buttons
#define SW1_PIN			NRF_GPIO_PIN_MAP(0, 4)
#define SW1_PIN2		NRF_GPIO_PIN_MAP(0, 4)
#define SW2_PIN			NRF_GPIO_PIN_MAP(0, 5)

// Magnetic field sensor joystick
#define PIN_ALS_SDA		NRF_GPIO_PIN_MAP(0, 31)
#define PIN_ALS_SCL		NRF_GPIO_PIN_MAP(1, 9)

// IMU pins
#define PIN_IMU_SCL		NRF_GPIO_PIN_MAP(1, 0)
#define PIN_IMU_SDA		NRF_GPIO_PIN_MAP(1, 1)

// Oled pins
#define OLED_PIN_RES	NRF_GPIO_PIN_MAP(0, 27)
#define OLED_PIN_CS		NRF_GPIO_PIN_MAP(0, 8)
#define OLED_PIN_D0		NRF_GPIO_PIN_MAP(0, 25)
#define OLED_PIN_D1		NRF_GPIO_PIN_MAP(0, 26)
#define OLED_PIN_DC		NRF_GPIO_PIN_MAP(1, 12)
#define OLED_PIN_BOOST	NRF_GPIO_PIN_MAP(0, 3)

// ADC Channels
#define ADC_CH_BATT		1

#elif defined(WAND_MAG)

// Push buttons
#define SW1_PIN			NRF_GPIO_PIN_MAP(1, 1)
#define SW1_PIN2		NRF_GPIO_PIN_MAP(1, 1)
#define SW2_PIN			NRF_GPIO_PIN_MAP(0, 13)

// Magnetic field sensor joystick
#define PIN_ALS_SDA		NRF_GPIO_PIN_MAP(0, 29)
#define PIN_ALS_SCL		NRF_GPIO_PIN_MAP(1, 9)
#define PIN_ALS_EN		NRF_GPIO_PIN_MAP(0, 11)

#define PIN_IMU_SCL		NRF_GPIO_PIN_MAP(1, 2)
#define PIN_IMU_SDA		NRF_GPIO_PIN_MAP(1, 3)

// Oled pins
#define OLED_PIN_RES	NRF_GPIO_PIN_MAP(0, 27)
#define OLED_PIN_CS		NRF_GPIO_PIN_MAP(0, 8)
#define OLED_PIN_D0		NRF_GPIO_PIN_MAP(0, 25)
#define OLED_PIN_D1		NRF_GPIO_PIN_MAP(0, 26)
#define OLED_PIN_DC		NRF_GPIO_PIN_MAP(1, 12)
#define OLED_PIN_BOOST	NRF_GPIO_PIN_MAP(0, 6)

// ADC Channels
#define ADC_CH_BATT		9

#else

// Push buttons
#define SW1_PIN			NRF_GPIO_PIN_MAP(0, 19)
#define SW1_PIN2		NRF_GPIO_PIN_MAP(1, 1)
#define SW2_PIN			NRF_GPIO_PIN_MAP(0, 13)

// Potentiometer joystick
#define JS_GND_PIN		NRF_GPIO_PIN_MAP(0, 21)
#define JS_C_PIN		NRF_GPIO_PIN_MAP(0, 2)

// IMU pins
#define PIN_IMU_SCL		NRF_GPIO_PIN_MAP(1, 2)
#define PIN_IMU_SDA		NRF_GPIO_PIN_MAP(1, 3)

// Oled pins
#define OLED_PIN_RES	NRF_GPIO_PIN_MAP(0, 27)
#define OLED_PIN_CS		NRF_GPIO_PIN_MAP(0, 8)
#define OLED_PIN_D0		NRF_GPIO_PIN_MAP(0, 25)
#define OLED_PIN_D1		NRF_GPIO_PIN_MAP(0, 26)
#define OLED_PIN_DC		NRF_GPIO_PIN_MAP(1, 12)
#define OLED_PIN_BOOST	NRF_GPIO_PIN_MAP(0, 6)

// ADC Channels
#define ADC_CH_JS		1
#define ADC_CH_BATT		9 // 9 is internal VDD
#endif

// Macros
#define SW1				(!nrf_gpio_pin_read(SW1_PIN) || !nrf_gpio_pin_read(SW1_PIN2))
#define SW2				!nrf_gpio_pin_read(SW2_PIN)

#ifdef JS_GND_PIN
#define JS_OFF()		nrf_gpio_pin_set(JS_GND_PIN)
#define JS_ON()			nrf_gpio_pin_clear(JS_GND_PIN)
#else
#define JS_OFF()
#define JS_ON()
#endif

#endif /* SRC_CONF_GENERAL_H_ */
