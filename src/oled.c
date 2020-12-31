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

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <math.h>

#include <zephyr.h>
#include <device.h>
#include <hal/nrf_gpio.h>

#include "oled.h"
#include "fonts.h"
#include "utils.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// Pins
#define PIN_RES		27
#define PIN_CS		8
#define PIN_D0		25
#define PIN_D1		26
#define PIN_DC		NRF_GPIO_PIN_MAP(1, 12)
#define PIN_BOOST	6

#define RES_HI()	nrf_gpio_pin_set(PIN_RES)
#define RES_LO()	nrf_gpio_pin_clear(PIN_RES)
#define CS_HI()		nrf_gpio_pin_set(PIN_CS)
#define CS_LO()		nrf_gpio_pin_clear(PIN_CS)
#define D0_HI()		nrf_gpio_pin_set(PIN_D0)
#define D0_LO()		nrf_gpio_pin_clear(PIN_D0)
#define D1_HI()		nrf_gpio_pin_set(PIN_D1)
#define D1_LO()		nrf_gpio_pin_clear(PIN_D1)
#define DC_HI()		nrf_gpio_pin_set(PIN_DC)
#define DC_LO()		nrf_gpio_pin_clear(PIN_DC)

#define BOOST_ON()	nrf_gpio_pin_set(PIN_BOOST)
#define BOOST_OFF()	nrf_gpio_pin_clear(PIN_BOOST)

#define DELAY()		__NOP()

// Private functions
static void write_cmd(uint8_t cmd);

// Private variables
static uint8_t m_display_buffer[128 * 128];
static int m_display_offset_x = 0;
static int m_display_offset_y = 0;

void oled_init(void) {
	nrf_gpio_cfg_output(PIN_RES);
	nrf_gpio_cfg_output(PIN_CS);
	nrf_gpio_cfg_output(PIN_D0);
	nrf_gpio_cfg_output(PIN_D1);
	nrf_gpio_cfg_output(PIN_DC);
	nrf_gpio_cfg_output(PIN_BOOST);

	DC_LO();
	BOOST_ON();

	CS_HI();
	D0_LO();
	D1_LO();

	RES_LO();
	k_msleep(1);
	RES_HI();
	k_msleep(1);

	write_cmd(SSD1327_SET_DISPLAY_OFF);

	// Regulator on
	write_cmd(SSD1327_FUNCTION_SELECTION_A);
	write_cmd(0x01);

	// Frequency and clock divider. Affects power consumption and flicker.
	write_cmd(SSD1327_SET_FRONT_CLOCK_DIVIDER);
	write_cmd(0x00);

	// Enable COM split odd even
	// Column remap and COM remap
	write_cmd(SSD1327_SET_REMAP);
	write_cmd(0b01010001);

	write_cmd(SSD1327_SET_DISPLAY_ON);
	write_cmd(SSD1327_SET_DISPLAY_MODE_NORMAL);

	oled_set_contrast(255);

	oled_clear();

	write_cmd(SSD1327_SET_COLUMN_ADDRESS);
	write_cmd(0);
	write_cmd(63);
	write_cmd(SSD1327_SET_ROW_ADDRESS);
	write_cmd(0);
	write_cmd(127);
}

void oled_clear(void) {
	memset(m_display_buffer, 0, sizeof(m_display_buffer));
}

void oled_sleep(void) {
	write_cmd(SSD1327_SET_DISPLAY_OFF);
	BOOST_OFF();
	write_cmd(SSD1327_FUNCTION_SELECTION_A);
	write_cmd(0x00);
}

void oled_wake_up(void) {
	BOOST_ON();
	write_cmd(SSD1327_SET_DISPLAY_ON);
}

void oled_set_offset_x(int offset) {
	m_display_offset_x = offset;
}

void oled_set_offset_y(int offset) {
	m_display_offset_y = offset;
}

void oled_put_char(const uint8_t *font, int x, int y, char c, int brightness, int bg) {
	if (c == '\0') {
		return;
	}

	c -= ' ';

	int w = font[0];
	int h = font[1];
	int bytes_per_char = (w * h) / 8;

	font += 2;

	if ((w * h) % 8 != 0) {
		bytes_per_char++;
	}

	for (int px = 0;px < (w * h);px++) {
		unsigned int cu = c;
		cu *= bytes_per_char * 8;
		cu += px;

		int set = ((1 << (cu % 8)) & (font[cu / 8]));

		if (set || bg >= 0) {
			oled_set_pixel(x + px % w, y + px / w, set ? brightness : bg);
		}
	}
}

void oled_printf(const uint8_t *font, int x, int y, int brightness, int bg, const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	char print_buffer[32];

	len = vsnprintf(print_buffer, 32, format, arg);
	va_end (arg);

	for (int i = 0;i < len;i++) {
		oled_put_char(font, x + (int)font[0] * i, y, print_buffer[i], brightness, bg);
	}
}

void oled_put_char_aa(const uint8_t *font, int x, int y, int brightness, char c) {
	if (c == '\0') {
		return;
	}

	c -= ' ';
	x += m_display_offset_x;
	y += m_display_offset_y;

	int w = font[0];
	int h = font[1];
	int bytes_per_char = (w * h) / 2;

	font += 2;

	if ((w * h) % 2 != 0) {
		bytes_per_char++;
	}

	unsigned int cu = c;
	cu *= bytes_per_char;

	for (int px = 0;px < (w * h);px++) {
		int xc = x + px % w;
		int yc = y + px / w;

		if (xc >= 0 && yc >= 0 && xc < 128 && yc < 128) {
			uint8_t ch = font[cu + px / 2];

			if (px % 2 == 0) {
				ch &= 0x0F;
			} else {
				ch >>= 4;
			}

			uint8_t *c = &m_display_buffer[yc * 128 + xc];			
			*c = (ch * brightness + (15 - ch) * (*c & 0x0F)) / 15;
		}
	}
}

void oled_printf_aa(const uint8_t *font, int x, int y, int brightness, const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	char print_buffer[32];

	len = vsnprintf(print_buffer, 32, format, arg);
	va_end (arg);

	for (int i = 0;i < len;i++) {
		oled_put_char_aa(font, x + (int)font[0] * i, y, brightness, print_buffer[i]);
	}
}

void oled_set_contrast(uint8_t value) {
	write_cmd(SSD1327_SET_CONTRAST_CURRENT);
	write_cmd(value);
}

void oled_set_pixel(int x, int y, int brightness) {
	x += m_display_offset_x;
	y += m_display_offset_y;

	if (x < 0 || y < 0 || x >= 128 || y >= 128) {
		return;
	}

	m_display_buffer[y * 128 + x] = brightness;
}

int8_t oled_get_pixel(int x, int y) {
	if (x < 0 || y < 0 || x >= 128 || y >= 128) {
		return -1;
	}

	return m_display_buffer[y * 128 + x];
}

void oled_draw_hline(int x, int y, int len, int brightness) {
	for (int i = 0;i < len;i++) {
		oled_set_pixel(x + i, y, brightness);
	}
}

void oled_draw_vline(int x, int y, int len, int brightness) {
	for (int i = 0;i < len;i++) {
		oled_set_pixel(x, y + i, brightness);
	}
}

void oled_draw_line(int x1, int y1, int x2, int y2, int brightness) {
	if (y1==y2) {
		oled_draw_hline(x1, y1, x2 - x1, brightness);
	} else if (x1==x2) {
		oled_draw_vline(x1, y1, y2 - y1, brightness);
	} else {
		unsigned int dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		short xstep =  x2 > x1 ? 1 : -1;
		unsigned int dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		short ystep =  y2 > y1 ? 1 : -1;
		int col = x1, row = y1;

		if (dx < dy) {
			int t = - (dy >> 1);
			while (true) {
				oled_set_pixel(col, row, brightness);

				if (row == y2) {
					return;
				}

				row += ystep;
				t += dx;

				if (t >= 0) {
					col += xstep;
					t   -= dy;
				}
			}
		} else {
			int t = - (dx >> 1);
			while (true) {
				oled_set_pixel(col, row, brightness);

				if (col == x2) {
					return;
				}

				col += xstep;
				t += dy;

				if (t >= 0) {
					row += ystep;
					t   -= dx;
				}
			}
		}
	}
}

void oled_fill_circle(int x, int y, int radius, int brightness) {
	for(int y1 = -radius;y1 <= 0;y1++) {
		for(int x1 =- radius;x1 <= 0;x1++) {
			if(x1 * x1 + y1 * y1 <= radius * radius) {
				oled_draw_hline(x + x1, y + y1, 2 * (-x1), brightness);
				oled_draw_hline(x + x1, y - y1, 2 * (-x1), brightness);
				break;
			}
		}
	}
}

void oled_fill_rectangle(int x, int y, int width, int height, int brightness) {
	for (int i = y;i < (y + height);i++) {
		oled_draw_hline(x, i, width, brightness);
	}
}

void oled_draw_image(int x, int y, int width, int height, int brightness, const uint8_t *image) {
	brightness++;
	if (brightness > 16) {
		brightness = 16;
	}

	for (int j = 0;j < height;j++) {
		for (int i = 0;i < ((width + 1) / 2);i++) {
			uint8_t p = image[j * ((width + 1) / 2) + i];
			uint8_t p0 = p >> 4;
			p &= 0x0F;

			p *= brightness;
			p0 *= brightness;
			p >>= 4;
			p0 >>= 4;

			oled_set_pixel(i * 2 + x, j + y, p0);

			if ((i * 2 + 1) < width) {
				oled_set_pixel(i * 2 + 1 + x, j + y, p);
			}
		}
	}
}

void oled_draw_image_rot_scale(
		int x, int y, // Where on display
		int x_start, int x_width, int y_start, int y_width, // Window to draw in (crop)
		int width, int height, // Image width and height
		int cx, int cy, // Center pixel of image
		float xr, float yr, // Pixel to rotate around
		float rot, // Rotation angle in degrees
		float scale, // Scale factor
		int transparent_color, // Color to be drawn as transparent. -1 = disabled
		int brightness, // Brightness, range 0 - 15
		const uint8_t *image) {

	x += m_display_offset_x;
	y += m_display_offset_y;

	float sr = sinf(-rot * M_PI / 180.0);
	float cr = cosf(-rot * M_PI / 180.0);

	brightness++;
	if (brightness > 16) {
		brightness = 16;
	}

	x -= cx * scale;
	y -= cy * scale;

	xr *= scale;
	yr *= scale;

	int x_end = (x_start + x_width);
	int y_end = (y_start + y_width);

	if (x_start < 0) {
		x_start = 0;
	}

	if (x_end > 128) {
		x_end = 128;
	}

	if (y_start < 0) {
		y_start = 0;
	}

	if (y_end > 128) {
		y_end = 128;
	}

	for (int j = y_start;j < y_end;j++) {
		for (int i = x_start;i < x_end;i++) {
			int px = (float)(i - x - xr) * cr + (float)(j - y - yr) * sr;
			int py = -(float)(i - x - xr) * sr + (float)(j - y - yr) * cr;

			px += xr;
			py += yr;

			px /= scale;
			py /= scale;

			if (px >= 0 && px < width && py >= 0 && py < height) {
				uint8_t p = image[py * ((width + 1) / 2) + px / 2];
				p >>= (px % 2 == 0) * 4;
				p &= 0xF;

				if (p != transparent_color) {
					p *= brightness;
					p >>= 4;
					m_display_buffer[j * 128 + i] = p;
				}
			}
		}
	}
}

void oled_show_buffer(void) {
	DC_HI();
	D0_LO();
	CS_LO();

	for(int ind = 0;ind < (128 * 64);ind++) {
		uint8_t b = m_display_buffer[2 * ind] << 4 | (m_display_buffer[2 * ind + 1] & 0xF);

//		m_display_buffer[2 * ind] = 0;
//		m_display_buffer[2 * ind + 1] = 0;

		for (int i = 0;i < 8;i++) {
			D0_LO();

			if (b & 0b10000000) {
				D1_HI();
			} else {
				D1_LO();
			}

			b <<= 1;

			D0_HI();
			DELAY();
		}
	}

	CS_HI();
}

void oled_draw_buffer_to_image(uint8_t *image) {
	for(int ind = 0;ind < (128 * 64);ind++) {
		uint8_t b = m_display_buffer[2 * ind] << 4 | (m_display_buffer[2 * ind + 1] & 0xF);
		image[ind] = b;
	}
}

static void write_cmd(uint8_t cmd) {
	DC_LO();
	D0_LO();
	DELAY();
	CS_LO();

	for (int i = 7;i >= 0;i--) {
		D0_LO();
		if ((cmd >> i) & 1) {
			D1_HI();
		} else {
			D1_LO();
		}
		D0_HI();
		DELAY();
	}

	CS_HI();
}
