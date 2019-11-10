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

#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>
#include <stdbool.h>

// Functions
void oled_init(void);
void oled_clear(void);
void oled_sleep(void);
void oled_wake_up(void);
void oled_set_offset_x(int offset);
void oled_set_offset_y(int offset);
void oled_put_char(const uint8_t *font, int x, int y, char c, int brightness, int bg);
void oled_printf(const uint8_t *font, int x, int y, int brightness, int bg, const char* format, ...);
void oled_put_char_aa(const uint8_t *font, int x, int y, int brightness, char c);
void oled_printf_aa(const uint8_t *font, int x, int y, int brightness, const char* format, ...);
void oled_set_contrast(uint8_t value);
void oled_set_pixel(int x, int y, int brightness);
int8_t oled_get_pixel(int x, int y);
void oled_draw_hline(int x, int y, int len, int brightness);
void oled_draw_vline(int x, int y, int len, int brightness);
void oled_draw_line(int x1, int y1, int x2, int y2, int brightness);
void oled_fill_circle(int x, int y, int radius, int brightness);
void oled_fill_rectangle(int x, int y, int width, int height, int brightness);
void oled_draw_image(int x, int y, int width, int height, int brightness, const uint8_t *image);
void oled_draw_image_rot_scale(
		int x, int y, // Where on display
		int x_start, int x_width, int y_start, int y_width, // Window to draw in (crop)
		int width, int height, // Image width and height
		int cx, int cy, // Center pixel of image
		float xr, float yr, // Pixel to rotate around
		float rot, // Rotation angle in degrees
		float scale, // Scale factor
		int transparent_color, // Color to be drawn as transparane. -1 = disabled
		int brightness, // Brightness, range 0 - 15
		const uint8_t *image);
void oled_show_buffer(void);
void oled_draw_buffer_to_image(uint8_t *image);

#define SSD1327_SET_COLUMN_ADDRESS						0x15
#define SSD1327_SET_ROW_ADDRESS							0x75
#define SSD1327_SET_CONTRAST_CURRENT					0x81
#define SSD1327_NOP1									0x84
#define SSD1327_NOP2									0x85
#define SSD1327_NOP3									0x86
#define SSD1327_SET_REMAP								0xA0
#define SSD1327_SET_DISPLAY_START_LINE					0xA1
#define SSD1327_SET_DISPLAY_OFFSET						0xA2
#define SSD1327_SET_DISPLAY_MODE_NORMAL					0xA4
#define SSD1327_SET_DISPLAY_MODE_ALL_ON					0xA5
#define SSD1327_SET_DISPLAY_MODE_ALL_OFF				0xA6
#define SSD1327_SET_DISPLAY_MODE_INVERT					0xA7
#define SSD1327_SET_MULTIPLEX_RATIO						0xA8
#define SSD1327_FUNCTION_SELECTION_A					0xAB
#define SSD1327_SET_DISPLAY_ON							0xAF
#define SSD1327_SET_DISPLAY_OFF							0xAE
#define SSD1327_SET_PHASE_LENGTH						0xB1
#define SSD1327_NOP4									0xB2
#define SSD1327_SET_FRONT_CLOCK_DIVIDER					0xB3
#define SSD1327_SET_GPIO								0xB5
#define SSD1327_SET_SECOND_PRECHARGE_PERIOD				0xB6
#define SSD1327_SET_GRAY_SCALE_TABLE					0xB8
#define SSD1327_SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE	0xB9
#define SSD1327_NOP5									0xBB
#define SSD1327_SET_PRECHARGE_VOLTAGE					0xBC
#define SSD1327_SET_VCOMH_VOLTAGE						0xBE
#define SSD1327_FUNCTION_SELECTION_B					0xD5
#define SSD1327_SET_COMMAND_LOCK						0xFD

#endif /* OLED_H_ */
