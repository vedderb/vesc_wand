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

#define CONFIG_DEPRECATED_ZEPHYR_INT_TYPES

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <nrfx.h>

#if defined(CONFIG_SOC_NRF52840)
#include <nrf52840.h>
#else
#include <nrf52.h>
#endif

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <drivers/clock_control.h>
#include <hal/nrf_gpio.h>
#include <drivers/flash.h>
#include <fs/nvs.h>
#include <storage/flash_map.h>

#include "conf_general.h"
#include "nrf_esb.h"
#include "crc.h"
#include "utils.h"
#include "buffer.h"
#include "oled.h"
#include "fonts.h"
#include "imu.h"
#include "datatypes.h"
#include "als31300.h"
#include "lis2dh12.h"

#include "deca_port.h"
#include "deca_range.h"
#include "deca_device_api.h"
#include <drivers/pwm.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

typedef struct {
	volatile bool use_imperial_units;
} conf_data;

#define CONF_DATA_ID	1

// Private variables
static volatile vesc_values_t vesc_val = {0};
static volatile int esb_rx_cnt = 0;
static volatile int esb_no_rx_cnt = 0;
static volatile float v_batt = 0.0;
static volatile float v_js = 0.5;
static volatile float v_js_diff = 0.0;
static volatile float v_js_avg_diff = 0.0;
static volatile int v_js_fault_samples = 0;
static volatile bool v_js_fault_now = false;
static volatile int sleep_in_ms = -1;
static volatile bool is_rev = false;
static volatile int is_cc = 0;
static volatile int screen_now = 0;
static volatile bool mote_locked = false;
static volatile bool intro_done = false;

#ifdef PIN_ALS_SDA
static volatile float als_mag_xyz[3] = {0.0};
static volatile float als2_mag_xyz[3] = {0.0};
static volatile float als_throttle[2] = {0.0};
#endif

static struct nvs_fs fs;
static conf_data conf;

struct esb_info {
    struct k_work work;
    struct nrf_esb_payload rx_payload;
} esb_info_obj;

// Global variables
volatile bool going_to_sleep = false;

static void decode_packet(uint8_t *data, int len) {
	esb_rx_cnt++;
	esb_no_rx_cnt = 0;

	if (len >= 15 && data[0] == MOTE_PACKET_ALIVE) {
		int32_t index = 1;
		vesc_val.rx_cnt++;
		vesc_val.battery_level = buffer_get_float16(data, 1e3, &index);
		vesc_val.speed = buffer_get_float32(data, 1e3, &index);
		vesc_val.distance = buffer_get_float32(data, 1e3, &index);
		vesc_val.temp_fet = buffer_get_float16(data, 1e1, &index);
		vesc_val.temp_motor = buffer_get_float16(data, 1e1, &index);
		index++; // Skip sequence number

		if (len >= 28) {
			vesc_val.wh_left = buffer_get_float32(data, 1e3, &index);
			vesc_val.wh_used = buffer_get_float32(data, 1e4, &index);
			vesc_val.wh_charged = buffer_get_float32(data, 1e4, &index);
			vesc_val.curr_prop = (float)((int8_t)data[index++]) / 100.0;
		}
	}
}

static void esb_event_handler(struct nrf_esb_evt const *event) {
	switch (event->evt_id) {
	case NRF_ESB_EVENT_TX_SUCCESS:
		break;
	case NRF_ESB_EVENT_TX_FAILED:
		nrf_esb_flush_tx();
		break;
	case NRF_ESB_EVENT_RX_RECEIVED: {
		if (nrf_esb_read_rx_payload(&esb_info_obj.rx_payload) == 0) {
			k_work_submit(&esb_info_obj.work);
		}
	} break;
	}
}

static void handle_esb_rx(struct k_work *item) {
	struct esb_info *info = CONTAINER_OF(item, struct esb_info, work);

	if (info->rx_payload.length > 0) {
		uint16_t crc = (uint16_t)info->rx_payload.data[info->rx_payload.length - 2] << 8 |
				(uint16_t)info->rx_payload.data[info->rx_payload.length - 1];
		uint16_t crc_calc = crc16_2(info->rx_payload.data, info->rx_payload.length - 2);

		if (crc == crc_calc) {
			decode_packet(info->rx_payload.data, info->rx_payload.length);
		}
	}
}

int esb_init(void) {
	k_work_init(&esb_info_obj.work, handle_esb_rx);

	static struct nrf_esb_config nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
	nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.retransmit_delay = 1000;
	nrf_esb_config.retransmit_count = 1;
	nrf_esb_config.tx_mode = NRF_ESB_TXMODE_AUTO;
	nrf_esb_config.bitrate = NRF_ESB_BITRATE_1MBPS;
	nrf_esb_config.event_handler = esb_event_handler;
	nrf_esb_config.mode = NRF_ESB_MODE_PTX;
	nrf_esb_config.selective_auto_ack = false;
	nrf_esb_config.crc = NRF_ESB_CRC_8BIT;
#ifdef WAND_DW
	nrf_esb_config.tx_output_power = NRF_ESB_TX_POWER_4DBM;
#else
	nrf_esb_config.tx_output_power = NRF_ESB_TX_POWER_8DBM;
#endif

	int err;

	err = nrf_esb_init(&nrf_esb_config);
	if (err) {
		return err;
	}

	err = nrf_esb_set_address_length(3);
	if (err) {
		return err;
	}

	return 0;
}

void esb_wait_idle(void) {
	int cnt = 0;
	while(!nrf_esb_is_idle()) {
		k_msleep(1);
		cnt++;

		if (cnt >= 10) {
			break;
		}
	}
}

void set_addr_ch(uint32_t h) {
	uint8_t base_addr_0[4] = {199, 0, 0, 0};
	uint8_t addr_prefix[8] = {198, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
	uint8_t channel = 67;

	addr_prefix[0] = h & 0xFF;
	base_addr_0[0] = (h >> 8) & 0xFF;
	base_addr_0[1] = (h >> 16) & 0xFF;
	channel = (h >> 24) & 0x3F;

	if (channel <= 5) {
		channel += 5;
	}

	nrf_esb_set_rf_channel(channel);
	nrf_esb_set_base_address_0(base_addr_0);
	nrf_esb_set_base_address_1(base_addr_0);
	nrf_esb_set_prefixes(addr_prefix, 1);
}

void rfhelp_send_data_crc(char *data, int len) {
	static struct nrf_esb_payload tx_payload;

	tx_payload.pipe = 0;
	tx_payload.length = len + 2;
	tx_payload.noack = false;

	unsigned short crc = crc16_2((unsigned char*)data, len);

	memcpy(tx_payload.data, data, len);
	tx_payload.data[len] = (char)(crc >> 8);
	tx_payload.data[len + 1] = (char)(crc & 0xFF);

	int res = nrf_esb_write_payload(&tx_payload);

	if (res) {
		printk("TX failed %d\r\n", res);
	}
}

void send_pairing_info(uint32_t h) {
	uint8_t base_addr_0[4] = {0xC5, 0, 0, 0};
	uint8_t addr_prefix[8] = {0xC6, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
	uint8_t channel = 67;

	uint8_t addr0 = h & 0xFF;
	uint8_t addr1 = (h >> 8) & 0xFF;
	uint8_t addr2 = (h >> 16) & 0xFF;
	uint8_t ch = (h >> 24) & 0x3F;

	if (ch <= 5) {
		ch += 5;
	}

	nrf_esb_stop_rx();

	nrf_esb_set_rf_channel(channel);
	nrf_esb_set_base_address_0(base_addr_0);
	nrf_esb_set_base_address_1(base_addr_0);
	nrf_esb_set_prefixes(addr_prefix, 1);

	uint8_t pl[6];
	int32_t index = 0;
	pl[index++] = MOTE_PACKET_PAIRING_INFO;
	pl[index++] = addr0;
	pl[index++] = addr1;
	pl[index++] = addr2;
	pl[index++] = ch;

	rfhelp_send_data_crc((char*)pl, index);
	esb_wait_idle();

	set_addr_ch(h);
	nrf_esb_start_rx();
}

static uint32_t crc32c(uint8_t *data, uint32_t len) {
	uint32_t crc = 0xFFFFFFFF;

	for (uint32_t i = 0; i < len;i++) {
		uint32_t byte = data[i];
		crc = crc ^ byte;

		for (int j = 7;j >= 0;j--) {
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (0x82F63B78 & mask);
		}
	}

	return ~crc;
}

int clocks_start(void) {
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

	/*
	 * HFCLK is required to run ESB, as the RC clock is not accurate enough to run the radio.
	 *
	 * It is no longer possible to start HFCLK manually in zephyr. If it is started as above
	 * some driver turns it off all the time, so the code above has to be run over and over
	 * somewhere.
	 *
	 * One way to work around that is setting
	 *
	 * CONFIG_CLOCK_CONTROL_NRF_K32SRC_SYNTH=y
	 *
	 * in prj.conf, as this will keep HFCLK on to synthesize the 32 kHz clock from HFCLK.
	 */

	printk("HF clock started\r\n");
	return 0;
}

void go_to_sleep(void) {
	sleep_in_ms = 5000;

	bool was_sw1 = false;
	bool was_sw2 = false;

	while (sleep_in_ms > 0) {
		bool clicked_1 = SW1 && !was_sw1;
		bool clicked_2 = SW2 && !was_sw2;

		if (clicked_1 || clicked_2) {
			esb_no_rx_cnt = 0;
		}

		was_sw1 = SW1;
		was_sw2 = SW2;

		sleep_in_ms -= 20;
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		k_msleep(20);

		if (esb_no_rx_cnt < 5) {
			sleep_in_ms = -1;
			return;
		}
	}

	going_to_sleep = true;

	for (int i = 0;i < 10;i++) {
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		k_msleep(100);
	}

	nrf_esb_disable();
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	nvs_write(&fs, CONF_DATA_ID, &conf, sizeof(conf));

#ifdef WAND_DW
	deca_port_go_to_sleep();
#endif

	JS_OFF();
	nrf_gpio_cfg_sense_input(SW1_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(SW1_PIN2, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(SW2_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	NRF_POWER->SYSTEMOFF = 1;
}

void go_to_sleep_long_press(void) {
	sleep_in_ms = 2000;

	while (sleep_in_ms > 0) {
		sleep_in_ms -= 20;
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		k_msleep(20);

		if (!SW2) {
			sleep_in_ms = -1;
			return;
		}
	}

	going_to_sleep = true;

	for (int i = 0;i < 10;i++) {
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		k_msleep(100);
	}

	while (SW2) {
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
		k_msleep(20);
	}

	nrf_esb_disable();
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	nvs_write(&fs, CONF_DATA_ID, &conf, sizeof(conf));

#ifdef WAND_DW
	deca_port_go_to_sleep();
#endif

	JS_OFF();
	nrf_gpio_cfg_sense_input(SW1_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(SW1_PIN2, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	nrf_gpio_cfg_sense_input(SW2_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	NRF_POWER->SYSTEMOFF = 1;
}

#ifdef WAND_DW
static volatile float dw_last_dist = 0.0;
static volatile uint8_t dw_last_id;
static void dw_range_func(float dist, uint8_t id) {
	dw_last_dist = dist;
	dw_last_id = id;
}

static void dw_packet_func(uint8_t sender, uint8_t *buffer, int len) {
	uint32_t uuid = crc32c((uint8_t*)NRF_FICR->DEVICEADDR, 6);
	uint8_t addr0 = uuid & 0xFF;
	uint8_t addr1 = (uuid >> 8) & 0xFF;
	uint8_t addr2 = (uuid >> 16) & 0xFF;
	uint8_t ch = (uuid >> 24) & 0x3F;

	if (len >= 4 && sender == ch &&
			buffer[0] == addr0 &&
			buffer[1] == addr1 &&
			buffer[2] == addr2) {
		decode_packet(buffer + 3, len - 3);
	}
}
#endif

static const struct device *pwm_dev;

void main(void) {
	nrf_gpio_cfg_input(SW1_PIN, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SW1_PIN2, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SW2_PIN, NRF_GPIO_PIN_PULLUP);

#ifdef JS_GND_PIN
	nrf_gpio_cfg_output(JS_GND_PIN);
#endif

#ifdef WAND_DW
	deca_port_init();

	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
//		for (;;) {
//
//		}
	}

	deca_port_set_spi_fast();
	deca_range_configure(254);
	deca_range_set_range_func(dw_range_func);
	deca_range_set_data_func(dw_packet_func);
#endif

#ifdef SABRE
//	pwm_dev = device_get_binding("PWM_0");
//	pwm_pin_set_usec(pwm_dev, 47, 2000, 1, 0);

//	float t = 0;
//	for (;;) {
//		t += M_PI / 100;
//		if (t > 2.0 * M_PI) {
//			t -= 2.0 * M_PI;
//		}
//		float v = (sinf(t) + 1.0) / 2.0;
//		pwm_pin_set_usec(pwm_dev, 47, 2000, v * 2000, 0);
//		k_msleep(10);
//	}

#endif

	// TODO: Test performance with
	// https://devzone.nordicsemi.com/f/nordic-q-a/15093/change-clock-speed-nrf52

	// TODO: try
	// https://devzone.nordicsemi.com/f/nordic-q-a/15243/high-power-consumption-when-using-fpu

	struct flash_pages_info info;
	int rc = 0;

	fs.offset = FLASH_AREA_OFFSET(storage);;
	rc = flash_get_page_info_by_offs(device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL), fs.offset, &info);
	if (rc) {
		printk("Unable to get page info");
	}

	fs.sector_size = info.size;
	fs.sector_count = 2;

	rc = nvs_init(&fs, DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
	if (rc) {
		printk("Flash Init failed\n");
	}

	rc = nvs_read(&fs, CONF_DATA_ID, &conf, sizeof(conf));
	if (rc != sizeof(conf)) {
		conf.use_imperial_units = false;
	}

	clocks_start();

	int err = esb_init();
	if (esb_init()) {
		printk("ESB initialization failed, err %d", err);
	}

	uint32_t uuid = crc32c((uint8_t*)NRF_FICR->DEVICEADDR, 6);

	printk("UUID: %u\r\n", uuid);

	set_addr_ch(uuid);

	// Send pairing info at boot
	for (int i = 0;i < 50;i++) {
		send_pairing_info(uuid);
	}

	esb_wait_idle();
	nrf_esb_start_rx();

	// Watchdog
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | (WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = 32768 / 2; // 500 ms timeout
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk | WDT_RREN_RR1_Msk | WDT_RREN_RR2_Msk;
	NRF_WDT->TASKS_START = 1;

	bool was_sw1 = false;
	bool was_sw2 = false;
	bool was_sw_both = false;
	int click_cnt = 0;
	int not_click_cnt = 0;
	int click_cnt_prev = 0;
	bool was_bt_z = false;
	bool was_locked = false;
	float sw1_time = 0.0;
	float sw2_time = 0.0;
	bool imperial_toggled = false;

	float loop_hz = 50.0;

	// Wait for buttons to be released
	while(SW1 || SW2) {
		k_msleep(10);
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	}
	k_msleep(100);

	for(;;) {
		NRF_WDT->RR[0] = WDT_RR_RR_Reload;

		float js = v_js * 2.0 - 1.0;

		bool chdir_valid = fabsf(js) < 0.1 &&
				(vesc_val.rx_cnt == 0 || fabsf(vesc_val.speed) < 1.0) &&
				is_cc != 2;
		bool clicked_1 = !SW1 && was_sw1 && sw1_time < 0.5;
		bool clicked_2 = !SW2 && was_sw2 && sw2_time < 1.0;

		if (chdir_valid) {
			if (clicked_1) {
				click_cnt++;
			}

			if (SW1) {
				not_click_cnt = 0;
			} else {
				not_click_cnt++;
			}

			if (not_click_cnt >= 10) {
				click_cnt = 0;
			}
		} else {
			not_click_cnt = 0;
			click_cnt = 0;
		}

		if ((chdir_valid && ((click_cnt_prev == 1 && click_cnt == 0) || (sw1_time > 0.5 && sw2_time < 0.1))) ||
				(!chdir_valid && (clicked_1 || (sw1_time > 0.5 && sw2_time < 0.1)))) {
			if (is_cc == 0) {
				is_cc = 2;
			} else if (sw1_time < 0.5) {
				is_cc = 0;
			}
		}

		if (js < -0.08 || js > 0.08 || click_cnt >= 2) {
			is_cc = 0;
		}

		int bt_z = click_cnt >= 2;
		int bt_push = 0;

		if (bt_z) {
			if (bt_z != was_bt_z) {
				is_rev = !is_rev;
			}
		}

		if (clicked_1 || clicked_2) {
			esb_no_rx_cnt = 0;
		}

		if (mote_locked) {
			is_cc = 0;
			screen_now = 0;
			is_rev = false;

			if (js > 0.9 && (clicked_1 || clicked_2)) {
				mote_locked = false;
				was_locked = true;
			}
		} else {
			if (fabsf(vesc_val.speed) < 0.1 && js < -0.9 &&
					(clicked_1 || clicked_2)) {
				mote_locked = true;
			}

			if (js < 0.1 && was_locked) {
				was_locked = false;
			}

			if (!mote_locked && clicked_2) {
				screen_now++;
				if (screen_now >= 3) {
					screen_now = 0;
				}
			}
		}

		click_cnt_prev = click_cnt;
		was_sw_both = (SW1 && SW2) || (was_sw_both && (SW1 || SW2));
		was_sw1 = SW1 && !was_sw_both;
		was_sw2 = SW2 && !was_sw_both;
		was_bt_z = bt_z;

		if (SW1 || was_sw_both) {
			sw1_time += 1.0 / loop_hz;
		} else {
			sw1_time = 0.0;
		}

		if (SW2 || was_sw_both) {
			sw2_time += 1.0 / loop_hz;
		} else {
			sw2_time = 0.0;
		}

		if (sw2_time > 1.0 && sw1_time < 0.1 && !mote_locked && fabsf(vesc_val.speed) < 1.0) {
			go_to_sleep_long_press();
			was_sw1 = false;
			was_sw2 = false;
		}

		if (is_cc == 2 && (sw1_time > 0.5 && sw2_time < 0.1)) {
			js = 1.0;
		}

		if (sw1_time > 2.0 && sw2_time > 2.0) {
			if (!imperial_toggled) {
				imperial_toggled = true;
				conf.use_imperial_units = !conf.use_imperial_units;
			}
		} else {
			imperial_toggled = false;
		}

		if (mote_locked || was_locked) {
			js = 0.0;
		}

		uint8_t pl[6];
		int32_t index = 0;
		pl[index++] = MOTE_PACKET_BUTTONS;
		pl[index++] = 128; // X
		pl[index++] = (js + 1.0) * 127; // Y
		pl[index++] = (is_cc == 2) | (0 << 1) | (bt_push << 2) | (1 << 3) | (is_rev << 4);
		pl[index++] = 0;
		pl[index++] = 0;

		nrf_esb_stop_rx();
		rfhelp_send_data_crc((char*)pl, index);
		esb_wait_idle();
		nrf_esb_start_rx();

#ifdef WAND_DW
		uint8_t pl_dw[index + 3];
		uint32_t uuid = crc32c((uint8_t*)NRF_FICR->DEVICEADDR, 6);
		pl_dw[0] = uuid & 0xFF;
		pl_dw[1] = (uuid >> 8) & 0xFF;
		pl_dw[2] = (uuid >> 16) & 0xFF;
		uint8_t ch = (uuid >> 24) & 0x3F;
		memcpy(pl_dw + 3, pl, index);

		deca_range_set_address(ch);

#if 1
		deca_range_send_data(ch, pl_dw, index + 3);
#else
		static int div = 0.0;
		div++;
		if (div > 5) {
			div = 0;
			deca_range_measure(ch, 2);
		} else {
			deca_range_send_data(ch, pl_dw, index + 3);
		}
#endif
#endif

		k_msleep(1000 / loop_hz);

		esb_no_rx_cnt++;
		if (((float)esb_no_rx_cnt / loop_hz) >= 30.0) {
			go_to_sleep();
		}
	}
}

void display_thd(void) {
	oled_init();

	// Startup animation
	int phase = 0;
	int ofs_x = 400;
	float rot = 0.0;
	float scale = 1.0;
	int brightness = 15;
	int sleep_next_ms = 0;

	for (;;) {
		NRF_WDT->RR[1] = WDT_RR_RR_Reload;

		if (phase == 0) {
			ofs_x -= 10;

			if (ofs_x <= -100) {
				phase++;
			}
		} else if (phase == 1) {
			ofs_x += 4;
			scale -= 0.017;
			rot += 8.6;
			if (ofs_x >= 64) {
				sleep_next_ms = 1000;
				phase++;
				rot = 0.0;
			}
		} else if (phase == 2) {
			brightness--;
			if (brightness == -1) {
				k_msleep(200);
				break;
			}
		}

		oled_set_offset_x(0);
		oled_set_offset_y(0);
		oled_clear();

		oled_draw_image_rot_scale(
				ofs_x, 64,
				0, 128, 0, 128,
				416, 106,
				208, 53,
				208, 53,
				rot,
				scale,
				-1,
				brightness,
				image_boot);

		oled_show_buffer();

		while (sleep_next_ms > 0) {
			NRF_WDT->RR[1] = WDT_RR_RR_Reload;
			k_msleep(10);
			sleep_next_ms -= 10;
		}
	}

	intro_done = true;

	// Main and lock screens

	int lock_img_brightness = 15;
	bool lock_img_brightness_increasing = false;
	int screen_offset_x = 0;
	int screen_offset_y = 0;
	int sleep_in_ms_start_time = -1;
	bool imperial_last = conf.use_imperial_units;

	for (;;) {
		NRF_WDT->RR[1] = WDT_RR_RR_Reload;

		if (going_to_sleep) {
			oled_sleep();
			for(;;) {
				NRF_WDT->RR[1] = WDT_RR_RR_Reload;
				k_msleep(10);
			}
		}

		if (sleep_in_ms >= 0) {
			if (sleep_in_ms_start_time < 0) {
				sleep_in_ms_start_time = sleep_in_ms;
			}

			oled_set_offset_x(0);
			oled_set_offset_y(0);
			oled_clear();
			oled_printf_aa(font_aa_8x16, 8 * 4, 28, 15, "Sleep in");
			oled_printf_aa(font_aa_16x24, 8 * 4, 16 * 3, 15, "%.2f", (float)sleep_in_ms / 1000.0);
			oled_fill_rectangle(0, 80, (sleep_in_ms * 128) / sleep_in_ms_start_time, 10, 15);
			oled_show_buffer();
			k_msleep(10);
			continue;
		}

		sleep_in_ms_start_time = -1;

		static int js_fault_cnt = 0;
		if (v_js_fault_now || js_fault_cnt > 0) {
			oled_set_offset_x(0);
			oled_set_offset_y(0);
			oled_clear();

			oled_printf_aa(font_aa_8x16, 8, 10, 15, "   Joystick");
			oled_printf_aa(font_aa_8x16, 8, 26, 15, "    faulty");

			oled_printf_aa(font_aa_8x16, 4, 50, 15, "  Please read");
			oled_printf_aa(font_aa_8x16, 8, 66, 15, "  manual for  ");
			oled_printf_aa(font_aa_8x16, 8, 82, 15, "  assistance  ");

			if (v_js_fault_now) {
				js_fault_cnt = 400;
			} else if (js_fault_cnt > 0) {
				js_fault_cnt--;
			}

			oled_show_buffer();
			k_msleep(10);
			continue;
		}

		static int imperial_change_cnt = 0;
		if (imperial_last != conf.use_imperial_units) {
			imperial_last = conf.use_imperial_units;
			imperial_change_cnt = 100;
		}

		if (imperial_change_cnt > 0) {
			imperial_change_cnt--;

			oled_set_offset_x(0);
			oled_set_offset_y(0);
			oled_clear();

			oled_printf_aa(font_aa_8x16, 8, 10, 15,     "     Unit ");
			oled_printf_aa(font_aa_8x16, 8, 26, 15,     "    Change");

			oled_printf_aa(font_aa_8x16, 4, 50, 15,     "   Now using  ");
			if (conf.use_imperial_units) {
				oled_printf_aa(font_aa_8x16, 8, 66, 15, "   imperial   ");
			} else {
				oled_printf_aa(font_aa_8x16, 4, 66, 15, "    metric    ");
			}
			oled_printf_aa(font_aa_8x16, 8, 82, 15,     "    units     ");

			oled_show_buffer();
			k_msleep(10);
			continue;
		}

		oled_clear();

		float imp_fact = conf.use_imperial_units ? 0.621371192 : 1.0;

		if (screen_offset_x < 128 && screen_offset_y < 128) {
			oled_set_offset_x(0 - screen_offset_x);
			oled_set_offset_y(0 - screen_offset_y);

			oled_draw_image(0, 0, 128, 128, 15,image_bg_0);
			oled_draw_image_rot_scale(13, 76, 0, 128, 0, 128, 15, 12, 0, 0, 50, -12,
					-(vesc_val.curr_prop + 1.0) * 0.5 * 220.0, 1.0, 0.0, 15, image_dial);

			oled_draw_image(utils_map(v_js, 0.0, 1.0, 19, 102),
					90, 6, 3, 15, image_map_up);

			float speed = fabsf(vesc_val.speed) * 3.6 * imp_fact;
			if (conf.use_imperial_units) {
				oled_printf_aa(font_aa_9x13, 51, 32, 15, "mph");
			} else {
				oled_printf_aa(font_aa_9x13, 46, 32, 15, "km/h");
			}

			oled_printf_aa(font_aa_25x45, 39 + (roundf(speed) < 10.0 ? 13 : 0), 44, 15, "%.0f", speed);

			// Print DW measured range
//			float dist_cm = dw_last_dist * 100.0;
//			oled_printf_aa(font_aa_25x45, 39 + (roundf(dist_cm) < 10.0 ? 13 : 0), 44, 15, "%.0f", dist_cm);

			// Print ALS31300 data
//			oled_printf_aa(font_aa_11x21, 40, 10, 10, "%.2f", als_throttle[0]);
//			oled_printf_aa(font_aa_11x21, 40, 31, 10, "%.2f", als_throttle[1]);
//			oled_printf_aa(font_aa_11x21, 20, 48, 10, "%.0f", als_mag_xyz[0]);
//			oled_printf_aa(font_aa_11x21, 20, 69, 10, "%.0f", als_mag_xyz[1]);
//			oled_printf_aa(font_aa_11x21, 20, 90, 10, "%.0f", als_mag_xyz[2]);
//			oled_printf_aa(font_aa_11x21, 70, 48, 10, "%.0f", als2_mag_xyz[0]);
//			oled_printf_aa(font_aa_11x21, 70, 69, 10, "%.0f", als2_mag_xyz[1]);
//			oled_printf_aa(font_aa_11x21, 70, 90, 10, "%.0f", als2_mag_xyz[2]);

			// Print difference between pull-samples for joystick.
//			oled_printf_aa(font_aa_11x21, 31, 100, 15, "%.2f", v_js_avg_diff);

			// Print roll
//			oled_printf_aa(font_aa_11x21, 31, 100, 15, "%.3f", imu_get_roll());

			// Print received packet counter
//			oled_printf_aa(font_aa_11x21, 31, 100, 15, "%d", esb_rx_cnt);

			if (is_cc == 1) {
				oled_printf_aa(font_aa_11x21, 31, 100, 15, "cc");
			} else if (is_cc == 2) {
				oled_printf_aa(font_aa_11x21, 31, 100, 15, "CC");
			}

			if (is_rev) {
				oled_printf_aa(font_aa_11x21, 79, 100, 15, "R");
			}

			if (esb_rx_cnt % 2 == 0) {
				oled_fill_circle(70, 124, 4, 5);
				oled_fill_circle(58, 124, 4, 15);
			} else {
				oled_fill_circle(70, 124, 4, 15);
				oled_fill_circle(58, 124, 4, 5);
			}
		}

		if (screen_offset_x > 0 && screen_offset_x < 255 && screen_offset_y < 128) {
			oled_set_offset_x(128 - screen_offset_x);
			oled_set_offset_y(0 - screen_offset_y);

			oled_draw_image(0, 0, 128, 128, 15, image_bg_1);

			float batt_vesc = vesc_val.battery_level * 100.0;
			utils_truncate_number(&batt_vesc, 0.0, 100.0);

			float batt_mote = utils_map(v_batt, 2.2, 3.0, 0.0, 100.0);
			utils_truncate_number(&batt_mote, 0.0, 100.0);

			// Prevent flickering
			static float batt_vesc_prev = 0.0;
			if (fabsf(batt_vesc - batt_vesc_prev) > 0.5) {
				batt_vesc_prev = batt_vesc;
			}

			static float batt_mote_prev = 0.0;
			if (fabsf(batt_mote - batt_mote_prev) > 1.0) {
				batt_mote_prev = batt_mote;
			}

			oled_printf_aa(font_aa_11x21, 48, 9, 15, "%.0f ", batt_vesc_prev);
			oled_printf_aa(font_aa_10x15, 66, 100, 15, "%.0f ", batt_mote_prev);

			if (conf.use_imperial_units) {
				oled_printf_aa(font_aa_9x13, 42, 61, 15, "%.2f mi", imp_fact * vesc_val.distance / 1000.0);
			} else {
				if (vesc_val.distance < 1000.0) {
					oled_printf_aa(font_aa_9x13, 42, 61, 15, "%.1f m", vesc_val.distance);
				} else {
					oled_printf_aa(font_aa_9x13, 42, 61, 15, "%.2f km", vesc_val.distance / 1000.0);
				}
			}

			float wh_km = (vesc_val.wh_used - fabsf(vesc_val.wh_charged)) /
					(vesc_val.distance / 1000.0);
			float dist_left = vesc_val.wh_left / wh_km;
			if (dist_left < 0.0) {
				dist_left = 0.0;
			}

			if (conf.use_imperial_units) {
				oled_printf_aa(font_aa_11x21, 42, 36, 15, "%.0f mi", dist_left * imp_fact);
				oled_printf_aa(font_aa_9x13, 42, 78, 15, "%.0f wh/mi", wh_km / imp_fact);
			} else {
				oled_printf_aa(font_aa_11x21, 42, 36, 15, "%.0f km", dist_left);
				oled_printf_aa(font_aa_9x13, 42, 78, 15, "%.0f wh/km", wh_km);
			}
		}

		if (screen_offset_x > 128 && screen_offset_y < 128) {
			oled_set_offset_x(256 - screen_offset_x);
			oled_set_offset_y(0 - screen_offset_y);

			oled_draw_image(0, 0, 128, 128, 15, image_bg_2);
			oled_printf_aa(font_aa_11x21, 52, 29, 15, "%.0f ", vesc_val.temp_fet);

			float temp_motor = vesc_val.temp_motor;
			utils_truncate_number(&temp_motor, -99, 499);

			oled_printf_aa(font_aa_11x21, 42, 76, 15, "%.0f ", temp_motor);

			static int version_cnt = 0;
			version_cnt++;
			if (version_cnt > 100) {
				version_cnt = 0;
			}

			if (version_cnt > 50) {
				oled_fill_rectangle(0, 107, 120, 40, 0);
				oled_printf_aa(font_aa_8x16, 8, 110, 15,     "    v %d.%d     ", FW_MAJOR, FW_MINOR);
			}
		}

		if (screen_offset_y > 0) {
			oled_set_offset_x(128 - screen_offset_x);
			oled_set_offset_y(128 - screen_offset_y);

			oled_printf_aa(font_aa_8x16, 8, 10, 15, "   Throttle");
			oled_printf_aa(font_aa_8x16, 8, 26, 15, "    Locked");

			oled_printf_aa(font_aa_8x16, 4, 50, 15, " Full throttle");
			oled_printf_aa(font_aa_8x16, 4, 66, 15, "plus any button");
			oled_printf_aa(font_aa_8x16, 4, 82, 15, "   to unlock   ");

			int br = lock_img_brightness;
			if (br < 0) {
				br = 0;
			}
			oled_draw_image(36, 109, 65, 16, br / 2, image_vesc_65_16);
			oled_show_buffer();

			lock_img_brightness += lock_img_brightness_increasing ? 1 : -1;
			if (lock_img_brightness > 31) {
				lock_img_brightness = 31;
				lock_img_brightness_increasing = !lock_img_brightness_increasing;
			}
			if (lock_img_brightness < -10) {
				lock_img_brightness = -10;
				lock_img_brightness_increasing = !lock_img_brightness_increasing;
			}
		}

		if (mote_locked) {
			if (screen_offset_y != 128 || screen_offset_x != 128) {
				utils_step_towards_int(&screen_offset_y, 128, 15);
				utils_step_towards_int(&screen_offset_x, 128, 15);
			}
		} else {
			if (screen_offset_y != 0) {
				utils_step_towards_int(&screen_offset_y, 0, 15);
			}

			if (screen_now == 0) {
				utils_step_towards_int(&screen_offset_x, 0, screen_offset_y > 0 ? 15 : 25);
			} else if (screen_now == 1) {
				utils_step_towards_int(&screen_offset_x, 128, 15);
			} else if (screen_now == 2) {
				utils_step_towards_int(&screen_offset_x, 256, 15);
			}
		}

		oled_show_buffer();
		k_msleep(10);
	}
}

void adc_sample_thd(void) {
	const struct device *adc_dev = device_get_binding("ADC_0");
	struct adc_channel_cfg channel_cfg;

	channel_cfg.channel_id = 0;
	channel_cfg.gain = ADC_GAIN_1_6;
	channel_cfg.reference = ADC_REF_INTERNAL;
	channel_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
	channel_cfg.input_positive = ADC_CH_BATT;
	channel_cfg.differential = 0;
	channel_cfg.input_negative = 0;
	adc_channel_setup(adc_dev, &channel_cfg);

#ifdef ADC_CH_JS
	channel_cfg.channel_id = 1;
	channel_cfg.gain = ADC_GAIN_1_4;
	channel_cfg.reference = ADC_REF_VDD_1_4;
	channel_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
	channel_cfg.input_positive = ADC_CH_JS;
	channel_cfg.differential = 0;
	channel_cfg.input_negative = 0;
	adc_channel_setup(adc_dev, &channel_cfg);
#endif

#ifdef PIN_ALS_SDA
#ifdef PIN_ALS_EN
	nrf_gpio_cfg_output(PIN_ALS_EN);
	nrf_gpio_pin_set(PIN_ALS_EN);
#endif

	i2c_bb_state i2c;
	i2c.sda_pin = PIN_ALS_SDA;
	i2c.scl_pin = PIN_ALS_SCL;
	i2c_bb_init(&i2c);

	als31300_sleep(&i2c, 96, 0);
	als31300_sleep(&i2c, 108, 0);
#endif

	// Put accelerometer in sleep mode to save power
#ifdef PIN_LIS_SDA
	i2c_bb_state i2c_imu;
	i2c_imu.sda_pin = PIN_LIS_SDA;
	i2c_imu.scl_pin = PIN_LIS_SCL;
	i2c_bb_init(&i2c_imu);
	lis2dh12_sleep(&i2c_imu, LIS_I2C_ADDR);
	nrf_gpio_cfg_default(PIN_LIS_SDA);
	nrf_gpio_cfg_default(PIN_LIS_SCL);
#endif

	s16_t sample;
	struct adc_sequence seq;

	seq.options = NULL;
	seq.resolution = 12;
	seq.oversampling = 3;
	seq.buffer = &sample;
	seq.buffer_size = sizeof(sample);
	seq.channels = 1 << 0;

	for (;;) {
		NRF_WDT->RR[2] = WDT_RR_RR_Reload;

		if (going_to_sleep) {
#ifdef PIN_ALS_SDA
			als31300_sleep(&i2c, 96, 1);
			als31300_sleep(&i2c, 108, 1);

			nrf_gpio_cfg_default(PIN_ALS_SDA);
			nrf_gpio_cfg_default(PIN_ALS_SCL);

#ifdef PIN_ALS_EN
			nrf_gpio_pin_clear(PIN_ALS_EN);
#endif

#endif

			for(;;) {
				NRF_WDT->RR[2] = WDT_RR_RR_Reload;
				k_msleep(10);
			}
		}

#ifdef PIN_ALS_SDA
		{
			bool ok1 = als31300_read_mag_xyz(&i2c, 96, (float*)als_mag_xyz);
			bool ok2 = als31300_read_mag_xyz(&i2c, 108, (float*)als2_mag_xyz);

			if (ok1) {
				static float mag_max = 0.0;
				static float mag_min = 70.0;
				float mag = fabsf(als_mag_xyz[1]);

				/*
				 * The throttle will have the maximum magnitude in the neutral position. Capture that
				 * magnitude up to 5 seconds after boot.
				 */
				if (k_uptime_get() < 5000) {
					if (mag > mag_max) {
						mag_max = mag;
					}
				}

				// 5 % deadband
				mag *= 1.05;

				als_throttle[0] = utils_map(mag, mag_max, mag_min, 0.0, 1.0);
				utils_truncate_number((float*)&(als_throttle[0]), 0.0, 1.0);
				als_throttle[0] *= -UTILS_SIGN(als_mag_xyz[2]);
			}

			if (ok2) {
				static float mag_max2 = 0.0;
				static float mag_min2 = 70.0;
				float mag2 = fabsf(als2_mag_xyz[1]);

				if (k_uptime_get() < 5000) {
					if (mag2 > mag_max2) {
						mag_max2 = mag2;
					}
				}

				// 5 % deadband
				mag2 *= 1.05;

				als_throttle[1] = utils_map(mag2, mag_max2, mag_min2, 0.0, 1.0);
				utils_truncate_number((float*)&(als_throttle[1]), 0.0, 1.0);
				als_throttle[1] *= -UTILS_SIGN(als2_mag_xyz[2]);
			}

#ifdef WAND_MAG
			als_throttle[0] *= -1.0;
			als_throttle[1] *= -1.0;
#endif

			if (ok1) {
				v_js = (als_throttle[0] + 1.0) / 2.0;
				v_js_fault_samples = 0;
			} else if (ok2) {
				v_js = (als_throttle[1] + 1.0) / 2.0;
				v_js_fault_samples = 0;
			} else {
				v_js_fault_samples++;

				// Restore JS to neutral after more than one second
				// of faulty samples.
				if (v_js_fault_samples > 200) {
					v_js = 0.5;
					v_js_fault_now = true;
				}
			}
		}
#endif

#ifdef ADC_CH_JS
		JS_ON();
		k_msleep(1);

		seq.channels = 1 << 1;

		/*
		 * Read one nominal sample, one with pull-up and one with
		 * pull-down. If the pull samples differ too much it means
		 * that the joystick pin has high impedance and most likely
		 * there is a fault. In the case of a fault use the old
		 * value, and if the fault persists for too long report it
		 * to the user.
		 */
		int res = adc_read(adc_dev, &seq);
		nrf_gpio_cfg_input(JS_C_PIN, NRF_GPIO_PIN_PULLUP);
//		k_msleep(1);
		float v_nom = (float)sample / 4095.0;
		res += adc_read(adc_dev, &seq);
		nrf_gpio_cfg_input(JS_C_PIN, NRF_GPIO_PIN_PULLDOWN);
//		k_msleep(1);
		float v_up = (float)sample / 4095.0;
		res += adc_read(adc_dev, &seq);
		float v_down = (float)sample / 4095.0;
		nrf_gpio_cfg_default(JS_C_PIN);

		if (res == 0) {
			v_js_diff = fabsf(v_up - v_down);
			float v_avg = (v_up + v_down) / 2.0;
			v_js_avg_diff = fabsf(v_nom - v_avg);

			if (v_js_diff < 0.4 && v_js_avg_diff < 0.15) {
				v_js_fault_samples = 0;
//				v_js = v_nom;
				UTILS_LP_FAST(v_js, v_nom, 0.5);
				v_js_fault_now = false;
			} else {
				v_js_fault_samples++;

				// Restore JS to neutral after more than one second
				// of faulty samples.
				if (v_js_fault_samples > 200) {
					v_js = 0.5;
					v_js_fault_now = true;
				}
			}
		}
		JS_OFF();
#endif

		seq.channels = 1 << 0;
		if (adc_read(adc_dev, &seq) == 0) {
			UTILS_LP_FAST(v_batt, ((float)sample / 4095.0) * (0.6 * 6.0), 0.02);
		}

		k_msleep(4);
	}
}

K_THREAD_DEFINE(display_id, 2048, display_thd, NULL, NULL, NULL, 7, 0, 10);
K_THREAD_DEFINE(adc_id, 4096, adc_sample_thd, NULL, NULL, NULL, 6, 0, 10);
