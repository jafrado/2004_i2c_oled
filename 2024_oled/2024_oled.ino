/**
 * @file 2024_oled.ino
 * Arduino sketch driving the Wide.HK 20x4 SSD1311 character-mode OLED.
 *
 * Target: Arduino Uno R4 Minima (Renesas RA4M1).
 *   Wire on the dedicated SDA/SCL header pins (also A4/A5).
 *   Module slave address: 0x3c (set in ssd13xx_io_arduino.cpp).
 *
 * The common display driver lives in ssd13xx_20x4_oled.c and is
 * shared verbatim with the Linux and NuttX builds; this sketch
 * only supplies setup()/loop() and reuses the lcd_* API. The
 * Arduino-specific I2C transport and ms_delay() are in
 * ssd13xx_io_arduino.cpp.
 *
 * The demo cycles through:
 *   - banner
 *   - compass-arrow walk at home position
 *   - dancing vertical bars (lower half)
 *   - nav-symbol charset cycle
 *   - full-screen dancing vertical bars
 *   - cylon scan across all four rows
 *   - full 0..255 character sweep
 *
 * @copyright
 * Copyright (C) 2024 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>

extern "C" {
#include "ssd13xx_20x4_oled.h"
}

/* CGROM A built-in compass arrows used for the home-position walk */
static const unsigned char arrows[] = {
	222, 23, 223, 25, 224, 24, 225, 22
};

void setup(void)
{
	Wire.begin();
	/* Default 100 kHz matches the Linux/NuttX configuration. The
	 * SSD1311 supports up to 400 kHz Fast Mode if you want to
	 * uncomment the line below.
	 */
	/* Wire.setClock(400000); */

	lcd_init();
	lcd_startup_banner();
}

void loop(void)
{
	int i;
	int j;
	int k;
	int c;
	unsigned char buf[32];
	unsigned char ubuf[32];

	lcd_cls();

	strcpy((char*)ubuf, "      Wide.HK       ");
	lcd_write(ubuf, 20);

	strcpy((char*)ubuf, "  20x4 OLED Display ");
	lcd_write(ubuf, 20);

	/* Compass arrow walk at home position */
	for (k = 0; k < 3; k++) {
		for (j = 0; j < 8; j++) {
			ubuf[0] = arrows[j];
			lcd_goto(0, 0);
			lcd_write(ubuf, 1);
			delay(50);
		}
	}

	/* Dancing vertical bars (lower half of the display) */
	lcd_define_vbar_symbol();

	for (j = 0; j < 10; j++) {
		lcd_goto(0, 2);
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)random(9);
		}
		lcd_write(ubuf, 20);

		for (i = 0; i < 20; i++) {
			lcd_goto(i, 3);
			ubuf[0] = 7;
			lcd_write(ubuf, 1);
			lcd_goto(i, 3);
			ubuf[0] = ' ';
			lcd_write(ubuf, 1);
		}
		for (i = 19; i >= 0; i--) {
			lcd_goto(i, 3);
			ubuf[0] = 7;
			lcd_write(ubuf, 1);
			lcd_goto(i, 3);
			ubuf[0] = ' ';
			lcd_write(ubuf, 1);
		}
	}
	delay(1000);

	/* Cycle through the eight custom nav symbols */
	lcd_define_nav_symbols();

	for (j = 0; j < 8; j++) {
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)j;
		}
		lcd_goto(0, 0);
		lcd_write(ubuf, 20);
		lcd_write(ubuf, 20);
		lcd_write(ubuf, 20);
		lcd_write(ubuf, 20);
		delay(100);
	}

	/* Full-screen dancing vertical bars */
	lcd_define_vbar_symbol();

	for (j = 0; j < 100; j++) {
		lcd_goto(0, 0);
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)random(9);
		}
		lcd_write(ubuf, 20);
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)random(9);
		}
		lcd_write(ubuf, 20);
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)random(9);
		}
		lcd_write(ubuf, 20);
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)random(9);
		}
		lcd_write(ubuf, 20);
	}

	lcd_cls();

	/* Cylon / Knight Rider scan, all four rows */
	for (j = 0; j < 4; j++) {
		for (k = 0; k < 3; k++) {
			for (i = 0; i < 20; i++) {
				lcd_goto(i, j);
				ubuf[0] = 7;
				lcd_write(ubuf, 1);
				delay(10);
				lcd_goto(i, j);
				ubuf[0] = ' ';
				lcd_write(ubuf, 1);
				delay(10);
			}
			for (i = 19; i >= 0; i--) {
				lcd_goto(i, j);
				ubuf[0] = 7;
				lcd_write(ubuf, 1);
				delay(10);
				lcd_goto(i, j);
				ubuf[0] = ' ';
				lcd_write(ubuf, 1);
				delay(10);
			}
		}
	}

	lcd_cls();

	/* Full 0..255 charset sweep with ASCII row-4 readout */
	for (c = 0; c <= 255; c++) {
		for (i = 0; i < 20; i++) {
			ubuf[i] = (unsigned char)c;
		}
		lcd_goto(0, 0);
		lcd_write(ubuf, 20);
		lcd_goto(0, 1);
		lcd_write(ubuf, 20);
		lcd_goto(0, 2);
		lcd_write(ubuf, 20);

		sprintf((char*)buf, "Text %03d            ", c);
		buf[20] = 0;
		lcd_goto(0, 3);
		lcd_puts((const char*)buf);

		delay(200);
	}
}
