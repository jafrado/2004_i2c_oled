/**
 * @file ssd13xx_io_arduino.cpp
 * Arduino I2C transport + millisecond delay glue for the common
 * ssd13xx_20x4_oled.c driver.
 *
 * Tested target: Arduino Uno R4 Minima (Renesas RA4M1).
 *   SDA on the dedicated SDA pin (also exposed on A4)
 *   SCL on the dedicated SCL pin (also exposed on A5)
 *   Module power: 3V3 or 5V per Wide.HK board configuration, plus GND.
 *
 * The common driver in ssd13xx_20x4_oled.c declares two unresolved
 * symbols it expects the platform to provide:
 *
 *   void ssd13xx_write(unsigned char addr, unsigned char data);
 *   void ms_delay(unsigned int ms);
 *
 * This file supplies both, backed by the Arduino Wire library and
 * the Arduino delay() primitive.
 *
 * @copyright
 * Copyright (C) 2024 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 */

#include <Arduino.h>
#include <Wire.h>

#define OLED_SADDR 0x3c /* SSD1311/OLED Module 7-bit slave address */

extern "C" {

/**
 * @brief SSD13xx I/O write routine (Arduino / Wire)
 *
 * Push a single (register, data) byte pair to the OLED via I2C.
 * Mirrors the Linux ssd13xx_io.c contract used by ssd13xx_20x4_oled.c.
 */
void ssd13xx_write(unsigned char addr, unsigned char data)
{
	Wire.beginTransmission(OLED_SADDR);
	Wire.write(addr);
	Wire.write(data);
	Wire.endTransmission();
}

/**
 * @brief millisecond delay
 *
 * The common driver names this argument "usecs" but consistently
 * passes a millisecond count (e.g. ms_delay(100) for 100 ms in
 * lcd_init). Map straight to Arduino delay().
 */
void ms_delay(unsigned int ms)
{
	delay(ms);
}

} /* extern "C" */
