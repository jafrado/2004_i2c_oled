/** 
 * @brief Display Driver for Wide.HK OLED 20x4 I2C Text Mode Display
 * 
 * This module implements Text output and positioning for the Wide.HK
 * 20x4 OLED Character display found online at the listing shown on the
 * <A HREF="https://www.ebay.com/itm/IIC-I2C-2004-20x4-Green-OLED-Module-Display-For-Arduino-PIC-AVR-ARM/162406508569"> Wide.HK Ebay Store</a>
 * 
 * Reference:
 * 
 * @copyright
 * Copyright (C) 2016 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 * 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include "ssd13xx_20x4_oled.h"

extern int serial_open(char* device, int highspeed) ; /* i2c.c */
extern void ms_delay(unsigned int usecs); /* time.c */

int i2c_fd = -1;

int main(int argc, char* argv[])
{
	int i = 0, c = 0, j = 0, k = 0;
	unsigned char buf[256];
	unsigned char ubuf[256];

	/* Compass arrows */
	unsigned char arrows[] = { 222, 23, 223, 25, 224, 24, 225, 22};

	if (argc != 2) { 
		printf ("usage: %s <comport> (e.g. /dev/i2c-1) \n", argv[0]);
		exit(1);
	}
	if ((i2c_fd = serial_open(argv[1],0)) < 0) { 
		printf("error: could not open [%s]\n", argv[1]);
		exit(2);
	}


	lcd_init();
	lcd_startup_banner();

	while(1){		
		
		lcd_cls();

		strcpy((char*)ubuf, "      Wide.HK       ");
		lcd_write(ubuf,20);
		
		strcpy((char*)ubuf, "  20x4 OLED Diplay  ");
		lcd_write(ubuf,20);

		
		for (k = 0; k < 3; k++)
			for (j = 0; j < 8; j++) { 
				ubuf[0] = arrows[j];
				lcd_goto(0,0);
				lcd_write(ubuf,1);
				ms_delay(50);
			}
		
		/* Loop through second character set - Dancing vertical bars */
		lcd_define_vbar_symbol();
		
		for (j = 0; j < 10; j++) { 
			lcd_goto(0,2);
			
			for (i = 0; i < 20; i++) 
				ubuf[i] = random() % 9 ;
			lcd_write(ubuf,20);
			
			for (i = 0; i < 20; i++) {
				lcd_goto(i, 3);
				ubuf[0] = 7;
				lcd_write(ubuf,1);
				lcd_goto(i, 3);
				ubuf[0] = ' ';
				lcd_write(ubuf, 1);
			}
			for (i = 19; i >= 0; i--) {
				lcd_goto(i, 3);
				ubuf[0] = 7;
				lcd_write(ubuf,1);
				lcd_goto(i, 3);
				ubuf[0] = ' ';
				lcd_write(ubuf, 1);
			}
			
			
		}
		ms_delay(1000);		

		/* Loop through first character set - nav symbols */
		lcd_define_nav_symbols();
		
		for (j = 0; j < 8; j++) { 
			for (i = 0; i < 20; i++) { 
				ubuf[i] = j;
			}
			lcd_goto(0,0);
			
			lcd_write(ubuf,20);
			lcd_write(ubuf,20);
			lcd_write(ubuf,20);
			lcd_write(ubuf,20);
			
			ms_delay(100);
			
		}
		
		/* Load vertical bar charset -then make Dancing vertical bars */
		lcd_define_vbar_symbol();
		
		for (j = 0; j < 100; j++) { 
			lcd_goto(0,0);
			
			for (i = 0; i < 20; i++) 
				ubuf[i] = random() % 9;
			lcd_write(ubuf,20);
			
			for (i = 0; i < 20; i++) 
				ubuf[i] = random() % 9;
			lcd_write(ubuf,20);
			
			for (i = 0; i < 20; i++) 
				ubuf[i] = random() % 9;
			lcd_write(ubuf,20);
			
			for (i = 0; i < 20; i++) 
				ubuf[i] = random() % 9;
			lcd_write(ubuf,20);
			
		}
		
		/* clear screen */
		lcd_cls();

		/* Cylon/Nightrider display - back and forth each line */
		for(j = 0; j < 4; j++) {
			for (k = 0; k < 3; k++) {         
				for (i = 0; i < 20; i++) {
					lcd_goto(i, j);
					ubuf[0] = 7;
					lcd_write(ubuf,1);
					ms_delay(10);
					lcd_goto(i, j);
					ubuf[0] = ' ';
					lcd_write(ubuf, 1);
					ms_delay(10);
				}
				for (i = 19; i >= 0; i--) {
					lcd_goto(i, j);
					ubuf[0] = 7;
					lcd_write(ubuf,1);
					ms_delay(10);
					lcd_goto(i, j);
					ubuf[0] = ' ';
					lcd_write(ubuf, 1);
					ms_delay(10);
				}
			}
		}
		
		/* Clear screen */
		lcd_cls();
		
		/* Load character set and display each character */
		for (c = 0; c <= 255; c++) { 
			
			for (i = 0; i < 20; i++) { 
				ubuf[i] = c;
			}
			
			lcd_goto(0,0);
			lcd_write(ubuf, 20);
			lcd_goto(0,1);
			lcd_write(ubuf, 20);
			lcd_goto(0,2);
			lcd_write(ubuf, 20);
			
			sprintf(buf, "Text %03d            ", c);
			buf[20] = 0;
			lcd_goto(0,3);
			lcd_puts(buf);

			ms_delay(200);
			
		}
	}

	return 0;
}
	
