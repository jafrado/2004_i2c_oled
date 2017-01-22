/** 
 * @brief Display Driver for Wide.HK OLED 20x4 I2C Text Mode Display
 * 
 * This module implements Text output and positioning for the Wide.HK
 * 20x4 OLED Character display found online at the listing shown on the
 * <A HREF="http://www.ebay.com/itm/162156498430"> Wide.HK Ebay Store</a>
 * 
 * Reference:
 * 
 * @copyright
 * Copyright (C) 2017 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 * 
 *
 */

/**
 * @defgroup display display
 * @addtogroup display
 * @file ssd13xx_20x4_oled.c 
 * @{
 */

/* device dependencies */

/* time.c */
extern void ms_delay(unsigned int usecs);

/* ssd13xx_io.c */
extern void ssd13xx_write(unsigned char addr, unsigned char data);

#define OLED_SADDR         0x3c /* SSD1311/OLED Module Slave address */
#define SSD1311_CMD_REG    0x80 /* Command Register                  */
#define SSD1311_DATA_REG   0x40 /* Data Register                     */

#define ssd13xx_command(data) \
	ssd13xx_write(SSD1311_CMD_REG, (data))
#define ssd13xx_data(data) \
	ssd13xx_write(SSD1311_DATA_REG, (data))

/* Library routines */

/** 
 * @brief display string
 * 
 * Write null-terminated string to display. The text is output
 * at the current position. 
 * 
 * @param[in] str the null terminated string to display
 */
void lcd_puts(const char *str)
{
	unsigned char i = 0;
	while ( str[i] ) {
		ssd13xx_data(str[i]);
		i++;
	}
}

/** 
 * @brief display data
 * 
 * Write binary data to display. The data is output
 * at the current position. 
 * 
 * @param[in] data the data bytes to write to the display
 * @param[in] len the number of bytes to write
 */
void lcd_write(const unsigned char *data, int len)
{
	unsigned char i = 0;
	while (i < len) {
		ssd13xx_data(data[i]);      
		i++;
	}
}

/** 
 * @brief clear screen
 * 
 * This routine clears the display memory
 * 
 */
inline void lcd_cls(void)
{ 
	ssd13xx_command(0x1);    
}

/** 
 * @brief move the cursor location
 * 
 * This routine moves the cursor to x,y location by setting the
 * CGRAM output pointer (AC) to the correct DRAM address. 
 *  
 * Cursor locations are 0 based and go from: 0-3 (y) and 0-19 (x)
 * 0,0 is the upper left hand corner of the display itself. 
 * 
 * @param[in] x the x position to move to
 * @param[in] y the y position to move to
 */
void lcd_goto(int x, int y)
{
	ssd13xx_command(0x80 + 0x20 * y + x);
}

/*
 * We set the CGRAM address of a character, and then we write the bits.
 * 
 * Each font is either 5x8 or 5x6 bits
 */
inline void set_cgram_addr(char addr)
{
	ssd13xx_command(0x40 + addr);  
}

inline void set_dram_addr(char addr)
{
	ssd13xx_command(0x80 + addr);  
}

void lcd_define_degree_symbol(void)
{
	set_cgram_addr(0); /* Degrees Symbol */
	ssd13xx_data(0x0c);
	ssd13xx_data(0x12);
	ssd13xx_data(0x12);
	ssd13xx_data(0x0c);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);

}
void lcd_define_minute_symbol(void)
{
	set_cgram_addr(8); /* Minutes Symbol */
	ssd13xx_data(0x08);
	ssd13xx_data(0x10);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
}

void lcd_define_feet_symbol(void)
{
	set_cgram_addr(16); /* Feet Symbol */
	ssd13xx_data(0x1c);
	ssd13xx_data(0x10);
	ssd13xx_data(0x18);
	ssd13xx_data(0x10);
	ssd13xx_data(0x17);
	ssd13xx_data(0x02);
	ssd13xx_data(0x02);
	ssd13xx_data(0x02);
	
}
void lcd_define_airplane_symbol(void)
{
	set_cgram_addr(24); /* Airplane / Fly / Lock */
	ssd13xx_data(0x04);
	ssd13xx_data(0x04);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x04);
	ssd13xx_data(0x04);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x00);
}

void lcd_define_nav_symbols(void)
{

	lcd_define_degree_symbol();
	lcd_define_minute_symbol();
	lcd_define_feet_symbol();
	lcd_define_airplane_symbol();
	
	
	set_cgram_addr(32); /* Sat 0 */
	ssd13xx_data(0x11);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x11);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	
	set_cgram_addr(40); /* Sat 1 */
	ssd13xx_data(0x15);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x15);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	
	
	set_cgram_addr(48); /* Sat 2 */
	ssd13xx_data(0x15);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x15);
	ssd13xx_data(0x00);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	
	set_cgram_addr(56); /* Sat 3 */
	ssd13xx_data(0x15);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x15);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	ssd13xx_data(0x0e);
	
}


void lcd_define_vbar_symbol(void)
{
	int i;
	
	/*
	 *  Airplane Symbol
	 *   00100
	 *   00100
	 *   01110
	 *   11111
	 *   00100
	 *   00100
	 *   01110
	 *   00000 
	 */
	set_cgram_addr(0);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	
	set_cgram_addr(8);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	
	
	set_cgram_addr(16);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	
	
	set_cgram_addr(24);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	

	set_cgram_addr(32);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	
	
	set_cgram_addr(40);
	ssd13xx_data(0x00);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	
	
	set_cgram_addr(48);
	ssd13xx_data(0x00);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	
	set_cgram_addr(56);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	ssd13xx_data(0x1f);
	set_dram_addr(0);
}


void ssd13xx_oled_20x4_init(void)
{
	ssd13xx_command(0x80);
	ssd13xx_command(0x2A);  /* **** Set "RE"=1  00101010B */
	ssd13xx_command(0x71);
	ssd13xx_command(0xC0);
	ssd13xx_command(0x00);
	ssd13xx_command(0x28);
	
	ssd13xx_command(0x08); /* **** Set Sleep Mode On     */
	ssd13xx_command(0x2A); /* **** Set "RE"=1  00101010B */
	ssd13xx_command(0x79); /* **** Set "SD"=1  01111001B */
	
	ssd13xx_command(0xD5);
	ssd13xx_command(0x70);
	ssd13xx_command(0x78); /* **** Set "SD"=0            */
	
	//ssd13xx_command(0x08);
	/* **** Set 5-dot, 3 or 4 line(0x09), 1 or 2 line(0x08) */
	ssd13xx_command(0x09); 
	
	
	ssd13xx_command(0x06); /* **** Set Com31-->Com0  Seg0-->Seg99 */
	ssd13xx_command(0x72);
	ssd13xx_command(0xC0);
	ssd13xx_command(0x01);
	
	/**** Set OLED Characterization ***/
	ssd13xx_command(0x2A);   /* **** Set "RE"=1  */
	ssd13xx_command(0x79);   /* **** Set "SD"=1 */

	/**** CGROM/CGRAM Management ***/
#if 0	
	ssd13xx_command(0x72); /* **** Set ROM */
	ssd13xx_command(0x00); /*  **** Set ROM A and 8 CGRAM */
#endif
	
	ssd13xx_command(0xDC);    /* **** Set ROM */
	ssd13xx_command(0x00);    /* **** Set ROM A and 8 CGRAM */
	
	ssd13xx_command(0xDA);    /* **** Set Seg Pins HW Config */
	ssd13xx_command(0x10);   
	
	ssd13xx_command(0x81);    /* **** Set Contrast */
	ssd13xx_command(0xD9);   
	ssd13xx_command(0x8F);    /* **** Set Contrast */
	
	ssd13xx_command(0xF1); 
	
	ssd13xx_command(0xDB);   /* **** Set VCOM deselect level */
	ssd13xx_command(0x30);   /* **** VCC x 0.83              */
	
	ssd13xx_command(0xDC);   /* *Set gpio -turn EN for 15V generator on. */
	ssd13xx_command(0x03);
	
	ssd13xx_command(0x78);   /* **** Exiting Set OLED Characterization */
	ssd13xx_command(0x28); 
	
	//flip display with these two lines, comment out the 0x06 write below
	//ssd13xx_command(0x2A); 
	//ssd13xx_command(0x05);   /* **** Set Entry Mode (invert) */
	
	ssd13xx_command(0x06);     /* **** Set Entry Mode */
	
	ssd13xx_command(0x28);     /* **** Set "IS"=0 , "RE" =0 /28 */
	ssd13xx_command(0x01); 
	ssd13xx_command(0x80);     /* Set DDRAM Address to 0x80 (line 1 start)*/
	
	ms_delay(100);
	ssd13xx_command(0x0C);   /* **** Turn on Display */
	
}

void lcd_init(void)
{
	ssd13xx_oled_20x4_init();
	ms_delay(100);
	
}
void lcd_startup_banner(void)
{
	lcd_goto(0,0);
	lcd_cls();
}

/** @}*/
