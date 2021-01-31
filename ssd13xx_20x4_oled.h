#ifndef __SSD13XX_OLED_H_
#define __SSD13XX_OLED_H_

/** 
 * @brief display string
 * 
 * Write null-terminated string to display. The text is output
 * at the current position. 
 * 
 * @param[in] str the null terminated string to display
 */
void lcd_puts(const char *str);
/** 
 * @brief display data
 * 
 * Write binary data to display. The data is output
 * at the current position. 
 * 
 * @param[in] data the data bytes to write to the display
 * @param[in] len the number of bytes to write
 */
void lcd_write(const unsigned char *data, int len);

/** 
 * @brief clear screen
 * 
 * This routine clears the display memory
 * 
 */
void lcd_cls(void);


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
void lcd_goto(int x, int y);

/*
 * We set the CGRAM address of a character, and then we write the bits.
 * 
 * Each font is either 5x8 or 5x6 bits
 */
void set_cgram_addr(char addr);
void set_dram_addr(char addr);

void lcd_init(void);
void lcd_startup_banner(void);
void lcd_define_nav_symbols(void);
void lcd_define_vbar_symbol(void);

/** 
 * @brief clear screen
 * 
 * This routine clears the display memory
 * 
 */
void lcd_cls(void);


#endif /* !__SSD13XX_OLED_H_ */

