/****************************************************************************
 * drivers/lcd/lcd_2004.c
 * Display Driver for Wide.HK OLED 20x4 I2C Text Mode Display
 * 
 *   Copyright (C) 2020 James F Dougherty. All rights reserved.
 *   Author: James F Dougherty <jfd@realflightsystems.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd_alpha.h>
#include <nuttx/random.h>
#include <memory.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LCD_20X4_I2C_FREQUENCY
#define CONFIG_LCD_20X4_I2C_FREQUENCY 100000
#endif

/* LCD_20X4 dev */
struct lcd20x4_dev_s {
	FAR struct lcd20x4_dev_s *flink; /* Linked list of drivers */
	FAR struct i2c_master_s *i2c;   /* I2C interface */
	uint8_t                 addr;   /* I2C address */
	int                     freq;   /* LCD_20X4 I2C Frequency    */
	sem_t                datasem;   /* Manages exclusive access */
};

/* Character Driver Methods */
static int     lcd20x4_open(FAR struct file *filep);
static int     lcd20x4_close(FAR struct file *filep);
static ssize_t lcd20x4_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t lcd20x4_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     lcd20x4_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fops =
{
	lcd20x4_open,
	lcd20x4_close,
	lcd20x4_read,
	lcd20x4_write,
	NULL,
	lcd20x4_ioctl
#ifndef CONFIG_DISABLE_POLL
	, NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	, NULL
#endif
};

/****************************************************************************
 * Name: lcd20x4_open
 *
 * Description:
 *   This method is called when the device is opened.
 *
 ****************************************************************************/
static int lcd20x4_open(FAR struct file *filep)
{
	FAR struct inode *inode        = filep->f_inode;
	FAR struct lcd20x4_dev_s *priv  = inode->i_private;
	sninfo("open");
	return OK;
}

/****************************************************************************
 * Name: lcd20x4_close
 *
 * Description:
 *   This method is called when the device is closed.
 *
 ****************************************************************************/
static int lcd20x4_close(FAR struct file *filep)
{
	FAR struct inode *inode        = filep->f_inode;
	FAR struct lcd20x4_dev_s *priv  = inode->i_private;
	
	return OK;
}

/****************************************************************************
 * Name: lcd20x4_read
 *
 * Description:
 *   A dummy read method.
 *
 ****************************************************************************/
static ssize_t lcd20x4_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
	FAR struct inode *inode        = filep->f_inode;
	FAR struct lcd20x4_dev_s *priv  = inode->i_private;
	sninfo("read");	
	return 0;
}

/****************************************************************************
 * Name: lcd20x4_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/
static ssize_t lcd20x4_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
	return buflen;
}

/****************************************************************************
 * Name: lcd20x4_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/
static int lcd20x4_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode        *inode = filep->f_inode;
	FAR struct lcd20x4_dev_s *priv  = inode->i_private;
	int                      ret   = OK;
	struct slcd_curpos_s position;
	FAR struct slcd_curpos_s *data = NULL;
		
	/* Aquire the semaphore before the data is copied */	
	ret = nxsem_wait(&priv->datasem);
	if (ret < 0)
	{
		snerr("ERROR: Could not aquire priv->datasem: %d\n", ret);
		return ret;
	}
	
	/* Handle ioctl commands */	  
	switch (cmd)
	{
		
	case LCD_ALPHA_GOTO_XY:
		data = (FAR struct slcd_curpos_s *)arg;
		//sninfo("goto: %d,%d", data->row, data->column);		
		lcd_goto(priv, data->row, data->column);
		break;
	case LCD_ALPHA_CLS:
		lcd_cls(priv);
		break;
	case LCD_ALPHA_PUTS:
		lcd_puts(priv, (char*)arg);
		break;
	  	
	default:
		snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
		ret = -ENOTTY;
		break;
	}

	/* Give back the semaphore */	
	nxsem_post(&priv->datasem);
	
	return ret;
}

/****************************************************************************
 * Name: ssd13xx_write
 *
 * Description:
 *   Write an 8-bit data byte at address
 *
 ****************************************************************************/

int ssd13xx_write(FAR struct lcd20x4_dev_s *priv, uint8_t addr, uint8_t regval)
{
	/* 8-bit data read sequence:
	 *
	 *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data - STOP
	 */
	struct i2c_msg_s msg;
	uint8_t txbuffer[2];
	int ret;
	
#ifdef CONFIG_LCD_SSD1306_REGDEBUG
	_err("-> 0x%02x\n", regval);
#endif
	
  /* Setup to the data to be transferred.  Two bytes:  The SSD1306 register
   * address followed by one byte of data.
   */
	
	txbuffer[0]   = addr;
	txbuffer[1]   = regval;
	
	/* Setup 8-bit SSD1306 address write message */
	
	msg.frequency = CONFIG_LCD_20X4_I2C_FREQUENCY;  /* I2C frequency */
	msg.addr      = priv->addr;              /* 7-bit address */
	msg.flags     = 0;                       /* Write transaction, beginning with START */
	msg.buffer    = txbuffer;                /* Transfer from this address */
	msg.length    = 2;                       /* Send two bytes following the address
						  * then STOP */
	
	/* Perform the transfer */
	ret = I2C_TRANSFER(priv->i2c, &msg, 1);
	if (ret < 0) {
		lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
	}
	
	return ret;
}


#define OLED_SADDR         0x3c /* SSD1311/OLED Module Slave address */
#define SSD1311_CMD_REG    0x80 /* Command Register                  */
#define SSD1311_DATA_REG   0x40 /* Data Register                     */

#define ssd13xx_command(priv, data)		\
        ssd13xx_write(priv, SSD1311_CMD_REG, (data))
#define ssd13xx_data(priv, data)		\
        ssd13xx_write(priv, SSD1311_DATA_REG, (data))

/* Library routines */

/** 
 * @brief display string
 * 
 * Write null-terminated string to display. The text is output
 * at the current position. 
 * 
 * @param[in] str the null terminated string to display
 */
void lcd_puts(FAR struct lcd20x4_dev_s *priv, const char *str)
{
	unsigned char i = 0;
	while ( str[i] ) {
		ssd13xx_data(priv, str[i]);
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
void lcd_write(FAR struct lcd20x4_dev_s *priv, const unsigned char *data, int len)
{
	unsigned char i = 0;
	while (i < len) {
		ssd13xx_data(priv, data[i]);      
		i++;
	}
}

/** 
 * @brief clear screen
 * 
 * This routine clears the display memory
 * 
 */
void lcd_cls(FAR struct lcd20x4_dev_s *priv)
{ 
	ssd13xx_command(priv, 0x1);    
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
void lcd_goto(FAR struct lcd20x4_dev_s *priv, int x, int y)
{
	ssd13xx_command(priv, 0x80 + 0x20 * y + x);
}

/*
 * We set the CGRAM address of a character, and then we write the bits.
 * 
 * Each font is either 5x8 or 5x6 bits
 */
void set_cgram_addr(FAR struct lcd20x4_dev_s *priv, char addr)
{
	ssd13xx_command(priv, 0x40 + addr);  
}

void set_dram_addr(FAR struct lcd20x4_dev_s *priv, char addr)
{
	ssd13xx_command(priv, 0x80 + addr);  
}

void lcd_define_degree_symbol(FAR struct lcd20x4_dev_s *priv)
{
	set_cgram_addr(priv, 0); /* Degrees Symbol */
	ssd13xx_data(priv, 0x0c);
	ssd13xx_data(priv, 0x12);
	ssd13xx_data(priv, 0x12);
	ssd13xx_data(priv, 0x0c);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);

}
void lcd_define_minute_symbol(FAR struct lcd20x4_dev_s *priv)
{
	set_cgram_addr(priv, 8); /* Minutes Symbol */
	ssd13xx_data(priv, 0x08);
	ssd13xx_data(priv, 0x10);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
}

void lcd_define_feet_symbol(FAR struct lcd20x4_dev_s *priv)
{
	set_cgram_addr(priv, 16); /* Feet Symbol */
	ssd13xx_data(priv, 0x1c);
	ssd13xx_data(priv, 0x10);
	ssd13xx_data(priv, 0x18);
	ssd13xx_data(priv, 0x10);
	ssd13xx_data(priv, 0x17);
	ssd13xx_data(priv, 0x02);
	ssd13xx_data(priv, 0x02);
	ssd13xx_data(priv, 0x02);
	
}
void lcd_define_airplane_symbol(FAR struct lcd20x4_dev_s *priv)
{
	set_cgram_addr(priv, 24); /* Airplane / Fly / Lock */
	ssd13xx_data(priv, 0x04);
	ssd13xx_data(priv, 0x04);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x04);
	ssd13xx_data(priv, 0x04);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x00);
}

void lcd_define_nav_symbols(FAR struct lcd20x4_dev_s *priv)
{

	lcd_define_degree_symbol(priv);
	lcd_define_minute_symbol(priv);
	lcd_define_feet_symbol(priv);
	lcd_define_airplane_symbol(priv);
	
	
	set_cgram_addr(priv, 32); /* Sat 0 */
	ssd13xx_data(priv, 0x11);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x11);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	
	set_cgram_addr(priv, 40); /* Sat 1 */
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	
	
	set_cgram_addr(priv, 48); /* Sat 2 */
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	
	set_cgram_addr(priv, 56); /* Sat 3 */
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x15);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	ssd13xx_data(priv, 0x0e);
	
}


void lcd_define_vbar_symbol(FAR struct lcd20x4_dev_s *priv)
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
	set_cgram_addr(priv, 0);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	
	set_cgram_addr(priv, 8);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	
	
	set_cgram_addr(priv, 16);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	
	
	set_cgram_addr(priv, 24);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	

	set_cgram_addr(priv, 32);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	
	
	set_cgram_addr(priv, 40);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	
	
	set_cgram_addr(priv, 48);
	ssd13xx_data(priv, 0x00);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	
	set_cgram_addr(priv, 56);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	ssd13xx_data(priv, 0x1f);
	set_dram_addr(priv, 0);
}


void ssd13xx_oled_20x4_init(FAR struct lcd20x4_dev_s *priv)
{

	ssd13xx_command(priv, 0x80);
	ssd13xx_command(priv, 0x2A);  /* **** Set "RE"=1  00101010B */
	ssd13xx_command(priv, 0x71);
	ssd13xx_command(priv, 0xC0);
	ssd13xx_command(priv, 0x00);
	ssd13xx_command(priv, 0x28);

	ssd13xx_command(priv, 0x08); /* **** Set Sleep Mode On     */
	ssd13xx_command(priv, 0x2A); /* **** Set "RE"=1  00101010B */
	ssd13xx_command(priv, 0x79); /* **** Set "SD"=1  01111001B */
	
	ssd13xx_command(priv, 0xD5);
	ssd13xx_command(priv, 0x70);
	ssd13xx_command(priv, 0x78); /* **** Set "SD"=0            */
	
	//ssd13xx_command(priv, 0x08);
	/* **** Set 5-dot, 3 or 4 line(0x09), 1 or 2 line(0x08) */
	ssd13xx_command(priv, 0x09); 
	
	
	ssd13xx_command(priv, 0x06); /* **** Set Com31-->Com0  Seg0-->Seg99 */
	ssd13xx_command(priv, 0x72);
	ssd13xx_command(priv, 0xC0);
	ssd13xx_command(priv, 0x01);
	
	/**** Set OLED Characterization ***/
	ssd13xx_command(priv, 0x2A);   /* **** Set "RE"=1  */
	ssd13xx_command(priv, 0x79);   /* **** Set "SD"=1 */

	/**** CGROM/CGRAM Management ***/
#if 0	
	ssd13xx_command(priv, 0x72); /* **** Set ROM */
	ssd13xx_command(priv, 0x00); /*  **** Set ROM A and 8 CGRAM */
#endif
	
	ssd13xx_command(priv, 0xDC);    /* **** Set ROM */
	ssd13xx_command(priv, 0x00);    /* **** Set ROM A and 8 CGRAM */
	
	ssd13xx_command(priv, 0xDA);    /* **** Set Seg Pins HW Config */
	ssd13xx_command(priv, 0x10);   
	
	ssd13xx_command(priv, 0x81);    /* **** Set Contrast */
	ssd13xx_command(priv, 0xD9);   
	ssd13xx_command(priv, 0x8F);    /* **** Set Contrast */
	
	ssd13xx_command(priv, 0xF1); 
	
	ssd13xx_command(priv, 0xDB);   /* **** Set VCOM deselect level */
	ssd13xx_command(priv, 0x30);   /* **** VCC x 0.83              */
	
	ssd13xx_command(priv, 0xDC);   /* *Set gpio -turn EN for 15V generator on. */
	ssd13xx_command(priv, 0x03);
	
	ssd13xx_command(priv, 0x78);   /* **** Exiting Set OLED Characterization */
	ssd13xx_command(priv, 0x28); 
	
	//flip display with these two lines, comment out the 0x06 write below
	//ssd13xx_command(priv, 0x2A); 
	//ssd13xx_command(priv, 0x05);   /* **** Set Entry Mode (invert) */
	
	ssd13xx_command(priv, 0x06);     /* **** Set Entry Mode */
	
	ssd13xx_command(priv, 0x28);     /* **** Set "IS"=0 , "RE" =0 /28 */
	ssd13xx_command(priv, 0x01); 
	ssd13xx_command(priv, 0x80);     /* Set DDRAM Address to 0x80 (line 1 start)*/
	
	up_mdelay(100);
	ssd13xx_command(priv, 0x0C);   /* **** Turn on Display */
	
}

void lcd_startup_banner(FAR struct lcd20x4_dev_s *priv)
{
	lcd_goto(priv, 0,0);
	lcd_cls(priv);
}

void lcd_init(FAR struct lcd20x4_dev_s *priv)
{
	sninfo("ssd13xx_init\n");
	ssd13xx_oled_20x4_init(priv);
	up_mdelay(100);
	
}

/****************************************************************************
 * Name: lcd20x4_register
 *
 * Description:
 *   Register the LCD_20X4 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g., "/dev/press0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LCD_20X4.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int lcd20x4_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
	FAR struct lcd20x4_dev_s *priv;
	int ret = 0;
	char ubuf[40];
	
	/* Sanity check */
	DEBUGASSERT(i2c != NULL);

	/* Initialize the device's structure */
	priv = (FAR struct lcd20x4_dev_s *)kmm_malloc(sizeof(*priv));
	if (priv == NULL)
	{
		snerr("ERROR: Failed to allocate instance\n");
		return -ENOMEM;
	}

	priv->i2c   = i2c;
	priv->addr  = OLED_SADDR;
	
	/* Register the character driver */
	ret = register_driver(devpath, &g_fops, 0666, priv);
	if (ret < 0)
	{
		snerr("ERROR: Failed to register driver: %d\n", ret);
		goto errout;
	}

	/* init display */
	lcd_init(priv);
	lcd_startup_banner(priv);
	lcd_cls(priv);
	
	strcpy((char*)ubuf, "      Wide.HK       ");
	lcd_write(priv, ubuf,20);
	
	lcd_puts(priv, "20x4 Display Init Ok!"); 
	return ret;
	
errout:
	kmm_free(priv);

	return ret;
}


