/** 
 * @file ssd13xx_io.c
 * SSD1311 IO routines for Linux/I2C
 *
 * @copyright
 * Copyright (C) 2017 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 */

/**
 * @defgroup ssd13xx_i2c
 * @addtogroup ssd13xx_i2c
 * @{
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>

/* from your main */
extern int i2c_fd;

/**
 * @brief SSD13xx IO write routine
 * 
 * Write bytes to SSD1311 device
 *
 * @param[in] addr address to write to
 * @param[in] data data to write
 */
void ssd13xx_write(unsigned char addr, unsigned char data)
{
	char cmd[2];
	cmd[0] = addr;
	cmd[1] = data;
	if (write(i2c_fd, cmd, 2) != 2){
		printf("%s:%d: error writing to i2c slave addr[%02x]\n", 
		       __FILE__, __LINE__, addr);
        }
}	 

/** @}*/
