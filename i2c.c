/** 
 * @file i2c.c
 * Linux I2C Driver for device IO
 * Setup I2C port on Raspberry Pi to blast bytes over I2C device descriptor;
 * should work on any other Linux I2C device. 
 *
 * @copyright
 * Copyright (C) 2017 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 */

/**
 * @defgroup serial serial
 * @addtogroup serial
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

static const char *device = "/dev/i2c-1";

#define OLED_DEVICE_ADDRESS     0x3c /* SSD1311 */

/**
 * @brief serial port open
 * 
 * Open the I2C device file (e.g. /dev/i2c-1) for read/write
 * shutdown via #serial_close 
 *
 * @param device string value for the device file (e.g. "/dev/i2c-1")
 * @param highspeed ignored
 */
int serial_open(char* device, int highspeed) 
{
	int fd = -1;
	int ret; 

	fd = open(device, O_RDWR);
	if (fd < 0) { 
		printf("device not found[%s]\n", device);
		return fd; 
	}
	if (ioctl(fd, I2C_SLAVE, OLED_DEVICE_ADDRESS) < 0) {  
		printf("unable to get bus access to talk to slave[0x%02x]\n", 
			OLED_DEVICE_ADDRESS);
		return fd;
	}   
	printf("%s: i2c mode\n", device);

	return fd;
}

/**
 * @brief serial port close
 * 
 * Close the I2C port file descriptor
 *
 * @param sfd serial file descriptor returned from #serial_open
 */
void serial_close(int sfd)
{	
	close(sfd);
}



/** @}*/
