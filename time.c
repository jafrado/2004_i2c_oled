/** 
 * @file time.c
 * Linux time delay functions based on select
 *
 * @copyright
 * Copyright (C) 2017 Real Flight Systems
 * @author James F Dougherty <jfd@realflightsystems.com>
 */

/**
 * @defgroup time
 * @addtogroup time
 * @{
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

/** 
 * @brief microsecond delay
 * 
 *  Delay specified number of microseconds, the processor will wait
 *  this amount of time before continuing execution.
 * 
 * @param[in] usecs the number of microseconds to sleep
 */
void us_delay(unsigned int usecs)
{
	int rv;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = usecs;
	do {
		rv = select(1,NULL,NULL,NULL,&tv);
	} while( (rv  == -1) && (errno == EINTR) );
}
/** 
 * @brief millisecond delay
 * 
 *  Delay specified number of milliseconds, the processor will wait
 *  this amount of time before continuing execution.
 * 
 * @param[in] usecs the number of milliseconds to sleep
 */
void ms_delay(unsigned int usecs)
{
	return us_delay(usecs*1000);
}

/** @}*/
