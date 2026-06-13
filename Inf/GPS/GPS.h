#ifndef __GPS_H
#define __GPS_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "com_comfig.h"
#include "com_filter.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "app_task.h"
#define BUFFER_SIZE 400
extern unsigned char GPSRX_BUFF[BUFFER_SIZE];
extern _SaveData Save_Data;

void parseGpsBuffer(void);
void check_gps(void);
void GPS_Filter(_falsh_savedate *gps_mpu_save);

#endif
