#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "led.h"
#include "app_date.h"
#include "app_save.h"
#include "com_comfig.h"
#include "GPS.h"


void App_Task_FreeRTOSStart(void);

extern SemaphoreHandle_t GPS_Handle;
#endif
