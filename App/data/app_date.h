#ifndef __APP_DATE_H
#define __APP_DATE_H

#include "FreeRTOS.h"
#include "task.h"
#include "MPU6050.h"
#include "com_filter.h"
#include "com_comfig.h"
#include "com_IMU.h"
#include "GPS.h"


void Date_MPU6050_get_filter(GyroAccel_Struct *gyroAccel);
void Date_MPU6050_get_Yaw(GyroAccel_Struct  *gyroAccel,
                           EulerAngle_Struct *eulerAngle,
						   _falsh_savedate *gps_mpu_save,
                           float              dt);
void Date_GPS_get(_falsh_savedate *gps_mpu_save);


#endif

