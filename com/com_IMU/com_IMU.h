#ifndef __COM_IMU_H
#define __COM_IMU_H


#include "com_comfig.h"
#include "math.h"



void Common_IMU_GetYawOnly(GyroAccel_Struct  *gyroAccel,
                           EulerAngle_Struct *eulerAngle,
                           float              dt);

void Common_IMU_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt);

#endif
