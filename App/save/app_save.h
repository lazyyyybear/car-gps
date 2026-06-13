#ifndef __APP_SAVE_H
#define __APP_SAVE_H



#define GPS_DATE_LEN 32



#include "W25Q128.h"
#include "com_comfig.h"
#include <string.h>



void W25Q128_Savedate(_falsh_savedate *gps_mpu_save);

#endif
