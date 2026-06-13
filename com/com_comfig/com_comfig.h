#ifndef __COM_COMFIG_H
#define __COM_COMFIG_H

#include "stdint.h"
#include "stdio.h"

typedef struct
{
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	
}Gyro_Struct;

typedef struct
{
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;
	
}Accel_Struct;

typedef struct
{
	Gyro_Struct gyro;
	Accel_Struct accel;
}GyroAccel_Struct;

/*********************************/

/* 欧拉角 */
typedef struct
{
    float pitch;
    float roll;
    float yaw;

} EulerAngle_Struct;

/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;






/*********************************************************************/

//定义数组长度
#define GPS_Buffer_Length 80
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 




typedef struct SaveData 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char speed[10];         // 地面速度（单位：节 knots）
    char course[10];        // 航向角（单位：度）
	char isUsefull;		//定位信息是否有效
} _SaveData;

typedef struct FLASH_GPS_SAVE
{
	char UTC[12];       // 12    0~11
	float latitude;     // 4     12~15
	float longitude;    // 4     16~19
	float speed;       // 4      20~23
	float course;	   // 4      24~27
}_save_gps_flash;


typedef struct FLASH_MPU_SAVE
{
	float YAW;       //4     28~31
}_save_MPU_flash;


typedef struct FLASH_SaveDate
{
	_save_gps_flash flash_gps_date;
	_save_MPU_flash falsh_mpu_date;
}_falsh_savedate;




//extern _saveflash flash_gps_date;
//extern _save_MPU_flash falsh_mpu_date;  //用于存储陀螺仪的偏航角

extern GyroAccel_Struct gyroAccel;   //用于存储实时陀螺仪数据
extern EulerAngle_Struct eulerangle;
extern _falsh_savedate  gps_mpu_save;

#endif


