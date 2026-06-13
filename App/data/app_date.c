#include "app_date.h"


/*对六轴传感器获取的数据进行滤波*/
void Date_MPU6050_get_filter(GyroAccel_Struct *gyroAccel)
{
	taskENTER_CRITICAL();    //进入临界区
    MPU6050_Read_Calibrate(gyroAccel);
    taskEXIT_CRITICAL();
	/*角速度滤波*/
	int16_t last_date[3]={0}; 
	gyroAccel->gyro.gyroX=Com_Filter_LowPass(gyroAccel->gyro.gyroX,last_date[0]);
	gyroAccel->gyro.gyroY=Com_Filter_LowPass(gyroAccel->gyro.gyroY,last_date[1]);
	gyroAccel->gyro.gyroZ=Com_Filter_LowPass(gyroAccel->gyro.gyroZ,last_date[2]);
	last_date[0]=gyroAccel->gyro.gyroX;
	last_date[1]=gyroAccel->gyro.gyroY;
	last_date[2]=gyroAccel->gyro.gyroZ;
	/*加速度滤波*/
	gyroAccel->accel.accelX=Common_Filter_KalmanFilter(&kfs[0],gyroAccel->accel.accelX);
	gyroAccel->accel.accelY=Common_Filter_KalmanFilter(&kfs[1],gyroAccel->accel.accelY);
	gyroAccel->accel.accelZ=Common_Filter_KalmanFilter(&kfs[2],gyroAccel->accel.accelZ);
}


/*获取六轴传感器的欧拉角-车子版*/
void Date_MPU6050_get_Yaw(GyroAccel_Struct  *gyroAccel,
                           EulerAngle_Struct *eulerAngle,
						   _falsh_savedate *gps_mpu_save,
                           float              dt)
{
	Common_IMU_GetYawOnly(gyroAccel,eulerAngle,dt);
//	Common_IMU_GetEulerAngle(gyroAccel,eulerAngle,dt);
	gps_mpu_save->falsh_mpu_date.YAW=eulerAngle->yaw;
}

uint8_t flag=0;
/*获取GPS平滑信息,并处理*/
void Date_GPS_get(_falsh_savedate *gps_mpu_save)
{
	flag+=1;
	check_gps();   //处理数据
	parseGpsBuffer();   //存储数据
	GPS_Filter(gps_mpu_save);   //四维滤波，并存储数据至flash结构体中
}












							   
							   
						   


