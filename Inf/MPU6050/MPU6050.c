#include "MPU6050.h"

#define abs(x) ((x) >= 0 ? (x) : -(x))

GyroAccel_Struct offsetgyroaccel;   //存储平稳状态下的陀螺仪数据

void MPU6050_Write(uint8_t reg,uint8_t byte)  //寄存器地址reg，要发送数据的指针byte
{
	HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR_W,reg,I2C_MEMADD_SIZE_8BIT,&byte,1,2000);
}


void MPU6050_Read(uint8_t reg,uint8_t *date,uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDR_R,reg,I2C_MEMADD_SIZE_8BIT,date,len,2000);
}

void MPU6050_Calibrate(void);

void MPU6050_Init(void)
{
	//复位+休眠+唤醒
	MPU6050_Write(MPU_PWR_MGMT1_REG, 1 << 7);  //电源管理寄存器第七位置1，复位所有寄存器
	HAL_Delay(100);
	MPU6050_Write(MPU_PWR_MGMT1_REG,0);   
	//设置角速度量程为 +-2000°
	MPU6050_Write(MPU_GYRO_CFG_REG, 3 << 3);   //1B 寄存器[4:3]位为11，手册可以得出为±2000°
	//设置加速度量程为+2g
	MPU6050_Write(MPU_ACCEL_CFG_REG, 0);    
	//关闭中断、关闭 第2 IIC 接口、禁止 FIFO
	MPU6050_Write(MPU_INT_EN_REG, 0);
	MPU6050_Write(MPU_USER_CTRL_REG, 0);
	MPU6050_Write(MPU_FIFO_EN_REG, 0);
	
	/*设置采样频率与低通滤波器，低通滤波器开启后，采样最高频率从8K->1K
	 采样频率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV)
     500Hz = 1000Hz / (1 + x)
     x = 1  
	
     where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), 
	and 1kHz when the DLPF is enabled (see Register 26).	*/
	MPU6050_Write(MPU_SAMPLE_RATE_REG, 1);
	MPU6050_Write(MPU_CFG_REG, 1 << 0);
	
	//配置系统时钟源
	MPU6050_Write(MPU_PWR_MGMT1_REG, 1);   //8MHZ
	//使能角加速度和加速度传感器
	MPU6050_Write(MPU_PWR_MGMT2_REG, 0);   //唤醒所有传感器，唤醒频率为1.25HZ
	
	//陀螺仪校准
	MPU6050_Calibrate();
	
}
/*  读取角速度  */
void MPU6050_ReadGyro(Gyro_Struct *gyro)    
{
	uint8_t data[6];
	MPU6050_Read(MPU_GYRO_XOUTH_REG,data, 6);
	
	gyro->gyroX = (int16_t)((data[0] << 8) | data[1]);   //寄存器数据为16位，高八位为data[0],低8位为data[1]
    gyro->gyroY = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroZ = (int16_t)((data[4] << 8) | data[5]);
	
}

/*  读取加速度  */
void MPU6050_ReadAccel(Accel_Struct *accel)
{
	uint8_t data[6];
	MPU6050_Read(MPU_ACCEL_XOUTH_REG, data, 6);
	
	accel->accelX = (int16_t)((data[0] << 8) | data[1]);
    accel->accelY = (int16_t)((data[2] << 8) | data[3]);
    accel->accelZ = (int16_t)((data[4] << 8) | data[5]);
	
}

/*  读取角速度和加速度  */
void MPU6050_ReadGyroAccel(GyroAccel_Struct *gyroAccel)
{
	MPU6050_ReadGyro(&gyroAccel->gyro);   //等价于&（gyroAccel->gyro）->优先级高于&
	MPU6050_ReadAccel(&gyroAccel->accel);
}


/* 陀螺仪校准，并获取稳定状态下的数据 */
/*校准时，要测得加速度与角速度在平稳状态下的偏离值（注意平稳状态下Z轴加速度为1g，即16384），其他为0  */
void MPU6050_Calibrate(void)    
{
	uint8_t   cnt = 30;
    GyroAccel_Struct current;
    GyroAccel_Struct last;
	while(cnt)
    {
		MPU6050_ReadGyroAccel(&current);
        if(abs(current.gyro.gyroX - last.gyro.gyroX) <= 10 &&
           abs(current.gyro.gyroY - last.gyro.gyroY) <= 10 &&
           abs(current.gyro.gyroZ - last.gyro.gyroZ) <= 10)
        {
            cnt--;
        }

        last = current; 
        HAL_Delay(3);
    }
	int32_t sumbuff[6]={0};
	uint16_t sumcount=255;
	uint16_t i;    //int8_t 为 -128~127，如果i为int8_t则永远无法大于sumcount
	for(i=0;i<sumcount;i++)        
	{
		MPU6050_ReadGyroAccel(&gyroAccel);
		sumbuff[0]+=gyroAccel.gyro.gyroX-0;
		sumbuff[1]+=gyroAccel.gyro.gyroY-0;
		sumbuff[2]+=gyroAccel.gyro.gyroZ-0;
		sumbuff[3]+=gyroAccel.accel.accelX-0;
		sumbuff[4]+=gyroAccel.accel.accelY-0;
		sumbuff[5]+=gyroAccel.accel.accelZ-16383;
		
		HAL_Delay(3);
	}
	offsetgyroaccel.gyro.gyroX=sumbuff[0]/sumcount;
	offsetgyroaccel.gyro.gyroY=sumbuff[1]/sumcount;
	offsetgyroaccel.gyro.gyroZ=sumbuff[2]/sumcount;
	offsetgyroaccel.accel.accelX=sumbuff[3]/sumcount;
	offsetgyroaccel.accel.accelY=sumbuff[4]/sumcount;
	offsetgyroaccel.accel.accelZ=sumbuff[5]/sumcount;
	
}

/* 获取陀螺仪偏离平稳状态下的数据，平稳状态下亦有偏移，所以减去offsetgyroaccel存储的值 */
void MPU6050_Read_Calibrate(GyroAccel_Struct *gyroAccel)   
{
	MPU6050_ReadGyroAccel(gyroAccel);
	
	gyroAccel->gyro.gyroX-=offsetgyroaccel.gyro.gyroX;
	gyroAccel->gyro.gyroY-=offsetgyroaccel.gyro.gyroY;
	gyroAccel->gyro.gyroZ-=offsetgyroaccel.gyro.gyroZ;
	gyroAccel->accel.accelX-=offsetgyroaccel.accel.accelX;
	gyroAccel->accel.accelY-=offsetgyroaccel.accel.accelY;
	gyroAccel->accel.accelZ-=offsetgyroaccel.accel.accelZ;
}





