#include "GPS.h"

unsigned char GPSRX_BUFF[BUFFER_SIZE]={0};
unsigned char GPSRX_LEN = 0;uint8_t rx1;
_SaveData Save_Data;
uint16_t Len=0;

BaseType_t xHigherPriorityTaskWoken;

char gps_gnrmc_data[] = "$GNRMC,090515.000,A,3110.34110,N,12122.03227,E,0.00,94.34,171122,,,A*46\r\n";  //模拟测试数据
void USART1_IRQHandler(void)
{
	
    // 先检查空闲中断
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
		uint32_t tmp = huart1.Instance->DR;  
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

		
		GPSRX_LEN = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        // 停止DMA
//      	HAL_UART_DMAStop(&huart1);
//		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
	}
}


void check_gps(void)
{
        // 检查数据是否有效
        if(GPSRX_LEN > 0)
        {
			HAL_UART_DMAStop(&huart1);
			for(uint16_t i=0;i<300;i++)
			{
				// 检查是否是GPRMC语句
            if(GPSRX_BUFF[i] == '$' && strncmp((char*)&GPSRX_BUFF[i+1], "GNRMC", 4) == 0)
            {
				for(uint16_t j=i;j<400;j++)
				{
                // 拷贝数据到安全缓冲区
					if(GPSRX_BUFF[j]=='\r')
					{
						memcpy(Save_Data.GPS_Buffer, (char*)(GPSRX_BUFF+i), j-i);
                        Save_Data.GPS_Buffer[j-i] = '\0';
//						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
                        Save_Data.isGetData = 1;
						break;
					}
				}
			if(Save_Data.isGetData==1)
				break;
            }
			}
			GPSRX_LEN = 0;
            memset((void*)GPSRX_BUFF, 0, GPSRX_LEN);
            
        }
        memcpy(Save_Data.GPS_Buffer,gps_gnrmc_data,sizeof(gps_gnrmc_data));  //模拟测试
        // 重启DMA
        HAL_UART_Receive_DMA(&huart1, GPSRX_BUFF, BUFFER_SIZE);
//		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}






void parseGpsBuffer(void)
{
        char *subString;
        char *subStringNext;
        char i = 0;
        if (Save_Data.isGetData)
        {
                Save_Data.isGetData = 0;
//               printf("**************\r\n");
//               printf("%s\r\n",Save_Data.GPS_Buffer);

                for (i = 0 ; i <= 8 ; i++)
                {
                        if (i == 0)
                        {
                           if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
                                                           printf("1\r\n");

                        }
                        else
                        {
                                subString++;
                                if ((subStringNext = strstr(subString, ",")) != NULL)
                                {
                                        char usefullBuffer[2];
                                        switch(i)
                                        {
                                                case 1:
											    memcpy(Save_Data.UTCTime, subString, subStringNext - subString);
												Save_Data.UTCTime[subStringNext - subString]='\0';
											    break;        //获取UTC时间
                                                case 2:memcpy(usefullBuffer, subString, subStringNext - subString);
											    break;        //获取UTC时间
                                                case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);
												Save_Data.latitude[subStringNext - subString]='\0';
											    break;        //获取纬度信息
                                                case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);
											    break;        //获取N/S
                                                case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);
												Save_Data.longitude[subStringNext - subString]='\0';
											    break;        //获取经度信息
                                                case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);
											    break;        //获取E/W
												case 7:memcpy(Save_Data.speed, subString, subStringNext - subString);
												Save_Data.speed[subStringNext - subString]='\0';
											    break;        //获取地面速度
												case 8:memcpy(Save_Data.course, subString, subStringNext - subString);
												Save_Data.course[subStringNext - subString]='\0';
											    break;        //获取航向角
                                                default:
											    break;
                                        }
                                        subString = subStringNext;
                                        Save_Data.isParseData = 1;
                                        if(usefullBuffer[0] == 'A')
                                                Save_Data.isUsefull = 1;
                                        else if(usefullBuffer[0] == 'V')
                                                Save_Data.isUsefull = 0;
                                }
                                else
                                {
//                                                                  printf("2\r\n");
                                }
                        }
                }
        }
}



/*将经纬度转换为以度为单位的浮点数*/
float convertDegMinToDec(float degMin)
{
    int degree = (int)(degMin / 100);   // 提取度
    float minute = degMin - degree * 100; // 提取分(包含小数）
    return degree + minute / 60.0f;     // 转十进制度
}




void GPS_Filter(_falsh_savedate *gps_mpu_save)
{
	if(Save_Data.isParseData && Save_Data.isUsefull)  // GPS定位有效才滤波
        {
            // 1. 【GPS提取的字符串 → 浮点数】
            float lat_degmin = atof(Save_Data.latitude);   // 原始纬度(度分)
            float lon_degmin = atof(Save_Data.longitude);  // 原始经度(度分)
            float speed_knot = atof(Save_Data.speed);      // 原始速度(节)
            float heading = atof(Save_Data.course);        // 原始航向(度)

            // 2. 单位转换
            float lat = convertDegMinToDec(lat_degmin);    // 转十进制度
            float lon = convertDegMinToDec(lon_degmin);
            float speed_kmh = speed_knot * 1.852f;         // 转 km/h

            // 3. 【喂给卡尔曼滤波】dt=1.0f (GPS默认1秒更新1次)
            KalmanGPS_Update(lat, lon, speed_kmh, heading, 1.0f);

            // 4. 【取出滤波后的平滑数据】→ 这就是最终可用的值！
            float lat_filter  = kf.X[0];  // 平滑纬度
            float lon_filter  = kf.X[1];  // 平滑经度
            float speed_filter= kf.X[2];  // 平滑速度
            float head_filter = kf.X[3];  // 平滑航向
			
			strcpy(gps_mpu_save->flash_gps_date.UTC,Save_Data.UTCTime);
			gps_mpu_save->flash_gps_date.latitude=lat_filter;
			gps_mpu_save->flash_gps_date.longitude=lon_filter;
			gps_mpu_save->flash_gps_date.speed=speed_filter;
			gps_mpu_save->flash_gps_date.course=head_filter;
        }
}







/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;             /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/******************************************************************************************/

