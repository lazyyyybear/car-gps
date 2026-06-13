#include "app_save.h"

static uint32_t count=0;
static uint32_t page=0;


/*将存储GPS和陀螺仪数据的结构体数据写入FLAHS内*/
void W25Q128_Savedate(_falsh_savedate *gps_mpu_save)
{
	if(count==0&&page==0)
	{
		W25QXX_Erase_Sector(0);  //刚开始擦除
	}
	if(count>=128)     //一个扇区有16页，1页存8组信息，0~15，一个扇区存128组
	{
		page+=1;
		W25QXX_Erase_Sector(page);
		count=0;
	}
	uint8_t date[32]={0};
	memcpy(date,gps_mpu_save,sizeof(_falsh_savedate));
	static uint32_t current_addr=0;
	W25QXX_Page_Program(date,current_addr,sizeof(_falsh_savedate));
	current_addr+=GPS_DATE_LEN;    //地址后移32字节
	count++;
}

