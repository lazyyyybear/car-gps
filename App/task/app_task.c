#include "app_task.h"


//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define GPS_TASK_PRIO		4
//任务堆栈大小	
#define GPS_STK_SIZE 		128 
//任务句柄
TaskHandle_t GPSTask_Handler;
//任务函数
void gps_task(void *pvParameters);

//任务优先级
#define MPU_TASK_PRIO		5
//任务堆栈大小	
#define MPU_STK_SIZE 		128 
//任务句柄
TaskHandle_t MPUTask_Handler;
//任务函数
void mpu_task(void *pvParameters);

//任务优先级
#define SAVE_TASK_PRIO		3
//任务堆栈大小	
#define SAVE_STK_SIZE 		128 
//任务句柄
TaskHandle_t SAVETask_Handler;
//任务函数
void save_task(void *pvParameters);

SemaphoreHandle_t GPS_Handle;
BaseType_t err = pdFALSE;

QueueHandle_t Date_Queue;
#define QUEUE_LEN 5   //队列中的消息长度
#define QUEUE_SIZE 32   //队列中的消息大小



/*******************************************************************************
* 函 数 名         : main
* 函数功能		   : 主函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/

void App_Task_FreeRTOSStart(void)
{
	MPU6050_Init();    //MPU6050初始化
	
//	HAL_UART_Receive_DMA(&huart1, GPSRX_BUFF, BUFFER_SIZE);
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //开启串口空闲中断
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
	

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	Date_Queue = xQueueCreate((UBaseType_t ) QUEUE_LEN,/* 消息队列的长度 */
                            (UBaseType_t ) QUEUE_SIZE);/* 消息的大小 */
      		
	//创建DATE任务
    xTaskCreate((TaskFunction_t )gps_task,     
                (const char*    )"date_task",   
                (uint16_t       )GPS_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )GPS_TASK_PRIO,
                (TaskHandle_t*  )&GPSTask_Handler);
	//创建DATE任务
    xTaskCreate((TaskFunction_t )mpu_task,     
                (const char*    )"date_task",   
                (uint16_t       )MPU_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )MPU_TASK_PRIO,
                (TaskHandle_t*  )&MPUTask_Handler);				
				
	
    //创建SAVE任务
    xTaskCreate((TaskFunction_t )save_task,     
                (const char*    )"save_task",   
                (uint16_t       )SAVE_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )SAVE_TASK_PRIO,
                (TaskHandle_t*  )&SAVETask_Handler);				
				
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
} 

//GPS任务函数
void gps_task(void *pvParameters)
{
	uint32_t ticktime=xTaskGetTickCount();
    while(1)
    {
		Date_GPS_get(&gps_mpu_save);
		vTaskDelayUntil(&ticktime,1000);
    }
}

//MPU任务函数
void mpu_task(void *pvParameters)
{
	uint32_t ticktime=xTaskGetTickCount();
    while(1)
    {
		Date_MPU6050_get_filter(&gyroAccel);
		Date_MPU6050_get_Yaw(&gyroAccel,&eulerangle,&gps_mpu_save,0.02);
		xQueueSend(Date_Queue, &gps_mpu_save,0);
		vTaskDelayUntil(&ticktime,20);	
    }
}

//SAVE任务函数
void save_task(void *pvParameters)
{
	_falsh_savedate res;
    while(1)
    {
		xQueueReceive(Date_Queue, &res,100);
		W25Q128_Savedate(&res);
    }
}







