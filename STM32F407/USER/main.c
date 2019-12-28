#include "task_config.h"

gyro_struct gyro;
acc_struct acc;
mag_struct mag;

//任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t LED0Task_Handler;
TaskHandle_t LED1Task_Handler;

u16 temperature;//MPU9250读取的温度值

//int main(void)
//{ 	
//    //初始化底层函数
//	BSP_Init();
//    //创建开始任务
//    xTaskCreate((TaskFunction_t )start_task,            //任务函数
//                (const char*    )"start_task",          //任务名称
//                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
//                (void*          )NULL,                  //传递给任务函数的参数
//                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
//                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
//    vTaskStartScheduler();          //开启任务调度
//}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,         
                (const char*    )"led0_task",       
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,              
                (UBaseType_t    )LED0_TASK_PRIO,    
                (TaskHandle_t*  )&LED0Task_Handler);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LED1_TASK_PRIO,
                (TaskHandle_t*  )&LED1Task_Handler);        
   
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
    while(1)
    {
        LED0=~LED0;
        MPU_Get_Gyroscope(&gyro);
		MPU_Get_Accelerometer(&acc);
		MPU_Get_Magnetometer(&mag);
		temperature = MPU_Get_Temperature();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//LED1任务函数
void led1_task(void *pvParameters)
{	
    while(1)
    {
        LED1=0;
        vTaskDelay(pdMS_TO_TICKS(600));
        LED1=1;
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}
