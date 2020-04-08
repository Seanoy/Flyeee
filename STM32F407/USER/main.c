#include "task_config.h"

//任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t Task_Handler_Sensor;
TaskHandle_t Task_Handler_Nrf;
TaskHandle_t Task_Handler_Stabilizer;

//队列
//static xQueueHandle nrfDataQueue;



int main(void)
{ 	
    //初始化底层函数
	BSP_Init();
    /* The queue is created to hold a maximum of 5 values, each of which is
    large enough to hold a variable of type int32_t. */
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
    //进入临界区
    taskENTER_CRITICAL();
    //创建传感器任务
    xTaskCreate((TaskFunction_t )sensor_task,         
                (const char*    )"sensor_task",       
                (uint16_t       )STK_SIZE_SENSOR, 
                (void*          )NULL,              
                (UBaseType_t    )TASK_PRIO_SENSOR,    
                (TaskHandle_t*  )&Task_Handler_Sensor);   
    //创建nrf通信任务
    xTaskCreate((TaskFunction_t )nrf_task,     
                (const char*    )"nrf_task",   
                (uint16_t       )STK_SIZE_NRF, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_NRF,
                (TaskHandle_t*  )&Task_Handler_Nrf);
    //创建姿态任务
    xTaskCreate((TaskFunction_t )stabilize_task,     
                (const char*    )"stabilize_task",   
                (uint16_t       )STK_SIZE_STABILIZER, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_STABILIZER,
                (TaskHandle_t*  )&Task_Handler_Stabilizer);      
    //删除开始任务
    vTaskDelete(StartTask_Handler);
    //退出临界区
    taskEXIT_CRITICAL();
}
