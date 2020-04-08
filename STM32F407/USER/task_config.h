#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"

#define LED0 PCout(13)
#define LED1 PCout(14)

/************************            任务定义            ****************************/
//主任务优先级
#define START_TASK_PRIO         1
//主任务堆栈大小    
#define START_STK_SIZE          128

//传感器数据获取任务优先级
#define TASK_PRIO_SENSOR        2
//传感器数据获取任务堆栈大小
#define STK_SIZE_SENSOR         128

//无线通信任务优先级
#define TASK_PRIO_NRF           3
//无线通信任务堆栈大小2
#define STK_SIZE_NRF            128

//姿态任务优先级
#define TASK_PRIO_STABILIZER    4
//姿态任务堆栈大小2
#define STK_SIZE_STABILIZER     256

/************************            软件定时器定义            ****************************/

/************************             消息队列定义            ****************************/


//任务句柄
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t Task_Handler_Sensor;
extern TaskHandle_t Task_Handler_nrf;
extern TaskHandle_t Task_Handler_Stabilizer;

//任务函数
void start_task(void *pvParameters);
void sensor_task(void *pvParameters);
void nrf_task(void *pvParameters);
void stabilizer_task(void *pvParameters);


#endif
