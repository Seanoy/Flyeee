#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"

#define LED0 PCout(13)
#define LED1 PCout(14)

/************************            任务定义            ****************************/
//主任务优先级
#define START_TASK_PRIO     1
//主任务堆栈大小    
#define START_STK_SIZE      128

//任务优先级1
#define TASK_PRIO_1         2
//任务堆栈大小1
#define STK_SIZE_1          256

//任务优先级2
#define TASK_PRIO_2         3
//任务堆栈大小2
#define STK_SIZE_2          256

/************************            软件定时器定义            ****************************/

/************************             消息队列定义            ****************************/


//任务句柄
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t Task_Handler_1;
extern TaskHandle_t Task_Handler_2;

//任务函数
void start_task(void *pvParameters);
void get_attitude_task(void *pvParameters);
void led1_task(void *pvParameters);


#endif
