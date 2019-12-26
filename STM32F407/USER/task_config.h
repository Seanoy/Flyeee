#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"
#include <stdlib.h>
#include <string.h>
#define LED0 PCout(13)
#define LED1 PCout(14)

//主任务优先级
#define START_TASK_PRIO     1
//主任务堆栈大小    
#define START_STK_SIZE      128  

//任务优先级1
#define LED0_TASK_PRIO      2
//任务堆栈大小1
#define LED0_STK_SIZE       256  

//任务优先级2
#define LED1_TASK_PRIO      3
//任务堆栈大小2
#define LED1_STK_SIZE       256

//任务句柄
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t LED0Task_Handler;
extern TaskHandle_t LED1Task_Handler;

//任务函数
void start_task(void *pvParameters);
void led0_task(void *pvParameters);
void led1_task(void *pvParameters);


#endif
