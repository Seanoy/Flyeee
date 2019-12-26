#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"
#include <stdlib.h>
#include <string.h>
#define LED0 PCout(13)
#define LED1 PCout(14)

//���������ȼ�
#define START_TASK_PRIO     1
//�������ջ��С    
#define START_STK_SIZE      128  

//�������ȼ�1
#define LED0_TASK_PRIO      2
//�����ջ��С1
#define LED0_STK_SIZE       256  

//�������ȼ�2
#define LED1_TASK_PRIO      3
//�����ջ��С2
#define LED1_STK_SIZE       256

//������
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t LED0Task_Handler;
extern TaskHandle_t LED1Task_Handler;

//������
void start_task(void *pvParameters);
void led0_task(void *pvParameters);
void led1_task(void *pvParameters);


#endif
