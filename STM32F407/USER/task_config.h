#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"

#define LED0 PCout(13)
#define LED1 PCout(14)

/************************            ������            ****************************/
//���������ȼ�
#define START_TASK_PRIO     1
//�������ջ��С    
#define START_STK_SIZE      128

//�������ȼ�1
#define TASK_PRIO_1         2
//�����ջ��С1
#define STK_SIZE_1          256

//�������ȼ�2
#define TASK_PRIO_2         3
//�����ջ��С2
#define STK_SIZE_2          256

/************************            �����ʱ������            ****************************/

/************************             ��Ϣ���ж���            ****************************/


//������
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t Task_Handler_1;
extern TaskHandle_t Task_Handler_2;

//������
void start_task(void *pvParameters);
void get_attitude_task(void *pvParameters);
void led1_task(void *pvParameters);


#endif
