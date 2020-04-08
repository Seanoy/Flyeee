#ifndef __TASK_CONFIG
#define __TASK_CONFIG

#include "bsp.h"

#define LED0 PCout(13)
#define LED1 PCout(14)

/************************            ������            ****************************/
//���������ȼ�
#define START_TASK_PRIO         1
//�������ջ��С    
#define START_STK_SIZE          128

//���������ݻ�ȡ�������ȼ�
#define TASK_PRIO_SENSOR        2
//���������ݻ�ȡ�����ջ��С
#define STK_SIZE_SENSOR         128

//����ͨ���������ȼ�
#define TASK_PRIO_NRF           3
//����ͨ�������ջ��С2
#define STK_SIZE_NRF            128

//��̬�������ȼ�
#define TASK_PRIO_STABILIZER    4
//��̬�����ջ��С2
#define STK_SIZE_STABILIZER     256

/************************            �����ʱ������            ****************************/

/************************             ��Ϣ���ж���            ****************************/


//������
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t Task_Handler_Sensor;
extern TaskHandle_t Task_Handler_nrf;
extern TaskHandle_t Task_Handler_Stabilizer;

//������
void start_task(void *pvParameters);
void sensor_task(void *pvParameters);
void nrf_task(void *pvParameters);
void stabilizer_task(void *pvParameters);


#endif
