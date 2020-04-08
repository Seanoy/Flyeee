#include "task_config.h"

//������
TaskHandle_t StartTask_Handler;
TaskHandle_t Task_Handler_Sensor;
TaskHandle_t Task_Handler_Nrf;
TaskHandle_t Task_Handler_Stabilizer;

//����
//static xQueueHandle nrfDataQueue;



int main(void)
{ 	
    //��ʼ���ײ㺯��
	BSP_Init();
    /* The queue is created to hold a maximum of 5 values, each of which is
    large enough to hold a variable of type int32_t. */
    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}
 
//��ʼ����������
void start_task(void *pvParameters)
{
    //�����ٽ���
    taskENTER_CRITICAL();
    //��������������
    xTaskCreate((TaskFunction_t )sensor_task,         
                (const char*    )"sensor_task",       
                (uint16_t       )STK_SIZE_SENSOR, 
                (void*          )NULL,              
                (UBaseType_t    )TASK_PRIO_SENSOR,    
                (TaskHandle_t*  )&Task_Handler_Sensor);   
    //����nrfͨ������
    xTaskCreate((TaskFunction_t )nrf_task,     
                (const char*    )"nrf_task",   
                (uint16_t       )STK_SIZE_NRF, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_NRF,
                (TaskHandle_t*  )&Task_Handler_Nrf);
    //������̬����
    xTaskCreate((TaskFunction_t )stabilize_task,     
                (const char*    )"stabilize_task",   
                (uint16_t       )STK_SIZE_STABILIZER, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_STABILIZER,
                (TaskHandle_t*  )&Task_Handler_Stabilizer);      
    //ɾ����ʼ����
    vTaskDelete(StartTask_Handler);
    //�˳��ٽ���
    taskEXIT_CRITICAL();
}
