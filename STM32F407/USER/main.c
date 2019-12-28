#include "task_config.h"

gyro_struct gyro;
acc_struct acc;
mag_struct mag;

//������
TaskHandle_t StartTask_Handler;
TaskHandle_t LED0Task_Handler;
TaskHandle_t LED1Task_Handler;

u16 temperature;//MPU9250��ȡ���¶�ֵ

//int main(void)
//{ 	
//    //��ʼ���ײ㺯��
//	BSP_Init();
//    //������ʼ����
//    xTaskCreate((TaskFunction_t )start_task,            //������
//                (const char*    )"start_task",          //��������
//                (uint16_t       )START_STK_SIZE,        //�����ջ��С
//                (void*          )NULL,                  //���ݸ��������Ĳ���
//                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
//                (TaskHandle_t*  )&StartTask_Handler);   //������              
//    vTaskStartScheduler();          //�����������
//}
 
//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //����LED0����
    xTaskCreate((TaskFunction_t )led0_task,         
                (const char*    )"led0_task",       
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,              
                (UBaseType_t    )LED0_TASK_PRIO,    
                (TaskHandle_t*  )&LED0Task_Handler);   
    //����LED1����
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LED1_TASK_PRIO,
                (TaskHandle_t*  )&LED1Task_Handler);        
   
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//LED0������ 
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

//LED1������
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
