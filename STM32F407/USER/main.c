#include "task_config.h"

//������
TaskHandle_t StartTask_Handler;
TaskHandle_t Task_Handler1;
TaskHandle_t Task_Handler2;

//����
QueueHandle_t xQueue;

typedef struct 
{
    axis3f_t gyro;
    axis3f_t acc;
    axis3f_t mag;
    S_FLOAT_ANGLE Q_ANGLE;
}attidude_t;

attidude_t attitude;

int main(void)
{ 	
    //��ʼ���ײ㺯��
	BSP_Init();
    /* The queue is created to hold a maximum of 5 values, each of which is
    large enough to hold a variable of type int32_t. */
    xQueue = xQueueCreate( 3, sizeof( struct nrf_rxdata* ) );//�洢��̬�ṹ��attitude_t
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
    taskENTER_CRITICAL();           //�����ٽ���
    //����LED0����
    xTaskCreate((TaskFunction_t )attitude_task,         
                (const char*    )"attitude_task",       
                (uint16_t       )STK_SIZE_1, 
                (void*          )NULL,              
                (UBaseType_t    )TASK_PRIO_1,    
                (TaskHandle_t*  )&Task_Handler1);   
    //����LED1����
    xTaskCreate((TaskFunction_t )nrf_task,     
                (const char*    )"nrf_task",   
                (uint16_t       )STK_SIZE_2, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_2,
                (TaskHandle_t*  )&Task_Handler2);        
   
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//��ȡ��̬������ 
void attitude_task(void *pvParameters)
{
    //imu init
    MPU9250_Init();
    //magnitude sensor init
    AK8963_Init();
    //barometric sensor init
    BMP_Init();
    //filter init
    Filter_Init();
    
    while(1)
    {
        LED0=~LED0;
//        MPU_Get_Gyroscope(&attitude.gyro);
//        MPU_Get_Accelerometer(&attitude.acc);
//        MPU_Get_Magnetometer(&attitude.mag);
//        temperature = MPU_Get_Temperature();
//        IMUupdate(attitude.gyro.x, attitude.gyro.y, attitude.gyro.z, attitude.acc.x, attitude.acc.y, attitude.acc.z, &attitude.Q_ANGLE);
//        mpu6050_send_data(attitude.acc.x,attitude.acc.y, attitude.acc.z, attitude.gyro.x, attitude.gyro.y, attitude.gyro.z);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
//        usart1_report_imu(attitude.acc.x,attitude.acc.y, attitude.acc.z, attitude.gyro.x, attitude.gyro.y, attitude.gyro.z,(int)(attitude.Q_ANGLE.Pitch*100),(int)(attitude.Q_ANGLE.Roll*100),(int)(attitude.Q_ANGLE.Yaw*10));
        processSensordata();
        vTaskDelay(500);
    }
}

//nrf������
void nrf_task(void *pvParameters)
{	    
//    BaseType_t xStatus;
    u8 rxbuf[32] = {0};
    while(1)
    {
        if(NRF24L01_RxPacket(rxbuf)==0)//���ճɹ�
        {
            if(Handle_NRF_Data(rxbuf)==0)//������ȷ
            {
                
                //����xy����ֵ����
            }
        }
        LED1=!LED1;
        vTaskDelay(400);
    }
}
