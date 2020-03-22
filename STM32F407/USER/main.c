#include "task_config.h"

//任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t Task_Handler1;
TaskHandle_t Task_Handler2;

//队列
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
    //初始化底层函数
	BSP_Init();
    /* The queue is created to hold a maximum of 5 values, each of which is
    large enough to hold a variable of type int32_t. */
    xQueue = xQueueCreate( 3, sizeof( struct nrf_rxdata* ) );//存储姿态结构体attitude_t
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
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )attitude_task,         
                (const char*    )"attitude_task",       
                (uint16_t       )STK_SIZE_1, 
                (void*          )NULL,              
                (UBaseType_t    )TASK_PRIO_1,    
                (TaskHandle_t*  )&Task_Handler1);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )nrf_task,     
                (const char*    )"nrf_task",   
                (uint16_t       )STK_SIZE_2, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_2,
                (TaskHandle_t*  )&Task_Handler2);        
   
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//获取姿态任务函数 
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
//        mpu6050_send_data(attitude.acc.x,attitude.acc.y, attitude.acc.z, attitude.gyro.x, attitude.gyro.y, attitude.gyro.z);//用自定义帧发送加速度和陀螺仪原始数据
//        usart1_report_imu(attitude.acc.x,attitude.acc.y, attitude.acc.z, attitude.gyro.x, attitude.gyro.y, attitude.gyro.z,(int)(attitude.Q_ANGLE.Pitch*100),(int)(attitude.Q_ANGLE.Roll*100),(int)(attitude.Q_ANGLE.Yaw*10));
        processSensordata();
        vTaskDelay(500);
    }
}

//nrf任务函数
void nrf_task(void *pvParameters)
{	    
//    BaseType_t xStatus;
    u8 rxbuf[32] = {0};
    while(1)
    {
        if(NRF24L01_RxPacket(rxbuf)==0)//接收成功
        {
            if(Handle_NRF_Data(rxbuf)==0)//数据正确
            {
                
                //根据xy坐标值操作
            }
        }
        LED1=!LED1;
        vTaskDelay(400);
    }
}
