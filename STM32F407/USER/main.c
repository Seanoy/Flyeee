#include "task_config.h"
#include "IMU.h"

gyro_struct gyro;
acc_struct acc;
mag_struct mag;

//任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t Task_Handler1;
TaskHandle_t Task_Handler2;

u16 temperature;//MPU9250读取的温度值

int main(void)
{ 	
    //初始化底层函数
	BSP_Init();
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
    xTaskCreate((TaskFunction_t )get_attitude_task,         
                (const char*    )"attitude_task",       
                (uint16_t       )STK_SIZE_1, 
                (void*          )NULL,              
                (UBaseType_t    )TASK_PRIO_1,    
                (TaskHandle_t*  )&Task_Handler1);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )STK_SIZE_2, 
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_2,
                (TaskHandle_t*  )&Task_Handler2);        
   
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//获取姿态任务函数 
void get_attitude_task(void *pvParameters)
{
    MPU9250_Init();
    while(1)
    {
        LED0=~LED0;
        MPU_Get_Gyroscope(&gyro);
		MPU_Get_Accelerometer(&acc);
		MPU_Get_Magnetometer(&mag);
		temperature = MPU_Get_Temperature();
        printf("gyro: %d, %d, %d\r\n",gyro.gx, gyro.gy, gyro.gz);
        printf("acc: %d, %d, %d\r\n",acc.ax, acc.ay, acc.az);
        printf("mag: %d, %d, %d\r\n",mag.mx, mag.my, mag.mz);
        printf("temperature:%d\r\n", temperature);
        IMUupdate(gyro.gx, gyro.gy, gyro.gz,acc.ax, acc.ay, acc.az);
        printf("pitch:%f\r\nroll:%f\r\nyaw:%f\r\n\r\n", Q_ANGLE.Pitch, Q_ANGLE.Roll, Q_ANGLE.Yaw);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//LED1任务函数
void led1_task(void *pvParameters)
{	
    u8 rxbuf[33] = {0};
    while(1)
    {
//        if(NRF24L01_RxPacket(rxbuf)==0)//接收成功
//        {
//            rxbuf[32]='\0';
//            printf("%s",rxbuf);
//        }
        LED1=!LED1;
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}
