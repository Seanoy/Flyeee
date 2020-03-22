#include "bsp.h"

void BSP_Init(void)
{
    //clock init
	STM32_Clock_Init(336,8,2,7);//设置时钟,168Mhz
    IWDG_FEED_TIMER_Init(8000-1, 8400-1);//feed dog per 800ms
    //watchdog is waitting to be fed
	IWDG_Init(4, 500);//1S后不喂狗就饿死T_T 4*2^4*500/32=1000ms
    //uart init
	UART_Init(84,115200);	//串口初始化为115200
    //leds init
	LED_Init();
    //mpu9250 iic init
    IIC_Init();     //初始化IIC总线
	IIC1_Slave_List();
	IIC2_Slave_List();

	//motors init
	MOTOR_Init();
    //nfr24l01 init
    Communication_Init();
    NRF24L01_Init();

}


