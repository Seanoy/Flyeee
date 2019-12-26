#include "bsp.h"


void BSP_Init(void)
{
//	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz 
	
	delay_init(168);
//	watchdogInit(WATCHDOG_RESET_MS);
    
	uart_init(84,115200);	//串口初始化为115200
	
	LED_Init();
	
	MPU9250_Init();
	
	IIC_Slave_List();
//	IIC_Slave_Register(0X68);
//	IIC_Slave_Register(0X0c);	

	MOTOR_TIM_Init();
}


