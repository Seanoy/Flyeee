#include "bsp.h"


void BSP_Init(void)
{
    //clock init
	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz 
//	watchdogInit(WATCHDOG_RESET_MS);
    
    //uart init
	uart_init(84,115200);	//串口初始化为115200
    
    //leds init
	LED_Init();
	
    //mpu9250 iic init
    IIC_Init();     //初始化IIC总线
	IIC_Slave_List();
//	IIC_Slave_Register(0X68);
//	IIC_Slave_Register(0X0c);

	//motors init
	MOTOR_TIM_Init();
    
    //nfr24l01 init
    NRF24L01_Init();
    if(NRF24L01_Check())//检测不到24L01
		printf("Device Error!\r\n");
    else
        printf("Device Init Success!\r\n");
	RX_Mode();//接收模式
    
}


