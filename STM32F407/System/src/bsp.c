#include "bsp.h"

void BSP_Init(void)
{
    //clock init
	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz
    Watchdog_TIMER_Init(8000-1, 8400-1);//feed dog per 800ms
    //watchdog is waitting to be fed
	IWDG_Init(4, 500);//1S后不喂狗就饿死T_T 4*2^4*500/32=1000ms
    //uart init
	uart_init(84,115200);	//串口初始化为115200
    //leds init
	LED_Init();
    //mpu9250 iic init
    IIC_Init();     //初始化IIC总线
	IIC_Slave_List();
    //read all registers of the device by specific address
//	IIC_Slave_Register(0X68);
//	IIC_Slave_Register(0X0c);
	//motors init
	MOTOR_TIM_Init();
    //nfr24l01 init
    NRF24L01_Init();
	NRF24L01_CSN=0; //SPI chip select		  		 		  
    if(NRF24L01_Check())//cannot check 24L01
		printf("Device Error!\r\n");
    else
        printf("Device Init Success!\r\n");
	RX_Mode();//receive mode
    
}


