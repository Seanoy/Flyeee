#include "bsp.h"

void BSP_Init(void)
{
    //clock init
	STM32_Clock_Init(336,8,2,7);//����ʱ��,168Mhz
    IWDG_FEED_TIMER_Init(8000-1, 8400-1);//feed dog per 800ms
    //watchdog is waitting to be fed
	IWDG_Init(4, 500);//1S��ι���Ͷ���T_T 4*2^4*500/32=1000ms
    //uart init
	UART_Init(84,115200);	//���ڳ�ʼ��Ϊ115200
    //leds init
	LED_Init();
    //mpu9250 iic init
    IIC_Init();     //��ʼ��IIC����
//	IIC_Slave_List();
    //magnitude sensor init
    AK8963_Init();
    //filter init
    Filter_Init();
	//motors init
	MOTOR_Init();
    //nfr24l01 init
    Communication_Init();
    NRF24L01_Init();
	NRF24L01_CSN=0; //SPI chip select		  		 		  
//    if(NRF24L01_Check())//cannot check 24L01
//		printf("Device Error!\r\n");
//    else
//        printf("Device Init Success!\r\n");
	RX_Mode();//receive mode
    
}


