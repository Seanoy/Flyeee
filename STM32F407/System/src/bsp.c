#include "bsp.h"


void BSP_Init(void)
{
    //clock init
	Stm32_Clock_Init(336,8,2,7);//����ʱ��,168Mhz 
//	watchdogInit(WATCHDOG_RESET_MS);
    
    //uart init
	uart_init(84,115200);	//���ڳ�ʼ��Ϊ115200
    
    //leds init
	LED_Init();
	
    //mpu9250 iic init
    IIC_Init();     //��ʼ��IIC����
	IIC_Slave_List();
//	IIC_Slave_Register(0X68);
//	IIC_Slave_Register(0X0c);

	//motors init
	MOTOR_TIM_Init();
    
    //nfr24l01 init
    NRF24L01_Init();
    if(NRF24L01_Check())//��ⲻ��24L01
		printf("Device Error!\r\n");
    else
        printf("Device Init Success!\r\n");
	RX_Mode();//����ģʽ
    
}


