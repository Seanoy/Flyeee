#include "bsp.h"

void USART_Config(void);

void BSP_init(void)
{	
    //UART Init
	USART_Config();
    
    //Led Init
	LED_GPIO_Config();
    
    //Clock Init
	delay_init();
    
    //Communication Init
    Comm_Init();
	NRF24L01_Init();
//    NRF24L01_CSN=0;
    if(NRF24L01_Check())
		printf("Device Error!\r\n");//检测不到24L01
	else
        printf("Device Pass!\r\n");
	TX_Mode();//发送模式

    //Joystick Init
    Joystick_DMA_Init();
    Joystick_ADC_Init();
    
    //Button Init
    Button_Init();

    //OLED Init
    OLED_Init();
    

}


