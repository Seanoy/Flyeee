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
    Communication_Init();
	NRF24L01_Init();
    
    if(NRF24L01_Check())
		printf("Device Error!\r\n");//��ⲻ��24L01
	else
        printf("Device Pass!\r\n");//��⵽24L01
	TX_Mode();//����ģʽ ң����

    //Joystick Init
    Joystick_DMA_Init();
    Joystick_ADC_Init();
    
    //Button Init
    Button_Init();
    
    //IIC Init
    IIC_Init();
    delay_ms(100);
    
    //OLED Init
    OLED_Init();
}


