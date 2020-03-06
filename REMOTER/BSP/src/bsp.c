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
	NRF24L01_Init();
    
    //Joystick Init
    Joystick_DMA_Init();
    Joystick_ADC_Init();
    
    //Button Init
    Button_Init();

    //OLED Init
    OLED_Init();
}


