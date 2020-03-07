#include "bsp.h"

void twinkle()
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
}

int main(){
	BSP_init();
    
    while(1)
    {
        Handle_Transform();//transform joystick data
        OLED_Show_XY();//OLED display joystick xy axis value
        Fill_Data(0x01, coordinate, &txdata);//fill data to struct data_frame
//        NRF24L01_Tx_Data((u8 *)&txdata);
        
        twinkle();
    }
}
