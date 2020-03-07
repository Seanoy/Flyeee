#include "bsp.h"

void twinkle()
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
}
	 
int main(){		
//	u8 txbuf[33]="Can you go out on a date with me?";
    signed char aa[]={0x12, 0x34, 0x56, 0x78};
    signed char bb[]={0x11, 0x22, 0x33};

	BSP_init();
//	NRF24L01_CSN=0;
//    if(NRF24L01_Check())//检测不到24L01
//	{
//		printf("Device Error!\r\n");
//	}
//	else
//        printf("Device Check!\r\n");
	
//	TX_Mode();//接收模式
    Fill_Data(0x01,aa,&txdata);
    Fill_Data(0x02,bb,&txdata);

    while(1)	
    {	
        Handle_XY();
        
//	printf("hello\r\n");
        twinkle();
	
//	if(NRF24L01_TxPacket(txbuf) == TX_OK)//发送成功
//	{
//		printf("Send %s OK!\r\n",txbuf);
//	}
//    delay_ms(1000);			//间隔1s发送一次

    }
}
