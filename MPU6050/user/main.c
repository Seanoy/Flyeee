#include "bsp.h"

void twinkle()
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	delay_ms(500);
}
	 
int main(){	
	short Accel[3];
	short Gyro [3];
	short Temp;
	
	BSP_init();
 

if (MPU6050ReadID() == 0)
	{	
    printf("��ⲻ��MPU6050��ͣ��");
	  while(1);
  }
  while(1)
  {
		MPU6050ReadAcc(Accel);			
		printf("\r\n���ٶȣ� %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
		MPU6050ReadGyro(Gyro);
		printf("�����ǣ� %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);
		
		MPU6050_ReturnTemp(&Temp);
		printf("�¶ȣ� %d",Temp);
		delay_ms(1000);
	}	

}
