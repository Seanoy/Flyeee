#include "bsp_ak8963.h"

u8 AK8963_Init(void)
{   
	u8 res=0;
	res=IIC_Read_One_Byte(AK8963_ADDR,AK8963_WIA);    			//��ȡAK8963 ID  
	//		printf("res:%x\r\n",res);
	if(res==AK8963_ID)
	{
		IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL2,0X01);		//��λAK8963
//		delay_ms(50);
		IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11);		//����AK8963Ϊ���β���
        return 0;
	}
	else return 1;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Magnetometer(axis3f_t *mag_s)
{
    u8 buf[6],res;  
	res=IIC_Read_NByte(AK8963_ADDR,AK8963_XOUT_L,6,buf);
	if(res==0)
	{
		mag_s->x=((u16)buf[1]<<8)|buf[0];  
		mag_s->y=((u16)buf[3]<<8)|buf[2];  
		mag_s->z=((u16)buf[5]<<8)|buf[4];
	} 	
    IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}
