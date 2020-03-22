#include "bsp_ak8963.h"
#include "task_config.h"

u8 AK8963_Init(void)
{   
	u8 res=0;
	res=IIC1_Read_One_Byte(AK8963_ADDR,AK8963_WIA);    			//��ȡAK8963 ID 
    vTaskDelay(20);
    
	if(res==AK8963_ID)
	{        
        printf("AK8963 I2C Connection [OK].\r\n");
		IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL2,0X01);		//��λAK8963
        vTaskDelay(50);
		IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11);		//����AK8963Ϊ���β���
        return 0;
	}
	else 
    {
        printf("AK8963 I2C Connection [FAIL].\r\n");
        return 1;
    }
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Magnetometer(axis3f_t *mag_s)
{
    u8 buf[6],res;  
	res=IIC1_Read_NByte(AK8963_ADDR,AK8963_XOUT_L,6,buf);
	if(res==0)
	{
		mag_s->x=((u16)buf[1]<<8)|buf[0];  
		mag_s->y=((u16)buf[3]<<8)|buf[2];  
		mag_s->z=((u16)buf[5]<<8)|buf[4];
	} 	
    IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}
