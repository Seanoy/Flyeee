#include "bsp_ak8963.h"
#include "task_config.h"

u8 AK8963_Init(void)
{   
	u8 res=0;
	res=IIC1_Read_One_Byte(AK8963_ADDR,AK8963_WIA);    			//读取AK8963 ID 
    vTaskDelay(20);
    
	if(res==AK8963_ID)
	{        
        printf("AK8963 I2C Connection [OK].\r\n");
		IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL2,0X01);		//复位AK8963
        vTaskDelay(50);
		IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11);		//设置AK8963为单次测量
        return 0;
	}
	else 
    {
        printf("AK8963 I2C Connection [FAIL].\r\n");
        return 1;
    }
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
    IIC1_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;
}
