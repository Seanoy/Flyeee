#include "bsp_ak8963.h"



u8 AK8963_Init(void)
{   
	u8 res=0;
	res=IIC_Read_One_Byte(AK8963_ADDR,AK8963_WIA);    			//读取AK8963 ID  
	//		printf("res:%x\r\n",res);
	if(res==AK8963_ID)
	{
		IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL2,0X01);		//复位AK8963
//		delay_ms(50);
		IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11);		//设置AK8963为单次测量
        return 0;
	}
	else return 1;
}




//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Magnetometer(mag_struct *mag_s)
{
    u8 buf[6],res;  
	res=IIC_Read_NByte(AK8963_ADDR,AK8963_XOUT_L,6,buf);
	if(res==0)
	{
		mag_s->mx=((u16)buf[1]<<8)|buf[0];  
		mag_s->my=((u16)buf[3]<<8)|buf[2];  
		mag_s->mz=((u16)buf[5]<<8)|buf[4];
	} 	
    IIC_Write_One_Byte(AK8963_ADDR,AK8963_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;;
}









