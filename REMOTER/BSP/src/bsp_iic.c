#include "bsp_iic.h"
//SDA PB12 SCL PB13

void iic_delay(void)
{
    __nop();__nop();__nop();__nop();__nop();__nop();
}

//初始化IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOB时钟

    //GPIOB8,B9初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    IIC_SCL=1;
    IIC_SDA=1;
}

//产生IIC起始信号
void IIC_Start(void)
{
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	iic_delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	 

//产生IIC停止信号
void IIC_Stop(void)
{
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay();
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	iic_delay();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC_SDA=1;
    iic_delay();	   
	IIC_SCL=1;
    iic_delay();	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;
	iic_delay();
	IIC_SCL=1;
	iic_delay();
	IIC_SCL=0;
}

//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	IIC_SDA=1;
	iic_delay();
	IIC_SCL=1;
	iic_delay();
	IIC_SCL=0;
}		

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay();   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		iic_delay(); 
		IIC_SCL=0;	
		iic_delay();
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        iic_delay();
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		iic_delay(); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 IIC_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 IIC_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
				else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
				len--;
				buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 IIC_Write_One_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 IIC_Read_One_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		    //读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}


//列出IIC总线上所有从机地址
void IIC_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC_Start();
    IIC_Send_Byte((i<<1)|0);
		res = IIC_Wait_Ack();          //等待应答
		if(res == 0)
			printf("IIC_ADDR = %#x\r\n",i);
		IIC_Stop();
	}printf("\r\n");
}

//读取IIC总线上某个器件的所有寄存器
void IIC_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}

