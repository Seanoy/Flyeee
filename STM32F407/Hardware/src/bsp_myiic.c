#include "bsp_myiic.h"
#include <stdio.h>
#define TRY_TIME        50

void delay1(void)//mpu6500延时
{
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();
}

void delay3(void)//ak8963延时 延时要足够长 ak8963才能读取到数据 因为STM32F407主频为168MHz 一个nop时间为1/168MHz
{
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}

void delay2(void)//BMP180延时
{
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}
//初始化IIC
void IIC_Init(void)
{			
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟

    //GPIOB8,B9初始化设置     MPU6500部分
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    IIC_SCL1=1;
    IIC_SDA1=1;
    
    //PE2-SCL PE3-SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
    IIC_SCL2=1;
    IIC_SDA2=1;
}

//产生IIC起始信号
static void IIC1_Start(void)
{
	SDA_OUT1();     //sda线输出
	IIC_SDA1=1;	  	  
	IIC_SCL1=1;
	delay1();
 	IIC_SDA1=0;//START:when CLK is high,DATA change form high to low 
	delay1();
	IIC_SCL1=0;//钳住I2C总线，准备发送或接收数据 
}	 

//产生IIC停止信号
static void IIC1_Stop(void)
{
	SDA_OUT1();//sda线输出
	IIC_SCL1=0;
	IIC_SDA1=0;//STOP:when CLK is high DATA change form low to high
 	delay1();
	IIC_SCL1=1; 
	IIC_SDA1=1;//发送I2C总线结束信号
	delay1();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 IIC1_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN1();      //SDA设置为输入  
	IIC_SDA1=1;delay1();	   
	IIC_SCL1=1;delay1();	 
	while(READ_SDA1)
	{
		ucErrTime++;
		if(ucErrTime>TRY_TIME)
		{
			IIC1_Stop();
			return 1;
		}
	}
	IIC_SCL1=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
static void IIC1_Ack(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=0;
	delay1();
	IIC_SCL1=1;
	delay1();
	IIC_SCL1=0;
}

//不产生ACK应答		    
static void IIC1_NAck(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=1;
	delay1();
	IIC_SCL1=1;
	delay1();
	IIC_SCL1=0;
}		

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC1_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT1(); 	    
    IIC_SCL1=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA1=(txd&0x80)>>7;
        txd<<=1; 	  
		delay1();   //对TEA5767这三个延时都是必须的
		IIC_SCL1=1;
		delay1(); 
		IIC_SCL1=0;	
		delay1();
    }	 
} 	

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN1();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL1=0; 
        delay1();
		IIC_SCL1=1;
        receive<<=1;
        if(READ_SDA1)receive++;   
		delay1(); 
    }					 
    if (!ack)
        IIC1_NAck();//发送nACK
    else
        IIC1_Ack(); //发送ACK   
    return receive;
}



//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 IIC1_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack())          //等待应答
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC1_Send_Byte(buf[i]);  //发送数据
        if(IIC1_Wait_Ack())      //等待ACK
        {
            IIC1_Stop();
            return 1;
        }
    }
    IIC1_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 IIC1_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack())          //等待应答
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答
    IIC1_Start();                
    IIC1_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC1_Read_Byte(0);//读数据,发送nACK 
				else *buf=IIC1_Read_Byte(1);		//读数据,发送ACK  
				len--;
				buf++;  
    }
    IIC1_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 IIC1_Write_One_Byte(u8 addr, u8 reg, u8 data)
{
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack())          //等待应答
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答
    IIC1_Send_Byte(data);        //发送数据
    if(IIC1_Wait_Ack())          //等待ACK
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 IIC1_Read_One_Byte(u8 addr, u8 reg)
{
    u8 res;
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC1_Wait_Ack();             //等待应答
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答
    IIC1_Start();                
    IIC1_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack();             //等待应答
    res=IIC1_Read_Byte(0);		    //读数据,发送nACK  
    IIC1_Stop();                 //产生一个停止条件
    return res;  
}




/********以下是AK8963的驱动代码，由于他的延时比MPU6500长很多，所以将它分离出来，其实这很不明智，代码冗余太多了*********/
//产生IIC起始信号
static void IIC1_Start_(void)
{
	SDA_OUT1();     //sda线输出
	IIC_SDA1=1;	  	  
	IIC_SCL1=1;
	delay3();
 	IIC_SDA1=0;//START:when CLK is high,DATA change form high to low 
	delay3();
	IIC_SCL1=0;//钳住I2C总线，准备发送或接收数据 
}	 

//产生IIC停止信号
static void IIC1_Stop_(void)
{
	SDA_OUT1();//sda线输出
	IIC_SCL1=0;
	IIC_SDA1=0;//STOP:when CLK is high DATA change form low to high
 	delay3();
	IIC_SCL1=1; 
	IIC_SDA1=1;//发送I2C总线结束信号
	delay3();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 IIC1_Wait_Ack_(void)
{
	u8 ucErrTime=0;
	SDA_IN1();      //SDA设置为输入  
	IIC_SDA1=1;delay3();	   
	IIC_SCL1=1;delay3();	 
	while(READ_SDA1)
	{
		ucErrTime++;
		if(ucErrTime>TRY_TIME)
		{
			IIC1_Stop_();
			return 1;
		}
	}
	IIC_SCL1=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
static void IIC1_Ack_(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=0;
	delay3();
	IIC_SCL1=1;
	delay3();
	IIC_SCL1=0;
}

//不产生ACK应答		    
static void IIC1_NAck_(void)
{
	IIC_SCL1=0;
	SDA_OUT1();
	IIC_SDA1=1;
	delay3();
	IIC_SCL1=1;
	delay3();
	IIC_SCL1=0;
}		

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC1_Send_Byte_(u8 txd)
{                        
    u8 t;   
	SDA_OUT1(); 	    
    IIC_SCL1=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA1=(txd&0x80)>>7;
        txd<<=1; 	  
		delay3();   //对TEA5767这三个延时都是必须的
		IIC_SCL1=1;
		delay3(); 
		IIC_SCL1=0;	
		delay3();
    }	 
} 	

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC1_Read_Byte_(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN1();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL1=0; 
        delay3();
		IIC_SCL1=1;
        receive<<=1;
        if(READ_SDA1)receive++;   
		delay3(); 
    }					 
    if (!ack)
        IIC1_NAck_();//发送nACK
    else
        IIC1_Ack_(); //发送ACK   
    return receive;
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 IIC1_Write_One_Byte_(u8 addr, u8 reg, u8 data)
{
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack_())          //等待应答
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Send_Byte_(reg);         //写寄存器地址
    IIC1_Wait_Ack_();             //等待应答
    IIC1_Send_Byte_(data);        //发送数据
    if(IIC1_Wait_Ack_())          //等待ACK
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Stop_();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 IIC1_Read_One_Byte_(u8 addr, u8 reg)
{
    u8 res;
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //发送器件地址+写命令
    IIC1_Wait_Ack_();             //等待应答
    IIC1_Send_Byte_(reg);         //写寄存器地址
    IIC1_Wait_Ack_();             //等待应答
    IIC1_Start_();                
    IIC1_Send_Byte_((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack_();             //等待应答
    res=IIC1_Read_Byte_(0);		    //读数据,发送nACK  
    IIC1_Stop_();                 //产生一个停止条件
    return res;  
}

u8 IIC1_Read_NByte_(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack_())          //等待应答
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Send_Byte_(reg);         //写寄存器地址
    IIC1_Wait_Ack_();             //等待应答
	  IIC1_Start_();                
    IIC1_Send_Byte_((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack_();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC1_Read_Byte_(0);//读数据,发送nACK 
				else *buf=IIC1_Read_Byte_(1);		//读数据,发送ACK  
				len--;
				buf++;  
    }
    IIC1_Stop_();                 //产生一个停止条件
    return 0;       
}


//I2C写一位
void IIC1_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable)
{
    u8 byte;
    byte = IIC1_Read_One_Byte(addr, reg);
    byte = (enable != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum)) ;
    IIC1_Write_One_Byte(addr, reg, byte);
}

//I2C写多位
void IIC1_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	uint8_t byte;

	byte = IIC1_Read_One_Byte(devAddress, memAddress);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte
    IIC1_Write_One_Byte(devAddress, memAddress, byte);
}

//列出IIC总线上所有从机地址
void IIC1_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC1_Start();
        IIC1_Send_Byte((i<<1)|0);
		res = IIC1_Wait_Ack();          //等待应答
		if(res == 0)
			printf("IIC_ADDR1 = 0x%x\r\n",i);
		IIC1_Stop();
	}printf("\r\n");
}

//读取IIC总线上某个器件的所有寄存器
void IIC1_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC1_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}



/************BMP180部分*************/
//产生IIC起始信号
static void IIC2_Start(void)
{
	SDA_OUT2();     //sda线输出
	IIC_SDA2=1;	  	  
	IIC_SCL2=1;
	delay2();
 	IIC_SDA2=0;//START:when CLK is high,DATA change form high to low 
	delay2();
	IIC_SCL2=0;//钳住I2C总线，准备发送或接收数据 
}	 

//产生IIC停止信号
static void IIC2_Stop(void)
{
	SDA_OUT2();//sda线输出
	IIC_SCL2=0;
	IIC_SDA2=0;//STOP:when CLK is high DATA change form low to high
 	delay2();
	IIC_SCL2=1; 
	IIC_SDA2=1;//发送I2C总线结束信号
	delay2();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN2();      //SDA设置为输入  
	IIC_SDA2=1;delay2();	   
	IIC_SCL2=1;delay2();	 
	while(READ_SDA2)
	{
		ucErrTime++;
		if(ucErrTime>TRY_TIME)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC_SCL2=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
static void IIC2_Ack(void)
{
	IIC_SCL2=0;
	SDA_OUT2();
	IIC_SDA2=0;
	delay2();
	IIC_SCL2=1;
	delay2();
	IIC_SCL2=0;
}

//不产生ACK应答		    
static void IIC2_NAck(void)
{
	IIC_SCL2=0;
	SDA_OUT2();
	IIC_SDA2=1;
	delay2();
	IIC_SCL2=1;
	delay2();
	IIC_SCL2=0;
}		

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC2_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT2(); 	    
    IIC_SCL2=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA2=(txd&0x80)>>7;
        txd<<=1; 	  
		delay2();   //对TEA5767这三个延时都是必须的
		IIC_SCL2=1;
		delay2(); 
		IIC_SCL2=0;	
		delay2();
    }	 
} 	

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN2();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL2=0; 
        delay2();
		IIC_SCL2=1;
        receive<<=1;
        if(READ_SDA2)receive++;   
		delay2(); 
    }					 
    if (!ack)
        IIC2_NAck();//发送nACK
    else
        IIC2_Ack(); //发送ACK   
    return receive;
}



//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 IIC2_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC2_Wait_Ack())          //等待应答
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //写寄存器地址
    IIC2_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC2_Send_Byte(buf[i]);  //发送数据
        if(IIC2_Wait_Ack())      //等待ACK
        {
            IIC2_Stop();
            return 1;
        }
    }
    IIC2_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 IIC2_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC2_Wait_Ack())          //等待应答
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //写寄存器地址
    IIC2_Wait_Ack();             //等待应答
    IIC2_Start();                
    IIC2_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC2_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC2_Read_Byte(0);//读数据,发送nACK 
				else *buf=IIC2_Read_Byte(1);		//读数据,发送ACK  
				len--;
				buf++;  
    }
    IIC2_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 IIC2_Write_One_Byte(u8 addr, u8 reg, u8 data)
{
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC2_Wait_Ack())          //等待应答
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //写寄存器地址
    IIC2_Wait_Ack();             //等待应答
    IIC2_Send_Byte(data);        //发送数据
    if(IIC2_Wait_Ack())          //等待ACK
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 IIC2_Read_One_Byte(u8 addr, u8 reg)
{
    u8 res;
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC2_Wait_Ack();             //等待应答
    IIC2_Send_Byte(reg);         //写寄存器地址
    IIC2_Wait_Ack();             //等待应答
    IIC2_Start();                
    IIC2_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC2_Wait_Ack();             //等待应答
    res=IIC2_Read_Byte(0);		    //读数据,发送nACK  
    IIC2_Stop();                 //产生一个停止条件
    return res;  
}

//I2C写一位
void IIC2_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable)
{
    u8 byte;
    byte = IIC2_Read_One_Byte(addr, reg);
    byte = (enable != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum)) ;
    IIC2_Write_One_Byte(addr, reg, byte);
}

//I2C写多位
void IIC2_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	uint8_t byte;

	byte = IIC2_Read_One_Byte(devAddress, memAddress);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte
    IIC2_Write_One_Byte(devAddress, memAddress, byte);
}

//列出IIC总线上所有从机地址
void IIC2_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC2_Start();
        IIC2_Send_Byte((i<<1)|0);
		res = IIC2_Wait_Ack();          //等待应答
		if(res == 0)
			printf("IIC2_ADDR = 0x%x\r\n",i);
		IIC2_Stop();
	}printf("\r\n");
}

//读取IIC总线上某个器件的所有寄存器
void IIC2_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC2_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}


