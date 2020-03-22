#ifndef __BSP_MYIIC_H
#define __BSP_MYIIC_H
#include "bsp_sys.h" 
   	   		   
//MPU9250部分IIC
#define SDA_IN1()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB9输入模式
#define SDA_OUT1() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB9输出模式
//IO操作函数	 
#define IIC_SCL1    PBout(6) //SCL
#define IIC_SDA1    PBout(7) //SDA	 
#define READ_SDA1   PBin(7)  //输入SDA 

//BMP180部分IIC
#define SDA_IN2()  {GPIOE->MODER&=~(3<<(3*2));GPIOE->MODER|=0<<3*2;}	//PB9输入模式
#define SDA_OUT2() {GPIOE->MODER&=~(3<<(3*2));GPIOE->MODER|=1<<3*2;}    //PB9输出模式
//IO操作函数	 
#define IIC_SCL2    PEout(2) //SCL
#define IIC_SDA2    PEout(3) //SDA	 
#define READ_SDA2   PEin(3)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);            //初始化IIC的IO口				 
//void IIC1_Start(void);					//发送IIC开始信号
//void IIC1_Stop(void);	  			//发送IIC停止信号
//void IIC1_Send_Byte(u8 txd);		//IIC发送一个字节
//u8 IIC1_Read_Byte(unsigned char ack);//IIC读取一个字节
//u8 IIC1_Wait_Ack(void); 				//IIC等待ACK信号
//void IIC1_Ack(void);						//IIC发送ACK信号
//void IIC1_NAck(void);					//IIC不发送ACK信号

//IIC1读写函数
u8 IIC1_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	//写一个字节
u8 IIC1_Read_One_Byte(u8 devaddr,u8 reg);					//读一个字节
u8 IIC1_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);//连续写多个字节
u8 IIC1_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	//连续读多个字节
void IIC1_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable);
void IIC1_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

//IIC2读写函数
u8 IIC2_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	//写一个字节
u8 IIC2_Read_One_Byte(u8 devaddr,u8 reg);					//读一个字节
u8 IIC2_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);//连续写多个字节
u8 IIC2_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	//连续读多个字节
void IIC2_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable);
void IIC2_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

//IIC工具函数
void IIC1_Slave_List(void); 						 //列出IIC上所有从机地址
void IIC1_Slave_Register(u8 Slave_Addr);                //读取IICSlave_Addr上的所有寄存器值
void IIC2_Slave_List(void); 						 //列出IIC上所有从机地址
void IIC2_Slave_Register(u8 Slave_Addr);                //读取IICSlave_Addr上的所有寄存器值
#endif
