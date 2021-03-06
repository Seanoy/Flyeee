#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stm32f10x.h>
#include "bsp_sys.h"
#include <stdio.h>
   	   		   
//IO操作函数	 
#define IIC_SCL    PBout(13) //SCL
#define IIC_SDA    PBout(12) //SDA
#define READ_SDA   PBin(12)  //输入SDA

//IIC所有操作函数
void IIC_Init(void);          //初始化IIC的IO口				 
void IIC_Start(void);					//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);		//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);						//IIC发送ACK信号
void IIC_NAck(void);					//IIC不发送ACK信号

//IIC读写函数
u8 IIC_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	//写一个字节
u8 IIC_Read_One_Byte(u8 devaddr,u8 reg);					//读一个字节
u8 IIC_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);//连续写多个字节
u8 IIC_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	//连续读多个字节

//IIC工具函数
void IIC_Slave_List(void); 						 //列出IIC上所有从机地址
void IIC_Slave_Register(u8 Slave_Addr);//读取IICSlave_Addr上的所有寄存器值

#endif
