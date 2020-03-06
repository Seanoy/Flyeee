#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stm32f10x.h>
#include "bsp_sys.h"
#include <stdio.h>
   	   		   
//IO��������	 
#define IIC_SCL    PBout(13) //SCL
#define IIC_SDA    PBout(12) //SDA
#define READ_SDA   PBin(12)  //����SDA

//IIC���в�������
void IIC_Init(void);          //��ʼ��IIC��IO��				 
void IIC_Start(void);					//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);		//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);						//IIC����ACK�ź�
void IIC_NAck(void);					//IIC������ACK�ź�

//IIC��д����
u8 IIC_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	//дһ���ֽ�
u8 IIC_Read_One_Byte(u8 devaddr,u8 reg);					//��һ���ֽ�
u8 IIC_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);//����д����ֽ�
u8 IIC_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	//����������ֽ�

//IIC���ߺ���
void IIC_Slave_List(void); 						 //�г�IIC�����дӻ���ַ
void IIC_Slave_Register(u8 Slave_Addr);//��ȡIICSlave_Addr�ϵ����мĴ���ֵ

#endif
