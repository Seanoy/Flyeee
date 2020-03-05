#ifndef __BSP_SPI_H
#define __BSP_SPI_H
#include "bsp_sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

#include "stm32f4xx.h"
// SPI�����ٶ����� 
#define SPI_SPEED_2   SPI_BaudRatePrescaler_2
#define SPI_SPEED_8   SPI_BaudRatePrescaler_8
#define SPI_SPEED_16  SPI_BaudRatePrescaler_16
#define SPI_SPEED_256 SPI_BaudRatePrescaler_256
						  	    													  
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
		 
#endif

