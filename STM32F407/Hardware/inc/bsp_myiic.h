#ifndef __BSP_MYIIC_H
#define __BSP_MYIIC_H
#include "bsp_sys.h" 
   	   		   
//MPU9250����IIC
#define SDA_IN1()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB7����ģʽ
#define SDA_OUT1() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}    //PB7���ģʽ
//IO��������	 
#define IIC_SCL1    PBout(6) //SCL
#define IIC_SDA1    PBout(7) //SDA	 
#define READ_SDA1   PBin(7)  //����SDA 

//BMP180����IIC
#define SDA_IN2()  {GPIOE->MODER&=~(3<<(3*2));GPIOE->MODER|=0<<3*2;}	//PE3����ģʽ
#define SDA_OUT2() {GPIOE->MODER&=~(3<<(3*2));GPIOE->MODER|=1<<3*2;}    //PE3���ģʽ
//IO��������	 
#define IIC_SCL2    PEout(2) //SCL
#define IIC_SDA2    PEout(3) //SDA	 
#define READ_SDA2   PEin(3)  //����SDA 

//IIC���в�������
void IIC_Init(void);        //��ʼ��IIC��IO��				 

//IIC1��д���� MPU6500
u8 IIC1_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	    //дһ���ֽ�
u8 IIC1_Read_One_Byte(u8 devaddr,u8 reg);				//��һ���ֽ�
u8 IIC1_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);     //����д����ֽ�
u8 IIC1_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	    //����������ֽ�
void IIC1_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable);
void IIC1_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

//AK8963��IIC��д��������Ҫ�������ʱ��ͬ
u8 IIC1_Write_One_Byte_(u8 devaddr,u8 reg,u8 data);	    //дһ���ֽ�
u8 IIC1_Read_One_Byte_(u8 devaddr,u8 reg);				//��һ���ֽ�
u8 IIC1_Read_NByte_(u8 addr,u8 reg,u8 len,u8 *buf);

//IIC2��д���� BMP180
u8 IIC2_Write_One_Byte(u8 devaddr,u8 reg,u8 data);	    //дһ���ֽ�
u8 IIC2_Read_One_Byte(u8 devaddr,u8 reg);				//��һ���ֽ�
u8 IIC2_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf);     //����д����ֽ�
u8 IIC2_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf);	    //����������ֽ�
void IIC2_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable);
void IIC2_WriteNBit(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data);

//IIC���ߺ���
void IIC1_Slave_List(void); 						    //�г�IIC�����дӻ���ַ
void IIC1_Slave_Register(u8 Slave_Addr);                //��ȡIICSlave_Addr�ϵ����мĴ���ֵ
void IIC2_Slave_List(void); 						    //�г�IIC�����дӻ���ַ
void IIC2_Slave_Register(u8 Slave_Addr);                //��ȡIICSlave_Addr�ϵ����мĴ���ֵ
#endif
