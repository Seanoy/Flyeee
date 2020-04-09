#include "bsp_myiic.h"
#include <stdio.h>
#define TRY_TIME        50

void delay1(void)//mpu6500��ʱ
{
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();
}

void delay3(void)//ak8963��ʱ ��ʱҪ�㹻�� ak8963���ܶ�ȡ������ ��ΪSTM32F407��ƵΪ168MHz һ��nopʱ��Ϊ1/168MHz
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

void delay2(void)//BMP180��ʱ
{
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
    __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}
//��ʼ��IIC
void IIC_Init(void)
{			
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

    //GPIOB8,B9��ʼ������     MPU6500����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
    IIC_SCL1=1;
    IIC_SDA1=1;
    
    //PE2-SCL PE3-SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
    IIC_SCL2=1;
    IIC_SDA2=1;
}

//����IIC��ʼ�ź�
static void IIC1_Start(void)
{
	SDA_OUT1();     //sda�����
	IIC_SDA1=1;	  	  
	IIC_SCL1=1;
	delay1();
 	IIC_SDA1=0;//START:when CLK is high,DATA change form high to low 
	delay1();
	IIC_SCL1=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	 

//����IICֹͣ�ź�
static void IIC1_Stop(void)
{
	SDA_OUT1();//sda�����
	IIC_SCL1=0;
	IIC_SDA1=0;//STOP:when CLK is high DATA change form low to high
 	delay1();
	IIC_SCL1=1; 
	IIC_SDA1=1;//����I2C���߽����ź�
	delay1();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 IIC1_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN1();      //SDA����Ϊ����  
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
	IIC_SCL1=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC1_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT1(); 	    
    IIC_SCL1=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA1=(txd&0x80)>>7;
        txd<<=1; 	  
		delay1();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL1=1;
		delay1(); 
		IIC_SCL1=0;	
		delay1();
    }	 
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC1_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN1();//SDA����Ϊ����
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
        IIC1_NAck();//����nACK
    else
        IIC1_Ack(); //����ACK   
    return receive;
}



//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 IIC1_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC1_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC1_Send_Byte(buf[i]);  //��������
        if(IIC1_Wait_Ack())      //�ȴ�ACK
        {
            IIC1_Stop();
            return 1;
        }
    }
    IIC1_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 IIC1_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC1_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    IIC1_Start();                
    IIC1_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC1_Read_Byte(0);//������,����nACK 
				else *buf=IIC1_Read_Byte(1);		//������,����ACK  
				len--;
				buf++;  
    }
    IIC1_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 IIC1_Write_One_Byte(u8 addr, u8 reg, u8 data)
{
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC1_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    IIC1_Send_Byte(data);        //��������
    if(IIC1_Wait_Ack())          //�ȴ�ACK
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 IIC1_Read_One_Byte(u8 addr, u8 reg)
{
    u8 res;
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    IIC1_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    IIC1_Start();                
    IIC1_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC1_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC1_Read_Byte(0);		    //������,����nACK  
    IIC1_Stop();                 //����һ��ֹͣ����
    return res;  
}




/********������AK8963���������룬����������ʱ��MPU6500���ܶ࣬���Խ��������������ʵ��ܲ����ǣ���������̫����*********/
//����IIC��ʼ�ź�
static void IIC1_Start_(void)
{
	SDA_OUT1();     //sda�����
	IIC_SDA1=1;	  	  
	IIC_SCL1=1;
	delay3();
 	IIC_SDA1=0;//START:when CLK is high,DATA change form high to low 
	delay3();
	IIC_SCL1=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	 

//����IICֹͣ�ź�
static void IIC1_Stop_(void)
{
	SDA_OUT1();//sda�����
	IIC_SCL1=0;
	IIC_SDA1=0;//STOP:when CLK is high DATA change form low to high
 	delay3();
	IIC_SCL1=1; 
	IIC_SDA1=1;//����I2C���߽����ź�
	delay3();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 IIC1_Wait_Ack_(void)
{
	u8 ucErrTime=0;
	SDA_IN1();      //SDA����Ϊ����  
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
	IIC_SCL1=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC1_Send_Byte_(u8 txd)
{                        
    u8 t;   
	SDA_OUT1(); 	    
    IIC_SCL1=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA1=(txd&0x80)>>7;
        txd<<=1; 	  
		delay3();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL1=1;
		delay3(); 
		IIC_SCL1=0;	
		delay3();
    }	 
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC1_Read_Byte_(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN1();//SDA����Ϊ����
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
        IIC1_NAck_();//����nACK
    else
        IIC1_Ack_(); //����ACK   
    return receive;
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 IIC1_Write_One_Byte_(u8 addr, u8 reg, u8 data)
{
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //����������ַ+д����
    if(IIC1_Wait_Ack_())          //�ȴ�Ӧ��
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Send_Byte_(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
    IIC1_Send_Byte_(data);        //��������
    if(IIC1_Wait_Ack_())          //�ȴ�ACK
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Stop_();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 IIC1_Read_One_Byte_(u8 addr, u8 reg)
{
    u8 res;
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //����������ַ+д����
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
    IIC1_Send_Byte_(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
    IIC1_Start_();                
    IIC1_Send_Byte_((addr<<1)|1); //����������ַ+������
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
    res=IIC1_Read_Byte_(0);		    //������,����nACK  
    IIC1_Stop_();                 //����һ��ֹͣ����
    return res;  
}

u8 IIC1_Read_NByte_(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC1_Start_();
    IIC1_Send_Byte_((addr<<1)|0); //����������ַ+д����
    if(IIC1_Wait_Ack_())          //�ȴ�Ӧ��
    {
        IIC1_Stop_();
        return 1;
    }
    IIC1_Send_Byte_(reg);         //д�Ĵ�����ַ
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
	  IIC1_Start_();                
    IIC1_Send_Byte_((addr<<1)|1); //����������ַ+������
    IIC1_Wait_Ack_();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC1_Read_Byte_(0);//������,����nACK 
				else *buf=IIC1_Read_Byte_(1);		//������,����ACK  
				len--;
				buf++;  
    }
    IIC1_Stop_();                 //����һ��ֹͣ����
    return 0;       
}


//I2Cдһλ
void IIC1_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable)
{
    u8 byte;
    byte = IIC1_Read_One_Byte(addr, reg);
    byte = (enable != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum)) ;
    IIC1_Write_One_Byte(addr, reg, byte);
}

//I2Cд��λ
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

//�г�IIC���������дӻ���ַ
void IIC1_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC1_Start();
        IIC1_Send_Byte((i<<1)|0);
		res = IIC1_Wait_Ack();          //�ȴ�Ӧ��
		if(res == 0)
			printf("IIC_ADDR1 = 0x%x\r\n",i);
		IIC1_Stop();
	}printf("\r\n");
}

//��ȡIIC������ĳ�����������мĴ���
void IIC1_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC1_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}



/************BMP180����*************/
//����IIC��ʼ�ź�
static void IIC2_Start(void)
{
	SDA_OUT2();     //sda�����
	IIC_SDA2=1;	  	  
	IIC_SCL2=1;
	delay2();
 	IIC_SDA2=0;//START:when CLK is high,DATA change form high to low 
	delay2();
	IIC_SCL2=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	 

//����IICֹͣ�ź�
static void IIC2_Stop(void)
{
	SDA_OUT2();//sda�����
	IIC_SCL2=0;
	IIC_SDA2=0;//STOP:when CLK is high DATA change form low to high
 	delay2();
	IIC_SCL2=1; 
	IIC_SDA2=1;//����I2C���߽����ź�
	delay2();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN2();      //SDA����Ϊ����  
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
	IIC_SCL2=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC2_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT2(); 	    
    IIC_SCL2=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA2=(txd&0x80)>>7;
        txd<<=1; 	  
		delay2();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL2=1;
		delay2(); 
		IIC_SCL2=0;	
		delay2();
    }	 
} 	

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC2_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN2();//SDA����Ϊ����
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
        IIC2_NAck();//����nACK
    else
        IIC2_Ack(); //����ACK   
    return receive;
}



//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 IIC2_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC2_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC2_Send_Byte(buf[i]);  //��������
        if(IIC2_Wait_Ack())      //�ȴ�ACK
        {
            IIC2_Stop();
            return 1;
        }
    }
    IIC2_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 IIC2_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC2_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    IIC2_Start();                
    IIC2_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC2_Read_Byte(0);//������,����nACK 
				else *buf=IIC2_Read_Byte(1);		//������,����ACK  
				len--;
				buf++;  
    }
    IIC2_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 IIC2_Write_One_Byte(u8 addr, u8 reg, u8 data)
{
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC2_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    IIC2_Send_Byte(data);        //��������
    if(IIC2_Wait_Ack())          //�ȴ�ACK
    {
        IIC2_Stop();
        return 1;
    }
    IIC2_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 IIC2_Read_One_Byte(u8 addr, u8 reg)
{
    u8 res;
    IIC2_Start();
    IIC2_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    IIC2_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    IIC2_Start();                
    IIC2_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC2_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC2_Read_Byte(0);		    //������,����nACK  
    IIC2_Stop();                 //����һ��ֹͣ����
    return res;  
}

//I2Cдһλ
void IIC2_WriteBit(u8 addr, u8 reg, u8 bitNum, u8 enable)
{
    u8 byte;
    byte = IIC2_Read_One_Byte(addr, reg);
    byte = (enable != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum)) ;
    IIC2_Write_One_Byte(addr, reg, byte);
}

//I2Cд��λ
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

//�г�IIC���������дӻ���ַ
void IIC2_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC2_Start();
        IIC2_Send_Byte((i<<1)|0);
		res = IIC2_Wait_Ack();          //�ȴ�Ӧ��
		if(res == 0)
			printf("IIC2_ADDR = 0x%x\r\n",i);
		IIC2_Stop();
	}printf("\r\n");
}

//��ȡIIC������ĳ�����������мĴ���
void IIC2_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC2_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}


