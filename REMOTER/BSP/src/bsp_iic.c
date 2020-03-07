#include "bsp_iic.h"
//SDA PB12 SCL PB13

void iic_delay(void)
{
    __nop();__nop();__nop();__nop();__nop();__nop();
}

//��ʼ��IIC
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

    //GPIOB8,B9��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
    IIC_SCL=1;
    IIC_SDA=1;
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	iic_delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	 

//����IICֹͣ�ź�
void IIC_Stop(void)
{
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay();
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	iic_delay();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;
	iic_delay();
	IIC_SCL=1;
	iic_delay();
	IIC_SCL=0;
}

//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	IIC_SDA=1;
	iic_delay();
	IIC_SCL=1;
	iic_delay();
	IIC_SCL=0;
}		

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		iic_delay(); 
		IIC_SCL=0;	
		iic_delay();
    }
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 IIC_Write_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 IIC_Read_NByte(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
				else *buf=IIC_Read_Byte(1);		//������,����ACK  
				len--;
				buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 IIC_Write_One_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //��������
    if(IIC_Wait_Ack())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 IIC_Read_One_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		    //������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}


//�г�IIC���������дӻ���ַ
void IIC_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC_Start();
    IIC_Send_Byte((i<<1)|0);
		res = IIC_Wait_Ack();          //�ȴ�Ӧ��
		if(res == 0)
			printf("IIC_ADDR = %#x\r\n",i);
		IIC_Stop();
	}printf("\r\n");
}

//��ȡIIC������ĳ�����������мĴ���
void IIC_Slave_Register(u8 Slave_Addr)
{
	u8 i = 0,res = 0;
	for(i = 0 ;i<255;i++)
	{
		res = IIC_Read_One_Byte(Slave_Addr,i);
		printf("IIC_%#x_%#x = %#x\r\n",Slave_Addr,i,res);
	}printf("\r\n\r\n");
}

