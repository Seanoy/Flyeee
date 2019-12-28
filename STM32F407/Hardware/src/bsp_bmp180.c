#include "bsp_BMP180.h"

//�洢BMP180���ݵĽṹ
BMP180_Data BMP180;

//BMP180��ʼ��
void BMP_Init(void)
{
//	��ʼ�� ��ȡУ��ֵ BMP_ReadCalibrationData();
//��ȡ����
//		ID=BMP_ReadOneByte(0xd0); 
//		BMP_UncompemstatedToTrue();
//		printf("ID = %d\t  temp = %d.%dC\t   Pressure = %ldPa\t   Altitude = %.5fm\r\n",ID,BMP180.Temp/10,BMP180.Temp%10,BMP180.p,BMP180.altitude);

}


//дһ�����ݵ�BMP180
void BMP_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Write_One_Byte(WriteAddr,BMP180_ADDR,DataToWrite);
}

//��BMP180��һ���ֽ�����
uint8_t BMP_ReadOneByte(uint8_t ReadAddr)
{
return IIC_Read_One_Byte(BMP180_ADDR,ReadAddr);	
}

//��BMP180��һ��16λ������
short BMP_ReadTwoByte(uint8_t ReadAddr)
{
	u8 buf[2]; 
	IIC_Read_NByte(BMP180_ADDR,ReadAddr,2,buf);
	return ((u16)buf[1]<<8)|buf[0];  
}

//��BMP180�Ļ�ȡ�������
void BMP_ReadCalibrationData(void)
{
	BMP180.AC1 = BMP_ReadTwoByte(BMP180_AC1_REG);
	BMP180.AC2 = BMP_ReadTwoByte(BMP180_AC2_REG);
	BMP180.AC3 = BMP_ReadTwoByte(BMP180_AC3_REG);
	BMP180.AC4 = BMP_ReadTwoByte(BMP180_AC4_REG);
	BMP180.AC5 = BMP_ReadTwoByte(BMP180_AC5_REG);
	BMP180.AC6 = BMP_ReadTwoByte(BMP180_AC6_REG);
	BMP180.B1  = BMP_ReadTwoByte(BMP180_B1_REG);
	BMP180.B2  = BMP_ReadTwoByte(BMP180_B2_REG);
	BMP180.MB  = BMP_ReadTwoByte(BMP180_MB_REG);
	BMP180.MC  = BMP_ReadTwoByte(BMP180_MC_REG);
	BMP180.MD  = BMP_ReadTwoByte(BMP180_MD_REG);
}

//��BMP180��ȡδ�������¶�
long BMP_Read_UT(void)
{
	long temp = 0;
	BMP_WriteOneByte(0xF4,0x2E);
	
//	delay_ms(5);
	temp = (long)BMP_ReadTwoByte(0xF6);
	return temp;
}

//��BMP180��ȡδ�����Ĵ���ѹ
long BMP_Read_UP(void)
{
	long pressure = 0;
	
	BMP_WriteOneByte(0xF4,0x34);
//	delay_ms(5);
	
	pressure = (long)BMP_ReadTwoByte(0xF6);
	//pressure = pressure + BMP_ReadOneByte(0xf8);
	pressure &= 0x0000FFFF;
	
	return pressure;
}

//�û�ȡ�Ĳ������¶Ⱥʹ���ѹ���������������㺣��
void BMP_UncompemstatedToTrue(void)
{
	BMP180.UT = BMP_Read_UT();//��һ�ζ�ȡ����
	BMP180.UT = BMP_Read_UT();//���еڶ��ζ�ȡ��������
	BMP180.UP = BMP_Read_UP();
	
	BMP180.X1 = ((BMP180.UT - BMP180.AC6) * BMP180.AC5) >> 15;
	BMP180.X2 = (((long)BMP180.MC) << 11) / (BMP180.X1 + BMP180.MD);
	BMP180.B5 = BMP180.X1 + BMP180.X2;
	BMP180.Temp  = (BMP180.B5 + 8) >> 4;
	
	BMP180.B6 = BMP180.B5 - 4000;
	BMP180.X1 = ((long)BMP180.B2 * (BMP180.B6 * BMP180.B6 >> 12)) >> 11;
	BMP180.X2 = ((long)BMP180.AC2) * BMP180.B6 >> 11;
	BMP180.X3 = BMP180.X1 + BMP180.X2;
	
	BMP180.B3 = ((((long)BMP180.AC1) * 4 + BMP180.X3) + 2) /4;
	BMP180.X1 = ((long)BMP180.AC3) * BMP180.B6 >> 13;
	BMP180.X2 = (((long)BMP180.B1) *(BMP180.B6*BMP180.B6 >> 12)) >>16;
	BMP180.X3 = ((BMP180.X1 + BMP180.X2) + 2) >> 2;
	BMP180.B4 = ((long)BMP180.AC4) * (unsigned long)(BMP180.X3 + 32768) >> 15;
	BMP180.B7 = ((unsigned long)BMP180.UP - BMP180.B3) * 50000;
	
	if(BMP180.B7 < 0x80000000)
	{
		BMP180.p = (BMP180.B7 * 2) / BMP180.B4;		
	}
	else
	{
		BMP180.p = (BMP180.B7 / BMP180.B4) * 2;
	}
	
	BMP180.X1 = (BMP180.p >> 8) * (BMP180.p >>8);
	BMP180.X1 = (((long)BMP180.X1) * 3038) >> 16;
	BMP180.X2 = (-7357 * BMP180.p) >> 16;
	
	BMP180.p = BMP180.p + ((BMP180.X1 + BMP180.X2 + 3791) >> 4);
	
	BMP180.altitude = 44330 * (1-pow(((BMP180.p) / 101325.0),(1.0/5.255)));  
}
