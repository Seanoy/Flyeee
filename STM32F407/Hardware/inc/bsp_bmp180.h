#ifndef __BSP_BMP180_H
#define __BSP_BMP180_H

#include "stm32f4xx.h"
#include "bsp_myiic.h"
#include <math.h>


#define BMP180_ADDR				                0x77			//BMP180从机地址

//两字节
#define BMP180_AC1_REG				            0xAA
#define BMP180_AC2_REG				            0xAC
#define BMP180_AC3_REG				            0xAE
#define BMP180_AC4_REG				            0xB0
#define BMP180_AC5_REG				            0xB2
#define BMP180_AC6_REG				            0xB4
#define BMP180_B1_REG				            0xB6
#define BMP180_B2_REG				            0xB8
#define BMP180_MB_REG				            0xBA
#define BMP180_MC_REG				            0xBC
#define BMP180_MD_REG				            0xBE

//全局存储
#define BMP180_OUT_XLSB_REG						0xF8			//default:0 		readonly
#define BMP180_OUT_LSB_REG						0xF7			//default:0			readonly
#define BMP180_OUT_MSB_REG						0xF6			//default:0x80	readonly
#define BMP180_CTRL_MEAS_REG					0xF4			//default:0			readwrite	ctrl_meas -> F4h (oss<1:0> 2bits) (sco 1bit) (measurement control 5 bits) 
#define BMP180_SOFT_RESET_REG					0xE0			//default:0			readwrite
#define BMP180_ID								0x55			//BMP180的ID值	readonly
//calib21 downto calib0 BFh downto AAh   readonly



typedef struct 												//BMP180数据结构体
{
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	long B3;
	unsigned long B4;
	long B5;
	long B6;
	long B7;
	short MB;
	short MC;
	short MD;
	long UT;					
	long UP;
	long X1;
	long X2;
	long X3;
	long p;
	long Temp;
	float altitude;	
}BMP180_Data;

extern BMP180_Data BMP180;

void BMP_Init(void);
uint8_t BMP_ReadOneByte(uint8_t ReadAddr);
void BMP_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite);
short BMP_ReadTwoByte(uint8_t ReadAddr);
void BMP_ReadCalibrationData(void);
long BMP_Read_UT(void);
long BMP_Read_UP(void);
void BMP_UncompemstatedToTrue(void);

#endif
