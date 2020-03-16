#ifndef __BSP_AK8963_H
#define __BSP_AK8963_H

#include <stm32f4xx.h>
#include <stdio.h>
#include "bsp_sys.h" 	
#include "bsp_myiic.h"
#include "sensor_type.h"

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDR						0X0C	//AK8963的I2C地址
#define AK8963_ID                       0X48	//AK8963的器件ID

//AK8963的内部寄存器
#define AK8963_WIA								0x00	//AK8963的器件ID寄存器地址
#define AK8963_RA_INFO        0x01
#define AK8963_RA_ST1         0x02

#define AK8963_XOUT_L						0X03	
#define AK8963_XOUT_H						0X04
#define AK8963_YOUT_L						0X05
#define AK8963_YOUT_H						0X06
#define AK8963_ZOUT_L						0X07
#define AK8963_ZOUT_H						0X08

#define AK8963_CNTL1         		0X0A    
#define AK8963_CNTL2         		0X0B
#define AK8963_RA_ASTC        	0x0C
#define AK8963_RA_TS1         	0x0D // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_TS2         	0x0E // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_I2CDIS     		0x0F
#define AK8963_RA_ASAX        	0x10
#define AK8963_RA_ASAY        	0x11
#define AK8963_RA_ASAZ        	0x12

#define AK8963_ST1_DRDY_BIT       0

#define AK8963_ST2_HOFL_BIT       3
#define AK8963_ST2_DERR_BIT       2

#define AK8963_CNTL_MODE_BIT      3
#define AK8963_CNTL_MODE_LENGTH   4

#define AK8963_MODE_POWERDOWN     0x00
#define AK8963_MODE_SINGLE        0x01
#define AK8963_MODE_CONT1         0x02
#define AK8963_MODE_CONT2         0x06
#define AK8963_MODE_EXTTRIG       0x04
#define AK8963_MODE_SELFTEST      0x08
#define AK8963_MODE_FUSEROM       0x0F
#define AK8963_MODE_14BIT         0x00
#define AK8963_MODE_16BIT         0x10

#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS			0x1B
#define AK8963_I2CDIS_BIT         0

#define AK8963_ST_X_MIN           (s16)(-200)
#define AK8963_ST_X_MAX           (s16)(200)
#define AK8963_ST_Y_MIN           (s16)(-200)
#define AK8963_ST_Y_MAX           (s16)(200)
#define AK8963_ST_Z_MIN           (s16)(-3200)
#define AK8963_ST_Z_MAX           (s16)(-800)

#define MAG_GAUSS_PER_LSB		(float)(666.7f)
    
u8 AK8963_Init(void);
u8 MPU_Get_Magnetometer(axis3f_t *mag_s);

#endif
