#ifndef __BSP_MPU6500_H
#define __BSP_MPU6500_H

#include <stm32f4xx.h>
#include <stdio.h>
#include "bsp_sys.h" 	
#include "bsp_myiic.h"
#include "sensors.h"
#include "stdbool.h"

//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define MPU9250_ADDR      		0X68    //MPU9250������I2C��ַ
#define MPU9250_ID              0X71  	//MPU6500������ID


//MPU6500���ڲ��Ĵ���
#define MPU_SELF_TEST_X_GYRO_REG		0x00	//���ٶ��Լ�Ĵ���X
#define MPU_SELF_TEST_Y_GYRO_REG		0x01	//���ٶ��Լ�Ĵ���Y
#define MPU_SELF_TEST_Z_GYRO_REG		0x02	//���ٶ��Լ�Ĵ���Z
#define MPU_SELF_TEST_X_ACCEL_REG		0X0D	//���ٶ��Լ�Ĵ���X
#define MPU_SELF_TEST_Y_ACCEL_REG		0X0E	//���ٶ��Լ�Ĵ���Y
#define MPU_SELF_TEST_Z_ACCEL_REG		0X0F	//���ٶ��Լ�Ĵ���Z

//#define MPU_SELF_TESTA_REG					0X10	//�Լ�Ĵ���A

#define MPU_XG_OFFSET_H_REG					0x13	
#define MPU_XG_OFFSET_L_REG					0x14
#define MPU_YG_OFFSET_H_REG					0x15
#define MPU_YG_OFFSET_L_REG					0x16
#define MPU_ZG_OFFSET_H_REG					0x17
#define MPU_ZG_OFFSET_L_REG					0x18

#define MPU_SAMPLE_RATE_REG					0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG							0X1A	//��ͨ�˲������üĴ���
#define MPU_GYRO_CFG_REG					0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG					0X1C	//���ٶȼ����üĴ���
#define MPU_ACCEL_CFG2_REG					0X1D
#define MPU_LP_ACCEL_ODR_REG				0X1E
#define MPU_MOTION_DET_REG					0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG						0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG					0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG				0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG						0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG				0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG				0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG						0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG				0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG				0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG						0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG				0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG				0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG						0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG				0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG				0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG						0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG					0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG				0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG					0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG					0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG					0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG						0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG						0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG					0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG					0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG					0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG					0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG					0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG					0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG					0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG					0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG					0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG					0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG					0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG					0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG					0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG					0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_EXT_SENS_DATA_00_REG		    0x49
#define MPU_EXT_SENS_DATA_01_REG		    0x4A
#define MPU_EXT_SENS_DATA_02_REG		    0x4B
#define MPU_EXT_SENS_DATA_03_REG		    0x4C
#define MPU_EXT_SENS_DATA_04_REG		    0x4D
#define MPU_EXT_SENS_DATA_05_REG		    0x4E
#define MPU_EXT_SENS_DATA_06_REG		    0x4F
#define MPU_EXT_SENS_DATA_07_REG		    0x50
#define MPU_EXT_SENS_DATA_08_REG		    0x51
#define MPU_EXT_SENS_DATA_09_REG		    0x52
#define MPU_EXT_SENS_DATA_10_REG		    0x53
#define MPU_EXT_SENS_DATA_11_REG		    0x54
#define MPU_EXT_SENS_DATA_12_REG		    0x55
#define MPU_EXT_SENS_DATA_13_REG		    0x56
#define MPU_EXT_SENS_DATA_14_REG		    0x57
#define MPU_EXT_SENS_DATA_15_REG		    0x58
#define MPU_EXT_SENS_DATA_16_REG		    0x59
#define MPU_EXT_SENS_DATA_17_REG		    0x5A
#define MPU_EXT_SENS_DATA_18_REG		    0x5B
#define MPU_EXT_SENS_DATA_19_REG		    0x5C
#define MPU_EXT_SENS_DATA_20_REG		    0x5D
#define MPU_EXT_SENS_DATA_21_REG		    0x5E
#define MPU_EXT_SENS_DATA_22_REG		    0x5F
#define MPU_EXT_SENS_DATA_23_REG		    0x60

#define MPU_I2CSLV0_DO_REG					0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG					0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG					0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG					0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG				0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG					0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG				0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG					0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG					0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG					0X6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG					0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG					0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG						0X74	//FIFO��д�Ĵ���
#define MPU_WHO_AM_I_REG					0X75	//����ID�Ĵ���

#define MPU_XA_OFFSET_H_REG					0X77
#define MPU_XA_OFFSET_L_REG					0X78
#define MPU_YA_OFFSET_H_REG					0X7A
#define MPU_YA_OFFSET_L_REG					0X7B
#define MPU_ZA_OFFSET_H_REG					0X7D
#define MPU_ZA_OFFSET_L_REG					0X7E


//register bits
#define MPU6500_TC_PWR_MODE_BIT     7
#define MPU6500_TC_OFFSET_BIT       6
#define MPU6500_TC_OFFSET_LENGTH    6
#define MPU6500_TC_OTP_BNK_VLD_BIT  0

#define MPU6500_VDDIO_LEVEL_VLOGIC  0
#define MPU6500_VDDIO_LEVEL_VDD     1

#define MPU6500_CFG_EXT_SYNC_SET_BIT    5
#define MPU6500_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6500_CFG_DLPF_CFG_BIT    2
#define MPU6500_CFG_DLPF_CFG_LENGTH 3

#define MPU6500_EXT_SYNC_DISABLED       0x0
#define MPU6500_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6500_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6500_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6500_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6500_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6500_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6500_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6500_DLPF_BW_256         0x00
#define MPU6500_DLPF_BW_188         0x01
#define MPU6500_DLPF_BW_98          0x02
#define MPU6500_DLPF_BW_42          0x03
#define MPU6500_DLPF_BW_20          0x04
#define MPU6500_DLPF_BW_10          0x05
#define MPU6500_DLPF_BW_5           0x06

#define MPU6500_GCONFIG_XG_ST_BIT       7
#define MPU6500_GCONFIG_YG_ST_BIT       6
#define MPU6500_GCONFIG_ZG_ST_BIT       5
#define MPU6500_GCONFIG_FS_SEL_BIT      4
#define MPU6500_GCONFIG_FS_SEL_LENGTH   2


#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03

#define MPU6500_ACONFIG_XA_ST_BIT           7
#define MPU6500_ACONFIG_YA_ST_BIT           6
#define MPU6500_ACONFIG_ZA_ST_BIT           5
#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6500_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6500_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6500_ACONFIG2_FCHOICE_B_BIT      2
#define MPU6500_ACONFIG2_FCHOICE_B_LENGTH   2
#define MPU6500_ACONFIG2_DLPF_BIT           0
#define MPU6500_ACONFIG2_DLPF_LENGTH        2

#define MPU6500_ACCEL_DLPF_BW_460   0x00
#define MPU6500_ACCEL_DLPF_BW_184   0x01
#define MPU6500_ACCEL_DLPF_BW_92    0x02
#define MPU6500_ACCEL_DLPF_BW_41    0x03
#define MPU6500_ACCEL_DLPF_BW_20    0x04
#define MPU6500_ACCEL_DLPF_BW_10    0x05
#define MPU6500_ACCEL_DLPF_BW_5     0x06

#define MPU6500_ACCEL_FS_2          0x00
#define MPU6500_ACCEL_FS_4          0x01
#define MPU6500_ACCEL_FS_8          0x02
#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_DHPF_RESET          0x00
#define MPU6500_DHPF_5              0x01
#define MPU6500_DHPF_2P5            0x02
#define MPU6500_DHPF_1P25           0x03
#define MPU6500_DHPF_0P63           0x04
#define MPU6500_DHPF_HOLD           0x07

#define MPU6500_TEMP_FIFO_EN_BIT    7
#define MPU6500_XG_FIFO_EN_BIT      6
#define MPU6500_YG_FIFO_EN_BIT      5
#define MPU6500_ZG_FIFO_EN_BIT      4
#define MPU6500_ACCEL_FIFO_EN_BIT   3
#define MPU6500_SLV2_FIFO_EN_BIT    2
#define MPU6500_SLV1_FIFO_EN_BIT    1
#define MPU6500_SLV0_FIFO_EN_BIT    0

#define MPU6500_MULT_MST_EN_BIT     7
#define MPU6500_WAIT_FOR_ES_BIT     6
#define MPU6500_SLV_3_FIFO_EN_BIT   5
#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4

#define MPU6500_CLOCK_DIV_348       0x0
#define MPU6500_CLOCK_DIV_333       0x1
#define MPU6500_CLOCK_DIV_320       0x2
#define MPU6500_CLOCK_DIV_308       0x3
#define MPU6500_CLOCK_DIV_296       0x4
#define MPU6500_CLOCK_DIV_286       0x5
#define MPU6500_CLOCK_DIV_276       0x6
#define MPU6500_CLOCK_DIV_267       0x7
#define MPU6500_CLOCK_DIV_258       0x8
#define MPU6500_CLOCK_DIV_500       0x9
#define MPU6500_CLOCK_DIV_471       0xA
#define MPU6500_CLOCK_DIV_444       0xB
#define MPU6500_CLOCK_DIV_421       0xC
#define MPU6500_CLOCK_DIV_400       0xD
#define MPU6500_CLOCK_DIV_381       0xE
#define MPU6500_CLOCK_DIV_364       0xF

#define MPU6500_I2C_SLV_RW_BIT      7
#define MPU6500_I2C_SLV_ADDR_BIT    6
#define MPU6500_I2C_SLV_ADDR_LENGTH 7
#define MPU6500_I2C_SLV_EN_BIT      7
#define MPU6500_I2C_SLV_BYTE_SW_BIT 6
#define MPU6500_I2C_SLV_REG_DIS_BIT 5
#define MPU6500_I2C_SLV_GRP_BIT     4
#define MPU6500_I2C_SLV_LEN_BIT     3
#define MPU6500_I2C_SLV_LEN_LENGTH  4

#define MPU6500_I2C_SLV4_RW_BIT         7
#define MPU6500_I2C_SLV4_ADDR_BIT       6
#define MPU6500_I2C_SLV4_ADDR_LENGTH    7
#define MPU6500_I2C_SLV4_EN_BIT         7
#define MPU6500_I2C_SLV4_INT_EN_BIT     6
#define MPU6500_I2C_SLV4_REG_DIS_BIT    5
#define MPU6500_I2C_SLV4_MST_DLY_BIT    4
#define MPU6500_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6500_MST_PASS_THROUGH_BIT    7
#define MPU6500_MST_I2C_SLV4_DONE_BIT   6
#define MPU6500_MST_I2C_LOST_ARB_BIT    5
#define MPU6500_MST_I2C_SLV4_NACK_BIT   4
#define MPU6500_MST_I2C_SLV3_NACK_BIT   3
#define MPU6500_MST_I2C_SLV2_NACK_BIT   2
#define MPU6500_MST_I2C_SLV1_NACK_BIT   1
#define MPU6500_MST_I2C_SLV0_NACK_BIT   0

#define MPU6500_INTCFG_INT_LEVEL_BIT        7
#define MPU6500_INTCFG_INT_OPEN_BIT         6
#define MPU6500_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6500_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6500_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6500_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6500_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6500_INTCFG_CLKOUT_EN_BIT        0

#define MPU6500_INTMODE_ACTIVEHIGH  0x00
#define MPU6500_INTMODE_ACTIVELOW   0x01

#define MPU6500_INTDRV_PUSHPULL     0x00
#define MPU6500_INTDRV_OPENDRAIN    0x01

#define MPU6500_INTLATCH_50USPULSE  0x00
#define MPU6500_INTLATCH_WAITCLEAR  0x01

#define MPU6500_INTCLEAR_STATUSREAD 0x00
#define MPU6500_INTCLEAR_ANYREAD    0x01

#define MPU6500_INTERRUPT_FF_BIT            7
#define MPU6500_INTERRUPT_MOT_BIT           6
#define MPU6500_INTERRUPT_ZMOT_BIT          5
#define MPU6500_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6500_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6500_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6500_INTERRUPT_DMP_INT_BIT       1
#define MPU6500_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6500_DMPINT_5_BIT            5
#define MPU6500_DMPINT_4_BIT            4
#define MPU6500_DMPINT_3_BIT            3
#define MPU6500_DMPINT_2_BIT            2
#define MPU6500_DMPINT_1_BIT            1
#define MPU6500_DMPINT_0_BIT            0

#define MPU6500_MOTION_MOT_XNEG_BIT     7
#define MPU6500_MOTION_MOT_XPOS_BIT     6
#define MPU6500_MOTION_MOT_YNEG_BIT     5
#define MPU6500_MOTION_MOT_YPOS_BIT     4
#define MPU6500_MOTION_MOT_ZNEG_BIT     3
#define MPU6500_MOTION_MOT_ZPOS_BIT     2
#define MPU6500_MOTION_MOT_ZRMOT_BIT    0

#define MPU6500_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6500_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6500_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6500_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6500_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6500_PATHRESET_GYRO_RESET_BIT    2
#define MPU6500_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6500_PATHRESET_TEMP_RESET_BIT    0

#define MPU6500_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6500_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6500_DETECT_FF_COUNT_BIT             3
#define MPU6500_DETECT_FF_COUNT_LENGTH          2
#define MPU6500_DETECT_MOT_COUNT_BIT            1
#define MPU6500_DETECT_MOT_COUNT_LENGTH         2

#define MPU6500_DETECT_DECREMENT_RESET  0x0
#define MPU6500_DETECT_DECREMENT_1      0x1
#define MPU6500_DETECT_DECREMENT_2      0x2
#define MPU6500_DETECT_DECREMENT_4      0x3

#define MPU6500_USERCTRL_DMP_EN_BIT             7
#define MPU6500_USERCTRL_FIFO_EN_BIT            6
#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6500_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6500_USERCTRL_DMP_RESET_BIT          3
#define MPU6500_USERCTRL_FIFO_RESET_BIT         2
#define MPU6500_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6500_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_CYCLE_BIT          5
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3

#define MPU6500_CLOCK_INTERNAL          0x00
#define MPU6500_CLOCK_PLL_XGYRO         0x01
#define MPU6500_CLOCK_PLL_YGYRO         0x02
#define MPU6500_CLOCK_PLL_ZGYRO         0x03
#define MPU6500_CLOCK_PLL_EXT32K        0x04
#define MPU6500_CLOCK_PLL_EXT19M        0x05
#define MPU6500_CLOCK_KEEP_RESET        0x07

#define MPU6500_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6500_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6500_PWR2_STBY_XA_BIT            5
#define MPU6500_PWR2_STBY_YA_BIT            4
#define MPU6500_PWR2_STBY_ZA_BIT            3
#define MPU6500_PWR2_STBY_XG_BIT            2
#define MPU6500_PWR2_STBY_YG_BIT            1
#define MPU6500_PWR2_STBY_ZG_BIT            0

#define MPU6500_WAKE_FREQ_1P25      0x0
#define MPU6500_WAKE_FREQ_2P5       0x1
#define MPU6500_WAKE_FREQ_5         0x2
#define MPU6500_WAKE_FREQ_10        0x3

#define MPU6500_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6500_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6500_BANKSEL_MEM_SEL_BIT         4
#define MPU6500_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6500_WHO_AM_I_BIT        6
#define MPU6500_WHO_AM_I_LENGTH     6

#define MPU6500_DMP_MEMORY_BANKS        8
#define MPU6500_DMP_MEMORY_BANK_SIZE    256
#define MPU6500_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6500_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

// Test limits
#define MPU6500_ST_GYRO_LOW      (-14.0)  // %
#define MPU6500_ST_GYRO_HIGH     14.0  // %
#define MPU6500_ST_ACCEL_LOW     (-14.0)  // %
#define MPU6500_ST_ACCEL_HIGH    14.0  // %

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE				4000	/* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* ���ټƲ������� */

// MPU9250����ģʽ��ȡ���� ����������
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	8
#define SENSORS_BARO_STATUS_LEN		1
#define SENSORS_BARO_DATA_LEN		6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)



typedef struct 
{
    short gx;
    short gy;
    short gz;
}gyro_struct;

typedef struct 
{
    short ax;
    short ay;
    short az;
}acc_struct;

//MPU9250��ȡ���¶�ֵ
extern u16 temperature;

u8 MPU9250_Init(void);

short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(axis3f_t *gyro_s);
u8 MPU_Get_Accelerometer(axis3f_t *acc_s);
bool MPU_SelfTest(void);

#endif
