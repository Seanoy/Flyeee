#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "config.h"
#include "stmflash.h"
#include "bsp_systick.h"
#include <stdio.h>
#include "watchdog.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;	
	pidInit_t yaw;	
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
	
	pidInit_t x;
	pidInit_t y;
	pidInit_t z;
} pidParamPos_t;

typedef struct
{
	int16_t accZero[3];
	int16_t accGain[3];
} accBias_t;

typedef struct
{
	int16_t magZero[3];
} magBias_t;

typedef struct 
{
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
} boardAlignment_t;


typedef struct	
{
	u8 version;				/*����汾��*/
	pidParam_t pidAngle;	/*�Ƕ�PID*/	
	pidParam_t pidRate;		/*���ٶ�PID*/	
	pidParamPos_t pidPos;	/*λ��PID*/
//	accBias_t accBias;		/*���ٶ�У׼ֵ*/
//	magBias_t magBias;		/*������У׼ֵ*/
	float trimP;			/*pitch΢��*/
	float trimR;			/*roll΢��*/
	u16 thrustBase;			/*���Ż���ֵ*/
	u8 cksum;				/*У��*/
} configParam_t;



#define BOOTLOADER_SIZE		(16*1024)	
#define CONFIG_PARAM_SIZE	(16*1024)

#define CONFIG_PARAM_ADDR 	(FLASH_BASE + BOOTLOADER_SIZE)	/*16K bootloader*/
#define FIRMWARE_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE + CONFIG_PARAM_SIZE)	/*16K bootloader+ 16 ģ��eeprom*/


#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

#define P_NAME "Flyeee"
#define MCU_ID_ADDRESS          0x1FFF7A10
#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22


extern configParam_t configParam;

void configParamInit(void);	/*�������ó�ʼ��*/
void configParamTask(void* param);	/*������������*/
bool configParamTest(void);

void configParamGiveSemaphore(void);
void resetConfigParamPID(void);
void saveConfigAndNotify(void);



#endif
