#include <stm32f10x.h>
#ifndef __BSP_SENSORFUSION_H
#define __BSP_SENSORFUSION_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;

typedef union 
{
	struct 
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
} Axis3i32;

typedef union 
{
	struct 
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};
	int64_t axis[3];
} Axis3i64;

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef struct  
{
	float roll;
	float pitch;
	float yaw;
} attitude_t;

/* Orientation as a quaternion */
typedef struct quaternion_s 
{
	uint32_t timestamp;

	union 
	{
		struct 
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct 
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

struct  vec3_s 
{
	u32 timestamp;	/*ʱ���*/

	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;


typedef struct
{
	attitude_t attitude;
	quaternion_t attitudeQuaternion;
	point_t position;
	velocity_t velocity;
	acc_t acc;
	bool isRCLocked;
} state_t;

typedef struct
{
	Axis3f acc;
	Axis3f gyro;
	short temp;
} sensorData_t;

typedef enum
{
	modeDisable = 0,/*�ر�ģʽ*/
	modeAbs,		/*����ֵģʽ*/
	modeVelocity	/*����ģʽ*/
} mode_e;


typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;

typedef struct
{
	attitude_t attitude;		// deg	
	attitude_t attitudeRate;	// deg/s
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	mode_t mode;
	float thrust;
} setpoint_t;

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
	u8 version;				/*����汾��*/
	pidParam_t pidAngle;	/*�Ƕ�PID*/	
	pidParam_t pidRate;		/*���ٶ�PID*/	
	pidParamPos_t pidPos;	/*λ��PID*/

	float trimP;			/*pitch΢��*/
	float trimR;			/*roll΢��*/
	u16 thrustBase;			/*���Ż���ֵ*/
	u8 cksum;				/*У��*/
} configParam_t;


#define RATE_5_HZ			5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)


#define MAIN_LOOP_RATE 			RATE_1000_HZ
#define MAIN_LOOP_DT			(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//��̬��������
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_5_HZ)

#define POSITION_ESTIMAT_RATE	RATE_250_HZ	//λ��Ԥ������
#define POSITION_ESTIMAT_DT		(1.0/RATE_250_HZ)

#define RATE_PID_RATE			RATE_500_HZ //���ٶȻ����ڻ���PID����
#define RATE_PID_DT				(1.0/RATE_500_HZ)

#define ANGEL_PID_RATE			ATTITUDE_ESTIMAT_RATE //�ǶȻ����⻷��PID����
#define ANGEL_PID_DT			(1.0/ATTITUDE_ESTIMAT_RATE)

#define VELOCITY_PID_RATE		POSITION_ESTIMAT_RATE //�ٶȻ����ڻ���PID����
#define VELOCITY_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define POSITION_PID_RATE		POSITION_ESTIMAT_RATE //λ�û����⻷��PID����
#define POSITION_PID_DT			(1.0/POSITION_ESTIMAT_RATE)

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define MAIN_LOOP_DT	(u32)(1000/MAIN_LOOP_RATE)	/*��λms*/

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

float constrainf(float amt, float low, float high);
void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt);
void imuTransformVectorBodyToEarth(Axis3f * v);
bool getIsCalibrated(void);

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
#endif
