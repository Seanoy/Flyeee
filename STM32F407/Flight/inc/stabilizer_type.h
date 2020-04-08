#ifndef __STABILIZER_TYPE_H
#define __STABILIZER_TYPE_H

#include "bsp_sys.h"
#include <stdbool.h>

typedef struct  
{
	u32 timestamp;	//ʱ���

	float roll;
	float pitch;
	float yaw;
} attitude_t;

struct vec3_s
{
	u32 timestamp;	//ʱ���
    
	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;          //λ��
typedef struct vec3_s velocity_t;       //�ٶ�
typedef struct vec3_s acc_t;            //���ٶ�
#pragma anon_unions
//��Ԫ�����Ͻṹ��
typedef struct
{
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
}quaternion_t;

typedef struct
{
    attitude_t attitude;
    quaternion_t attitudeQuaternion;
    point_t position;
    velocity_t velocity;
    acc_t acc;
    bool isRCLocked;
}state_t;

enum dir_e
{
	CENTER=0,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
};

typedef struct
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	float thrust;
	enum dir_e flipDir;		/*��������*/
} control_t;

typedef enum
{
    modeDisable = 0,//�ر�ģʽ
    modeAbs,        //����ֵģʽ
    modeVelocity    //����ģʽ
}mode_e;

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
    attitude_t attitude;    //��λdeg
    attitude_t attitudeRate;//��λdeg/s
    point_t position;       //��λm
    velocity_t velocity;    //��λm/s
    mode_t mode;            //ģʽ
    float thrust;           //����
}setpoint_t;
//3axis sensor type
typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
}axis3f_t;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef struct 
{
	uint32_t timestamp;	//ʱ���
	float distance;		//��������
	float quality;		//���Ŷ�
} zRange_t;

typedef struct
{
	axis3f_t acc;
	axis3f_t gyro;
	axis3f_t mag;
	baro_t baro;
	point_t position;
	zRange_t zrange;
} sensorData_t;

#endif
