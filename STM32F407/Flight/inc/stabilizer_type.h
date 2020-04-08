#ifndef __STABILIZER_TYPE_H
#define __STABILIZER_TYPE_H

#include "bsp_sys.h"
#include <stdbool.h>

typedef struct  
{
	u32 timestamp;	//时间戳

	float roll;
	float pitch;
	float yaw;
} attitude_t;

struct vec3_s
{
	u32 timestamp;	//时间戳
    
	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;          //位置
typedef struct vec3_s velocity_t;       //速度
typedef struct vec3_s acc_t;            //加速度
#pragma anon_unions
//四元数联合结构体
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
	enum dir_e flipDir;		/*翻滚方向*/
} control_t;

typedef enum
{
    modeDisable = 0,//关闭模式
    modeAbs,        //绝对值模式
    modeVelocity    //速率模式
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
    attitude_t attitude;    //单位deg
    attitude_t attitudeRate;//单位deg/s
    point_t position;       //单位m
    velocity_t velocity;    //单位m/s
    mode_t mode;            //模式
    float thrust;           //油门
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
	uint32_t timestamp;	//时间戳
	float distance;		//测量距离
	float quality;		//可信度
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
