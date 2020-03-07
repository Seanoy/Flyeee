#ifndef __FLIGHT_PID_H
#define __FLIGHT_PID_H

#include "bsp_ak8963.h"
#include "bsp_mpu9250.h"

#define DEFAULT_PID_INTEGRAL_LIMIT  500.0//default integral limit
#define DEFAULT_PID_OUTPUT_LIMIT 0.0//default do not limit pid output


//PID struct
typedef struct 
{
    float desire;       //desire value
    float error;        //error value
    float prevError;    //previous error
    float integral;     //integral value
    float derivative;   //derivative value
    float kp;           //p coefficient
    float ki;           //i coefficient
    float kd;           //d coefficient
    float iLimit;       //limit integral
    float outLimit;     //output limit value
    float outP;         //P output value
    float outI;         //I output value
    float outD;         //D output value
    float dt;           //delta time
    float out;          //total output value
}PidObject_t;

//PID Init structure
typedef struct
{
    float kp;           //p coefficient
    float ki;           //i coefficient
    float kd;           //d coefficient
}PidInit_t;

//PID position structure
typedef struct
{
    PidInit_t roll;
    PidInit_t pitch;
    PidInit_t raw;
}PidParam_t;



void PID_Init(PidObject_t *pid, const float desire, const PidInit_t pidParam, const float dt);
float PID_Update(PidObject_t *pid, const float error);


#endif
