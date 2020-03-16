#ifndef __PID_H
#define __PID_H

#include "bsp_ak8963.h"
#include "sensor_type.h"

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
}pidObject_t;

//PID Init structure
typedef struct
{
    float kp;           //p coefficient
    float ki;           //i coefficient
    float kd;           //d coefficient
}pidInit_t;

//PID position structure
typedef struct
{
    pidInit_t roll;
    pidInit_t pitch;
    pidInit_t yaw;
}pidParam_t;

void pidInit(pidObject_t *pid, const float desire, const pidInit_t pidParam, const float dt);
float pidUpdate(pidObject_t *pid, const float error);
void pidSetIntegralLimit(pidObject_t* pid, const float ilimit); 
void pidSetError(pidObject_t* pid, const float error); 
void pidSetOutputLimit(pidObject_t* pid, const float outLimit); 
void pidSetDesire(pidObject_t* pid, const float desire); 
float pidGetDesire(pidObject_t* pid); 
void pidSetKp(pidObject_t* pid, const float kp); 
void pidSetKi(pidObject_t* pid, const float ki); 
void pidSetKd(pidObject_t* pid, const float kd); 
void pidSetDt(pidObject_t* pid, const float dt); 
void pidReset(pidObject_t* pid); 

#endif
