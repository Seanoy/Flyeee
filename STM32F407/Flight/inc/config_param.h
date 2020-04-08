#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H

#include "bsp_sys.h"
#include "pid.h"

//位置PID
typedef struct
{
    pidInit_t vx;
    pidInit_t vy;
    pidInit_t vz;

    pidInit_t x;
    pidInit_t y;
    pidInit_t z;
}pidParamPos_t;

typedef struct
{
    uint8_t version;        //版本号
    pidParam_t pidAngle;    //角度PID
    pidParam_t pidRate;     //角速度PID
    pidParamPos_t pidPos;   //位置PID
    float trimP;            //pitch微调值
    float trimR;            //roll微调值
    uint16_t thrustBase;    //油门基础值
    uint8_t checksum;       //校验和
}configParam_t;

extern configParam_t configParam;
extern configParam_t configParamDefault;
#endif
