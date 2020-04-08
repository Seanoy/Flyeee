#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H

#include "bsp_sys.h"
#include "pid.h"

//λ��PID
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
    uint8_t version;        //�汾��
    pidParam_t pidAngle;    //�Ƕ�PID
    pidParam_t pidRate;     //���ٶ�PID
    pidParamPos_t pidPos;   //λ��PID
    float trimP;            //pitch΢��ֵ
    float trimR;            //roll΢��ֵ
    uint16_t thrustBase;    //���Ż���ֵ
    uint8_t checksum;       //У���
}configParam_t;

extern configParam_t configParam;
extern configParam_t configParamDefault;
#endif
