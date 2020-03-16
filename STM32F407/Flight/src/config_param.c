#include "config_param.h"

//该文件用于配置pid和版本等信息

#define VERSION 10

configParam_t configParam;

static configParam_t configParamDefault=
{
    .version = VERSION,
    .pidAngle=
    {
        .pitch=
        {
            .kp=8.0,
            .ki=0.0,
            .kd=0.0,
        },
        .roll=
        {
            .kp=8.0,
            .ki=0.0,
            .kd=0.0,
        },
        .yaw=
        {
            .kp=20.0,
            .ki=0.0,
            .kd=0.0,
        },
    },
    .pidRate=
    {
        .pitch=
        {
            .kp=300.0,
            .ki=0.0,
            .kd=6.5,
        },
        .roll=
        {
            .kp=300.0,
            .ki=0.0,
            .kd=6.5,
        },
        .yaw=
        {
            .kp=200.0,
            .ki=18.5,
            .kd=0.0,
        },
    },
    .pidPos=
    {
        .vx=
        {
            .kp=4.5,
            .ki=0.0,
            .kd=0.0,
        },
        .vy=
        {
            .kp=4.5,
            .ki=0.0,
            .kd=0.0,
        },
        .vz=
        {
            .kp=100.0,
            .ki=150.0,
            .kd=10.0,
        },
        .x=
        {
            .kp=4.0,
            .ki=0.0,
            .kd=0.6,
        },
        .y=
        {
            .kp=4.0,
            .ki=0.0,
            .kd=0.6,
        },
        .z=
        {
            .kp=6.0,
            .ki=0.0,
            .kd=4.5,
        },
    },
    .trimPitch=0.0f,     //pitch微调值
    .trimRoll=0.0f,     //roll微调值
    .thrustBase=34000,  //定高油门基础值
};

