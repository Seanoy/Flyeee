#include "attitude_pid.h"


//角度环积分限幅
#define PID_ANGLE_INTEGRATION_LIMIT_ROLL    30.0
#define PID_ANGLE_INTEGRATION_LIMIT_PITCH   30.0
#define PID_ANGLE_INTEGRATION_LIMIT_YAW     180.0


//角速度环积分限幅
#define PID_RATE_INTEGRATION_LIMIT_ROLL     500.0
#define PID_RATE_INTEGRATION_LIMIT_PITCH    500.0
#define PID_RATE_INTEGRATION_LIMIT_YAW      500.0

//角度环pid
pidObject_t pidAngleRoll;
pidObject_t pidAnglePitch;
pidObject_t pidAngleYaw;
//角速度环pid
pidObject_t pidRateRoll;
pidObject_t pidRatePitch;
pidObject_t pidRateYaw;

//积分限幅
static inline int16_t pidOutLimit(float in)
{
    if(in>INT16_MAX)
        return INT16_MAX;//限制上限
    else if(in<-INT16_MAX)
        return -INT16_MAX;//限制下限
    else
        return (int16_t)in;//float转换为short
}

//姿态控制初始化
/*
anglePidDt 角度pid的变化频率
ratePidDt 角速度pid的变化频率
*/
void attitudeControlInit(float anglePidDt, float ratePidDt)
{
    //初始化角度三轴的PID参数以及积分限幅值
    pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);
    pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);
    pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);
    pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_INTEGRATION_LIMIT_ROLL);
    pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_INTEGRATION_LIMIT_PITCH);
    pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_INTEGRATION_LIMIT_YAW);
    
    //初始化角速度三轴的PID参数以及积分限幅值
    pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);
    pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);
    pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);
    pidSetIntegralLimit(&pidRateRoll, PID_RATE_INTEGRATION_LIMIT_ROLL);
    pidSetIntegralLimit(&pidRatePitch, PID_RATE_INTEGRATION_LIMIT_PITCH);
    pidSetIntegralLimit(&pidRateYaw, PID_RATE_INTEGRATION_LIMIT_YAW);
}

//角速度环PID 内环
void attitudeRatePID(axis3f_t *actualRate, attitude_t *desireRate, control_t *output)
{
    output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desireRate->roll - actualRate->x));
    output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desireRate->pitch - actualRate->y));
    output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desireRate->yaw - actualRate->z));
}

//角度环PID 外环
void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desireAngle, attitude_t *outDesiredRate)
{
    outDesiredRate->roll = pidUpdate(&pidAngleRoll, desireAngle->roll - outDesiredRate->roll);
    outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desireAngle->pitch - outDesiredRate->pitch);
    
    //数值转换，yaw数据范围(180~-180)比较特殊 
    float yawError = desireAngle->yaw - actualAngle->yaw;
    if(yawError > 180.0f)
        yawError -= 360.0f;
    else if(yawError < -180.0f)
        yawError += 360.0f;
    outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

//复位roll姿态控制结构体
void attitudeControllerResetAttitudePIDRoll(void)
{
    pidReset(&pidAngleRoll);
}

//复位pitch姿态控制结构体
void attitudeControllerResetAttitudePIDPitch(void)
{
    pidReset(&pidAnglePitch);
}

//复位所有PID
void attitudeResetAllPID(void)
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

//配置参数结构体
void attitudeConfigParam(void)
{
    //角度环PID配置
    configParam.pidAngle.roll.kp = pidAngleRoll.kp;
    configParam.pidAngle.roll.ki = pidAngleRoll.ki;
    configParam.pidAngle.roll.kd = pidAngleRoll.kd;
    
    configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
    configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
    configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
    
    configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
    configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
    configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
    
    //角速度环PID配置
    configParam.pidRate.roll.kp = pidRateRoll.kp;
    configParam.pidRate.roll.ki = pidRateRoll.ki;
    configParam.pidRate.roll.kd = pidRateRoll.kd;
    
    configParam.pidRate.pitch.kp = pidRatePitch.kp;
    configParam.pidRate.pitch.ki = pidRatePitch.ki;
    configParam.pidRate.pitch.kd = pidRatePitch.kd;
    
    configParam.pidRate.yaw.kp = pidRateYaw.kp;
    configParam.pidRate.yaw.ki = pidRateYaw.ki;
    configParam.pidRate.yaw.kd = pidRateYaw.kd;
}
