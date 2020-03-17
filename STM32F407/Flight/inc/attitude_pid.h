#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include "pid.h"
#include <math.h>
#include "config_param.h"
#include "sensors.h"
#include "stabilizer_type.h"



//角度环pid
extern pidObject_t pidAngleRoll;
extern pidObject_t pidAnglePitch;
extern pidObject_t pidAngleYaw;
//角速度环pid
extern pidObject_t pidRateRoll;
extern pidObject_t pidRatePitch;
extern pidObject_t pidRateYaw;

void attitudeControlInit(float anglePidDt, float ratePidDt);

void attitudeRatePID(axis3f_t *actualRate, attitude_t *desireRate, control_t *output);
void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desireAngle, attitude_t *outDesiredRate);
void attitudeControllerResetAttitudePIDRoll(void);
void attitudeControllerResetAttitudePIDPitch(void);
void attitudeResetAllPID(void);
void attitudeConfigParam(void);

#endif
