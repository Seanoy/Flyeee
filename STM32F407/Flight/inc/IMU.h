#ifndef __IMU_H
#define __IMU_H

#include "bsp.h"
#include "math.h"

typedef struct float_angle{
    float Roll;
    float Pitch;
    float Yaw;
}S_FLOAT_ANGLE;


                
void imuUpdate(axis3f_t acc, axis3f_t gyro, state_t *state, float dt);
void agmImuUpdate(axis3f_t acc, axis3f_t gyro, axis3f_t mag, state_t *state, float dt);

#endif
