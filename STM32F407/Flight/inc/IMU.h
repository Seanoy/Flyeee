#ifndef __IMU_H
#define __IMU_H

#include "bsp.h"
#include "math.h"

typedef struct float_angle{
				float Roll;
				float Pitch;
				float Yaw;}S_FLOAT_ANGLE;

extern S_FLOAT_ANGLE Q_ANGLE;
                
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void AGMIMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz);

#endif
