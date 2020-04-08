#ifndef __SENSORS_H
#define __SENSORS_H

#include <stdbool.h>
#include "bsp_sys.h"
#include "stabilizer_type.h"



void Filter_Init(void);
void sensorInit(void);
void processSensordata(void);
/*从队列读取陀螺数据*/
bool sensorsReadGyro(axis3f_t *gyro);
bool sensorsReadAcc(axis3f_t *acc);
bool sensorsReadMag(axis3f_t *mag);
bool sensorsReadBaro(baro_t *baro);
void sensorsAcquire(sensorData_t *sensors, const u32 tick);

#endif
