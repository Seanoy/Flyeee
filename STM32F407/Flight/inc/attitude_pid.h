#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H

#include <stdbool.h>
#include "pid.h"
#include <stdint.h>

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*���ٶȻ������޷�*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		50.0




#endif
