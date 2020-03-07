#include "flight_pid.h"

//init pid object struct
void PID_Init(PidObject_t *pid, const float desire, const PidInit_t pidParam, const float dt)
{
    pid->prevError  = 0;
    pid->error      = 0;
    pid->integral   = 0;
    pid->derivative = 0;
    pid->iLimit     = DEFAULT_PID_INTEGRAL_LIMIT;
    pid->outLimit   = DEFAULT_PID_OUTPUT_LIMIT;
    pid->desire     = desire;
    pid->kp         = pidParam.kp;
    pid->ki         = pidParam.ki;
    pid->kd         = pidParam.kd;
    pid->dt         = dt;
}

//update pid object by error value
float PID_Update(PidObject_t *pid, const float error)
{
    float output;//total output value
    
    pid->error = error;
    pid->integral += pid->error * pid->dt;//calculate integral value
    
    //integration limit
    if(pid->integral > pid->iLimit)
        pid->integral = pid->iLimit;
    else if(pid->integral < -pid->iLimit)
        pid->integral = -pid->iLimit;
    
    //difference of error and prevError
    pid->derivative = (pid->error - pid->prevError) * pid->dt;
    
    //calculate all parts output value
    pid->outP = pid->kp * pid->error;
    pid->outI = pid->ki * pid->integral;
    pid->outD = pid->kd * pid->derivative;
    
    output = pid->outP + pid->outI + pid->outD;
    
    //output limit
    if(pid->outLimit != 0)
    {
        if(output > pid->outLimit)
            output = pid->outLimit;
        else if(output < -pid->outLimit)
            output = -pid->outLimit;
    }
    
    pid->prevError = error;
    pid->out = output;
    return output;
}







