#include "pid.h"

//PID初始化
void pidInit(PidObject* pid, const float desire, const pidInit_t pidParam, const float dt)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;
	pid->desire = desire;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit = DEFAULT_PID_INTEGRAL_LIMIT;
	pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
	pid->dt = dt;
}

//PID更新参数
float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   
	pid->integral += pid->error * pid->dt;
	
	//积分限幅
	if (pid->integral > pid->iLimit)
	{
		pid->integral = pid->iLimit;
	}
	else if (pid->integral < -pid->iLimit)
	{
		pid->integral = -pid->iLimit;
	}

	pid->derivative = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integral;
	pid->outD = pid->kd * pid->derivative;

	output = pid->outP + pid->outI + pid->outD;
	
	//输出限幅
	if (pid->outputLimit != 0)
	{
		if (output > pid->outputLimit)
			output = pid->outputLimit;
		else if (output < -pid->outputLimit)
			output = -pid->outputLimit;
	}
	
	pid->prevError = pid->error;

	pid->out = output;
	return output;
}

//设置积分限幅值
void pidSetIntegralLimit(PidObject* pid, float ilimit)
{
	pid->iLimit = ilimit;
}

//设置输出限幅值
void pidSetOutputLimit(PidObject* pid, const float limit) 
{
	pid->outputLimit = limit;
}
	
//设置偏差值
void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

//设置给定值
void pidSetDesired(PidObject* pid, const float desire)
{
	pid->desire = desire;
}

//获得给定值
float pidGetDesired(PidObject* pid)
{
	return pid->desire;
}

//设置比例值
void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

//设置积分值
void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

//设置微分值
void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

//设置时间增量
void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}

//重设PID参数
void pidReset(PidObject* pid)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;
}
