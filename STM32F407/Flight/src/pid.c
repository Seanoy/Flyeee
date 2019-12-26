#include "pid.h"

//PID��ʼ��
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

//PID���²���
float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   
	pid->integral += pid->error * pid->dt;
	
	//�����޷�
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
	
	//����޷�
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

//���û����޷�ֵ
void pidSetIntegralLimit(PidObject* pid, float ilimit)
{
	pid->iLimit = ilimit;
}

//��������޷�ֵ
void pidSetOutputLimit(PidObject* pid, const float limit) 
{
	pid->outputLimit = limit;
}
	
//����ƫ��ֵ
void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

//���ø���ֵ
void pidSetDesired(PidObject* pid, const float desire)
{
	pid->desire = desire;
}

//��ø���ֵ
float pidGetDesired(PidObject* pid)
{
	return pid->desire;
}

//���ñ���ֵ
void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

//���û���ֵ
void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

//����΢��ֵ
void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

//����ʱ������
void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}

//����PID����
void pidReset(PidObject* pid)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integral = 0;
	pid->derivative = 0;
}
