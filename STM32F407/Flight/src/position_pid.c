#include "position_pid.h"


#define THRUST_BASE             (20000)     //��������ֵ

#define PID_OUTPUT_LIMIT_VX     120.0f      //ROLL�޷� ��0.15ϵ��
#define PID_OUTPUT_LIMIT_VY     120.0f      //PITCH�޷� ��0.15ϵ��
#define PID_OUTPUT_LIMIT_VZ     (40000)     //YAW�޷�
#define PID_OUTPUT_LIMIT_X      1200.0f     //X���ٶ��޷�
#define PID_OUTPUT_LIMIT_Y      1200.0f     //Y���ٶ��޷�
#define PID_OUTPUT_LIMIT_Z      120.0f      //Z���ٶ��޷�

static float thrustLpf = THRUST_BASE;       //���ŵ�ͨ�˲���

pidObject_t pidVX;
pidObject_t pidVY;
pidObject_t pidVZ;

pidObject_t pidX;
pidObject_t pidY;
pidObject_t pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
    pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);   //vx pid init
    pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);   //vy pid init
    pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);   //vz pid init
    pidSetOutputLimit(&pidVX, PID_OUTPUT_LIMIT_VX);             //output limit
    pidSetOutputLimit(&pidVY, PID_OUTPUT_LIMIT_VY);             //output limit
    pidSetOutputLimit(&pidVZ, PID_OUTPUT_LIMIT_VZ);             //output limit

    pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);          //x pid init
    pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);          //y pid init
    pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);          //z pid init
    pidSetOutputLimit(&pidX, PID_OUTPUT_LIMIT_X);               //output limit
    pidSetOutputLimit(&pidY, PID_OUTPUT_LIMIT_Y);               //output limit
    pidSetOutputLimit(&pidZ, PID_OUTPUT_LIMIT_Z);               //output limit
}

void velocityController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)
{
    static uint16_t altholdCount = 0;
    
    //Roll and Pitch
    attitude->roll = (float)0.15*pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
    attitude->pitch = (float)0.15*pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

    //thrust
    float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);
    
//    *thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 60000);
    
    thrustLpf += (*thrust - thrustLpf) * 0.003f;
    
//    if(getCommanderKeyFlight())//����ģʽ
	{
		if(fabs(state->acc.z) < 35.f)
		{
			altholdCount++;
			if(altholdCount > 1000)
			{
				altholdCount = 0;
				if(fabs(configParam.thrustBase - thrustLpf) > 1000.f)	/*���»�������ֵ*/
					configParam.thrustBase = thrustLpf;
			}
		}else
		{
			altholdCount = 0;
		}
	}
//    else if(getCommanderKeyland() == false)	/*������ɣ���������*/
//	{
//		*thrust = 0;
//	}
}

void positionController()
{
    
}



