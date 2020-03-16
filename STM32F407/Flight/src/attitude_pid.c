#include "attitude_pid.h"


//�ǶȻ������޷�
#define PID_ANGLE_INTEGRATION_LIMIT_ROLL    30.0
#define PID_ANGLE_INTEGRATION_LIMIT_PITCH   30.0
#define PID_ANGLE_INTEGRATION_LIMIT_YAW     180.0


//���ٶȻ������޷�
#define PID_RATE_INTEGRATION_LIMIT_ROLL     500.0
#define PID_RATE_INTEGRATION_LIMIT_PITCH    500.0
#define PID_RATE_INTEGRATION_LIMIT_YAW      500.0

//�ǶȻ�pid
pidObject_t pidAngleRoll;
pidObject_t pidAnglePitch;
pidObject_t pidAngleYaw;
//���ٶȻ�pid
pidObject_t pidRateRoll;
pidObject_t pidRatePitch;
pidObject_t pidRateYaw;

//�����޷�
static inline int16_t pidOutLimit(float in)
{
    if(in>INT16_MAX)
        return INT16_MAX;//��������
    else if(in<-INT16_MAX)
        return -INT16_MAX;//��������
    else
        return (int16_t)in;//floatת��Ϊshort
}

//��̬���Ƴ�ʼ��
/*
anglePidDt �Ƕ�pid�ı仯Ƶ��
ratePidDt ���ٶ�pid�ı仯Ƶ��
*/
void attitudeControlInit(float anglePidDt, float ratePidDt)
{
    //��ʼ���Ƕ������PID�����Լ������޷�ֵ
    pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);
    pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);
    pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);
    pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_INTEGRATION_LIMIT_ROLL);
    pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_INTEGRATION_LIMIT_PITCH);
    pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_INTEGRATION_LIMIT_YAW);
    
    //��ʼ�����ٶ������PID�����Լ������޷�ֵ
    pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);
    pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);
    pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);
    pidSetIntegralLimit(&pidRateRoll, PID_RATE_INTEGRATION_LIMIT_ROLL);
    pidSetIntegralLimit(&pidRatePitch, PID_RATE_INTEGRATION_LIMIT_PITCH);
    pidSetIntegralLimit(&pidRateYaw, PID_RATE_INTEGRATION_LIMIT_YAW);
}

//���ٶȻ�PID �ڻ�
void attitudeRatePID(axis3f_t *actualRate, attitude_t *desireRate, control_t *output)
{
    output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desireRate->roll - actualRate->x));
    output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desireRate->pitch - actualRate->y));
    output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desireRate->yaw - actualRate->z));
}

//�ǶȻ�PID �⻷
void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desireAngle, attitude_t *outDesiredRate)
{
    outDesiredRate->roll = pidUpdate(&pidAngleRoll, desireAngle->roll - outDesiredRate->roll);
    outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desireAngle->pitch - outDesiredRate->pitch);
    
    //��ֵת����yaw���ݷ�Χ(180~-180)�Ƚ����� 
    float yawError = desireAngle->yaw - actualAngle->yaw;
    if(yawError > 180.0f)
        yawError -= 360.0f;
    else if(yawError < -180.0f)
        yawError += 360.0f;
    outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

//��λroll��̬���ƽṹ��
void attitudeControllerResetAttitudePIDRoll(void)
{
    pidReset(&pidAngleRoll);
}

//��λpitch��̬���ƽṹ��
void attitudeControllerResetAttitudePIDPitch(void)
{
    pidReset(&pidAnglePitch);
}

//��λ����PID
void attitudeResetAllPID(void)
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

//���ò����ṹ��
void attitudeConfigParam(void)
{
    //�ǶȻ�PID����
    configParam.pidAngle.roll.kp = pidAngleRoll.kp;
    configParam.pidAngle.roll.ki = pidAngleRoll.ki;
    configParam.pidAngle.roll.kd = pidAngleRoll.kd;
    
    configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
    configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
    configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
    
    configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
    configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
    configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
    
    //���ٶȻ�PID����
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
