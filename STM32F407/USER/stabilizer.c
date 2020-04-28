#include "stabilizer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors.h"
#include "stabilizer_type.h"
#include "imu.h"
#include "communication.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"

u32 getSysTickCnt(void);

static sensorData_t sensorData;	/*����������*/
static state_t state;           /*������̬*/
static setpoint_t setpoint;	    /*����Ŀ��״̬*/
static control_t control;	    /*������Ʋ���*/
static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick)
{
    //�ǶȻ����⻷��
	if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick))
	{
        actualThrust = setpoint->thrust;

        attitudeDesired.roll = setpoint->attitude.roll;
        attitudeDesired.pitch = setpoint->attitude.pitch;
    
        attitudeDesired.yaw += setpoint->attitude.yaw/ANGEL_PID_RATE; /*����YAW ����ģʽ*/
        if(attitudeDesired.yaw > 180.0f) 
            attitudeDesired.yaw -= 360.0f;
        if(attitudeDesired.yaw < -180.0f) 
            attitudeDesired.yaw += 360.0f;
		
		attitudeDesired.roll += configParam.trimR;	//����΢��ֵ
		attitudeDesired.pitch += configParam.trimP;		
		
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
	}
	
	//���ٶȻ����ڻ���
	if (RATE_DO_EXECUTE(RATE_PID_RATE, tick))
	{
        rateDesired.roll = setpoint->attitudeRate.roll;
        rateDesired.pitch = setpoint->attitudeRate.pitch;
		attitudeRatePID(&sensors->gyro, &rateDesired, control);
	}

	control->thrust = actualThrust;	
}

//����ң����Ϣ���ò���
void commanderGetSetpoint(setpoint_t *setpoint, nrf_ctrl_t *nrf_ctrl)
{
    //�����Ȱ���ƽ��������
//    setpoint->attitude.pitch = 0;
//    setpoint->attitude.roll = nrf_ctrl->roll;
//    setpoint->attitude.yaw = nrf_ctrl->yaw;
//    setpoint->thrust = nrf_ctrl->thrust;
    
    setpoint->attitude.pitch = nrf_ctrl->pitch;
    setpoint->attitude.roll = nrf_ctrl->roll;
    setpoint->attitude.yaw = nrf_ctrl->yaw;
    setpoint->thrust = nrf_ctrl->thrust;
}

void stabilize_task(void *pvParameters)
{
    u32 tick = 0;
    u32 lastWakeTime = getSysTickCnt();

    attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT);         /*��ʼ����̬PID*/	

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms������ʱ*/
        
		//��ȡ6�����ѹ���ݣ�500Hz��
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);				/*��ȡ9�����ѹ����*/
		}

		//��Ԫ����ŷ���Ǽ��㣨250Hz��
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
//			agmImuUpdate(sensorData.acc, sensorData.gyro, sensorData.mag, &state, ATTITUDE_ESTIMAT_DT);
          imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
//            AHRSupdateIMUMadgwick(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
            ANO_Send_03(state.attitude.roll*100, state.attitude.pitch*100, state.attitude.yaw*100, 0);
		}
        
//		//λ��Ԥ�����㣨250Hz���жϸ߶�
//		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
//		{
//			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
//		}
        
        //Ŀ����̬�ͷ���ģʽ�趨��100Hz��	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			commanderGetSetpoint(&setpoint, &nrf_ctrl);	/*Ŀ�����ݺͷ���ģʽ�趨*/	
		}
        
        stateControl(&control, &sensorData, &state, &setpoint, tick);
        
		//���Ƶ�������500Hz��
        control.thrust = 20000;
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);	
		}
		
        tick++;
    }
}



