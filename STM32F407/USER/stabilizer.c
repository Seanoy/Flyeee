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

static sensorData_t sensorData;	/*传感器数据*/
static state_t state;           /*四轴姿态*/
static setpoint_t setpoint;	    /*设置目标状态*/
static control_t control;	    /*四轴控制参数*/
static float actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick)
{
    //角度环（外环）
	if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick))
	{
        actualThrust = setpoint->thrust;

        attitudeDesired.roll = setpoint->attitude.roll;
        attitudeDesired.pitch = setpoint->attitude.pitch;
    
        attitudeDesired.yaw += setpoint->attitude.yaw/ANGEL_PID_RATE; /*期望YAW 速率模式*/
        if(attitudeDesired.yaw > 180.0f) 
            attitudeDesired.yaw -= 360.0f;
        if(attitudeDesired.yaw < -180.0f) 
            attitudeDesired.yaw += 360.0f;
		
		attitudeDesired.roll += configParam.trimR;	//叠加微调值
		attitudeDesired.pitch += configParam.trimP;		
		
		attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
	}
	
	//角速度环（内环）
	if (RATE_DO_EXECUTE(RATE_PID_RATE, tick))
	{
        rateDesired.roll = setpoint->attitudeRate.roll;
        rateDesired.pitch = setpoint->attitudeRate.pitch;
		attitudeRatePID(&sensors->gyro, &rateDesired, control);
	}

	control->thrust = actualThrust;	
}

//根据遥控信息配置参数
void commanderGetSetpoint(setpoint_t *setpoint, nrf_ctrl_t *nrf_ctrl)
{
    //这里先按照平衡条件来
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

    attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT);         /*初始化姿态PID*/	

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms周期延时*/
        
		//获取6轴和气压数据（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);				/*获取9轴和气压数据*/
		}

		//四元数和欧拉角计算（250Hz）
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
//			agmImuUpdate(sensorData.acc, sensorData.gyro, sensorData.mag, &state, ATTITUDE_ESTIMAT_DT);
          imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
//            AHRSupdateIMUMadgwick(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
            ANO_Send_03(state.attitude.roll*100, state.attitude.pitch*100, state.attitude.yaw*100, 0);
		}
        
//		//位置预估计算（250Hz）判断高度
//		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
//		{
//			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
//		}
        
        //目标姿态和飞行模式设定（100Hz）	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			commanderGetSetpoint(&setpoint, &nrf_ctrl);	/*目标数据和飞行模式设定*/	
		}
        
        stateControl(&control, &sensorData, &state, &setpoint, tick);
        
		//控制电机输出（500Hz）
        control.thrust = 20000;
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerControl(&control);	
		}
		
        tick++;
    }
}



