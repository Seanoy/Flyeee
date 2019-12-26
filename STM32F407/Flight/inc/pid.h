#ifndef __PID_H
#define __PID_H
#include <stdbool.h>
#include "config.h"
#define DEFAULT_PID_INTEGRAL_LIMIT 		500.0 //默认pid的积分限幅
#define DEFAULT_PID_OUTPUT_LIMIT      0.0	  //默认pid输出限幅，0为不限幅

typedef struct
{
	float desire;			//设定值
	float error;			//偏差值
	float prevError;	//前一次偏差值
	float integral;		//积分值
	float derivative;	//导数
	float kp;					//比例
	float ki;					//积分
	float kd;					//微分
	float outP;				//比例输出
	float outI;				//积分输出
	float outD;				//微分输出
	float iLimit;			//积分限幅值
	float outputLimit;//输出限幅值
	float dt;					//时间增量
	float out;				//输出值
}PidObject;

/*pid结构体初始化*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid积分限幅设置*/
void pidSetOutputLimit(PidObject* pid, const float limit);
void pidSetDesired(PidObject* pid, const float desired);	/*pid设置期望值*/
float pidUpdate(PidObject* pid, const float error);			/*pid更新*/
float pidGetDesired(PidObject* pid);	/*pid获取期望值*/
bool pidIsActive(PidObject* pid);		/*pid状态*/
void pidReset(PidObject* pid);			/*pid结构体复位*/
void pidSetError(PidObject* pid, const float error);/*pid偏差设置*/
void pidSetKp(PidObject* pid, const float kp);		/*pid Kp设置*/
void pidSetKi(PidObject* pid, const float ki);		/*pid Ki设置*/
void pidSetKd(PidObject* pid, const float kd);		/*pid Kd设置*/
void pidSetDt(PidObject* pid, const float dt);		/*pid dt设置*/

#endif
