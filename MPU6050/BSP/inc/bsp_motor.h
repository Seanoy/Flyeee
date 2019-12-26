#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stm32f10x.h>
#include <stdbool.h>

#define ENABLE_THRUST_BAT_COMPENSATED
#define MOTORS_PWM_BITS           	8

#define TIM_CLOCK_HZ 				72000000
#define MOTORS_PWM_BITS           	8
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	0


//#define ENABLE_THRUST_BAT_COMPENSATED	/*使能电池油门补偿*/

#define NBR_OF_MOTORS 	4
#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3

#define MOTORS_TEST_RATIO         (u16)(0.2*(1<<16))	//20%
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

enum dir_e
{
	CENTER=0,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
};

typedef struct 
{
	u32 m1;
	u32 m2;
	u32 m3;
	u32 m4;
	
}motorPWM_t;

typedef struct
{
	s16 roll;
	s16 pitch;
	s16 yaw;
	float thrust;
	enum dir_e flipDir;		/*翻滚方向*/
} control_t;

void getMotorPWM(motorPWM_t* get);
void setMotorPWM(bool enable, u32 m1_set, u32 m2_set, u32 m3_set, u32 m4_set);
void powerControl(control_t *control);	/*功率输出控制*/
u16 limitThrust(int value);
bool powerControlTest(void);
void powerControlInit(void);
/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust);

void motorsInit(void);	/*电机初始化*/

#endif
