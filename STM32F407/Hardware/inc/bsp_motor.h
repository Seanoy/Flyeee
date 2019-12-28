#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stm32f4xx.h>
#include <stdbool.h>
//


#define TIM_CLOCK_HZ 				168000000
#define MOTORS_PWM_BITS           	16
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	0


#define MOTOR_M1 0
#define MOTOR_M2 1
#define MOTOR_M3 2
#define MOTOR_M4 3

#define MOTORS_TEST_RATIO         (u16)(0.2*(1<<16))	//20%
#define MOTORS_TEST_ON_TIME_MS    300
#define MOTORS_TEST_DELAY_TIME_MS 500


void MOTOR_TIM_Init(void);
bool motorsTest(void);		/*电机测试*/
void motorsSetRatio(u32 id, u16 ithrust);	/*设置电机占空比*/

#endif
