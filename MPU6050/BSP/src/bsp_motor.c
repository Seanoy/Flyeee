#include "bsp_motor.h"

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet={0, 0, 0, 0};

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
static const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };


static u16 ratioToCCRx(u16 val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void motorsInit(void)	/*�����ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��PORTAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2��TIM4ʱ��ʹ��    
	
	TIM_DeInit(TIM2);	//���³�ʼ��TIM2ΪĬ��״̬
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;	//PA0 1 2 3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;        				
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;				//�ٶ�100MHz

	GPIO_Init(GPIOA,&GPIO_InitStructure);              				
	
	TIM_TimeBaseStructure.TIM_Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_ClockDivision=0; 					//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;				//�ظ���������
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.TIM_Pulse=0;							//CCRx
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;	//���иߵ�ƽ	
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH2����Ƚ�
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH4����Ƚ�
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���

	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPEʹ�� 

	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	

	isInit = true;
}

/*���õ��PWMռ�ձ�*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare2(TIM4,ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare1(TIM4,ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM2,ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM_SetCompare1(TIM2,ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}


void powerControlInit(void)
{
	motorsInit();
}


u16 limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

void powerControl(control_t *control)	/*�����������*/
{
	s16 r = control->roll / 2.0f;
	s16 p = control->pitch / 2.0f;
	
	motorPWM.m1 = limitThrust(control->thrust - r - p + control->yaw);
	motorPWM.m2 = limitThrust(control->thrust - r + p - control->yaw);
	motorPWM.m3 = limitThrust(control->thrust + r + p + control->yaw);
	motorPWM.m4 = limitThrust(control->thrust + r - p - control->yaw);		

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(MOTOR_M1, motorPWM.m1);	/*���Ƶ������ٷֱ�*/
	motorsSetRatio(MOTOR_M2, motorPWM.m2);
	motorsSetRatio(MOTOR_M3, motorPWM.m3);
	motorsSetRatio(MOTOR_M4, motorPWM.m4);
}

void getMotorPWM(motorPWM_t* get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u32 m1_set, u32 m2_set, u32 m3_set, u32 m4_set)
{
	motorSetEnable = enable;
	motorPWMSet.m1 = m1_set;
	motorPWMSet.m2 = m2_set;
	motorPWMSet.m3 = m3_set;	
	motorPWMSet.m4 = m4_set;
}
