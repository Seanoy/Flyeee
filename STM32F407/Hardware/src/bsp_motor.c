#include "bsp_motor.h"

//MOTOR 1	PE5		TIM9_CH1
//MOTOR 2	PA8		TIM1_CH1
//MOTOR 3	PD12	TIM4_CH1
//MOTOR 4	PA1		TIM5_CH2

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
static const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

static u16 ratioToCCRx(u16 val)
{
	return ((val) >> (32 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

void MOTOR_TIM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_DeInit(TIM1);	
    TIM_DeInit(TIM4);	
    TIM_DeInit(TIM5);	
    TIM_DeInit(TIM9);	

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM1,ENABLE);//max 84MHz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5,ENABLE);//max 42MHz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE,ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOE,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);	
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);	


    TIM_TimeBaseInitStructure.TIM_Prescaler = 168-1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000-1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);

    TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;			//空闲高电平	
    TIM_OCInitStructure.TIM_Pulse = 500;

    TIM_OC1Init(TIM9, &TIM_OCInitStructure);  										//TIM9_CH1
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM9在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM9,ENABLE);
    TIM_Cmd(TIM9, ENABLE);  //使能TIM9

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  										//TIM1_CH1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM1,ENABLE);
    TIM_Cmd(TIM1, ENABLE);  //使能TIM1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);	

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  										//TIM4_CH1
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM4,ENABLE);
    TIM_Cmd(TIM4, ENABLE);  //使能TIM4

    TIM_OC2Init(TIM5, &TIM_OCInitStructure);  										//TIM5_CH2
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM5在CCR2上的预装载寄存器
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);  //使能TIM5
}



/*电机测试*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], 500);
//		delay_ms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
//		delay_ms(MOTORS_TEST_ON_TIME_MS);
	}

	return isInit;
}

/*设置电机PWM占空比*/
void motorsSetRatio(u32 id, u16 ithrust)
{
    switch(id)
    {
        case 0:		/*MOTOR_M1*/
            TIM_SetCompare1(TIM1,500);
            break;
        case 1:		/*MOTOR_M2*/
            TIM_SetCompare1(TIM4,500);
            break;
        case 2:		/*MOTOR_M3*/
            TIM_SetCompare1(TIM9,500);
            break;
        case 3:		/*MOTOR_M4*/	
            TIM_SetCompare2(TIM5,500);
            break;
        default: break;
    }
}
