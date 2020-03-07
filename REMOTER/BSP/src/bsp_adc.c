#include "bsp_adc.h"

/*
J1 
X: ADC12_IN3 PA3 
Y: ADC12_IN4 PA4
SW:PA11

J2
X: ADC12_IN0 PA0
Y: ADC12_IN1 PA1
SW:PA2
*/ 
uint16_t ADC_value[CHANNEL_NUM*SAMPLE_TIME];
uint32_t adc_x1, adc_y1, adc_x2, adc_y2;
signed char coordinate[4];//{x, y} 0~100

void Joystick_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMAʱ��
 
    DMA_DeInit(DMA1_Channel1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ��ADC1����DMAͨ��1
    DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA����ADC����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_value;      //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //�ڴ���Ϊ���ݴ����Ŀ�ĵ�
    DMA_InitStructure.DMA_BufferSize = CHANNEL_NUM*SAMPLE_TIME;  //��ֵΪ����һ��DMA����Ĵ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //��������λ���16λ����DMA����ߴ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��16λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //������ѭ������ģʽ��һ�ֽ������Զ���ʼ���ִ���
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��x��ֹ�ڴ浽�ڴ�
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //����DMA_InitStruct�в���DMAͨ��
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

void Joystick_ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    ADC_DeInit(ADC1);
    
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڶ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = CHANNEL_NUM;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���  
    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_71Cycles5);
    
    ADC_DMACmd(ADC1, ENABLE); 
    
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼ 
	while(ADC_GetResetCalibrationStatus(ADC1)){};	//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1)){};	 //�ȴ�У׼����
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
}

//handle raw adc value(get average value)
void Handle_adc_value(void)
{
    int sum1, sum2, sum3, sum4, i;
    sum1 = sum2 = sum3 = sum4 = 0;
    for(i = 0; i<SAMPLE_TIME; i++)
    {
        sum1 += ADC_value[i*CHANNEL_NUM];
        sum2 += ADC_value[1+i*CHANNEL_NUM];
        sum3 += ADC_value[2+i*CHANNEL_NUM];
        sum4 += ADC_value[3+i*CHANNEL_NUM];
    }
    adc_x2 = sum1/SAMPLE_TIME;
    adc_y2 = sum2/SAMPLE_TIME;
    adc_x1 = sum3/SAMPLE_TIME;
    adc_y1 = sum4/SAMPLE_TIME;    
}

void cal_range(uint16_t num, signed char  *buffer)
{
    signed short diff;

    if(num>0 && num<1948)
    {
        if(num<200)
            *buffer = -100;
        else
        {
            diff = num - 1948;
            *buffer = (float)diff/1748.0 *100;
        }
    }
    else if(num>2148 && num<4096)
    {
        if(num>3896)
            *buffer = 100;
        else
        {
            diff = num - 2148;
            *buffer = (float)diff/1748.0 *100;
        }
    }
}
//joystick1 handle direction -100~100
//joystick2 handle accelerator -100~100
void Handle_coordinate(uint16_t adc_x, uint16_t adc_y, uint8_t joystick)
{
    cal_range(adc_x,coordinate+joystick*2);
    cal_range(adc_y,coordinate+1+joystick*2);
}
