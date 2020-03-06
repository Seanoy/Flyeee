#ifndef __BPS_ADC_H
#define __BPS_ADC_H

#include <stm32f10x.h>
#define SAMPLE_TIME     5
#define CHANNEL_NUM     4

#define ORIGIN_POINT    2048
#define THRESHOLD_XY    248

void Joystick_DMA_Init(void);
void Joystick_ADC_Init(void);

void Handle_adc_value(void);//use timer processes it regularly
void Handle_xy(uint16_t adc_x, uint16_t adc_y);

#endif
