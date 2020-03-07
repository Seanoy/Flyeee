#ifndef __BPS_ADC_H
#define __BPS_ADC_H

#include <stm32f10x.h>
#define SAMPLE_TIME     5
#define CHANNEL_NUM     4

extern signed char coordinate[4];//{x, y} 0~100
extern uint32_t adc_x1, adc_y1, adc_x2, adc_y2;

void Joystick_DMA_Init(void);
void Joystick_ADC_Init(void);

void Handle_adc_value(void);//use timer processes it regularly
void Handle_coordinate(uint16_t adc_x, uint16_t adc_y, uint8_t joystick);
void Handle_XY(void);

#endif
