#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f10x.h"
#include <string.h>

extern struct data_frame txdata;

void Comm_Init(void);
void Fill_Data(uint8_t cmd, signed char *buffer,struct data_frame *frame);

#endif
