#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f10x.h"
#include <string.h>

#define DATA_LENGTH 4

extern struct data_frame txdata;

void Comm_Init(void);
void Fill_Data(uint8_t cmd, signed char *buffer,struct data_frame *frame);
//void Convert_Struct(struct data_frame *frame, uint8_t *buffer);

#endif
