#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f10x.h"
#include <string.h>

#define DATA_LENGTH 4

typedef struct {
    uint8_t head;
    uint8_t cmd;
    signed char data[DATA_LENGTH];
    uint8_t checksum;
}nrf_data_t;

extern nrf_data_t nrf_txdata;

void Communication_Init(void);
void Fill_Data(uint8_t cmd, signed char *buffer, nrf_data_t *frame);
//void Convert_Struct(struct data_frame *frame, uint8_t *buffer);

#endif
