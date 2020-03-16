#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "bsp.h"

#define DATA_LENGTH 4

typedef struct{
    uint8_t head;
    uint8_t cmd;
    signed char data[DATA_LENGTH];
    uint8_t checksum;
}nrf_data_t;

extern nrf_data_t nrf_rxdata;

void Communication_Init(void);
u8 Handle_NRF_Data(u8* data);

#endif
