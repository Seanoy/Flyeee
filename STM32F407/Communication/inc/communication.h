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

typedef struct{
    float roll;
    float pitch;
    float yaw;
    short thrust;
    uint8_t cmd;
}nrf_ctrl_t;

extern nrf_data_t nrf_rxdata;
extern nrf_ctrl_t nrf_ctrl;

void Communication_Init(void);
u8 NRF24L01_Check_Data(u8* data);
void NRF24L01_Convert_Data(nrf_data_t *nrf_rxdata, nrf_ctrl_t *nrf_ctrl);

#endif
