#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "bsp.h"

extern struct nrf_data_t nrf_rxdata;

void Communication_Init(void);
u8 Handle_NRF_Data(u8* data);

#endif
