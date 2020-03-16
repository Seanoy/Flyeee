#include "communication.h"
#define DATA_LENGTH 4

typedef struct{
    uint8_t head;
    uint8_t cmd;
    signed char data[DATA_LENGTH];
    uint8_t checksum;
}nrf_data_t;

nrf_data_t nrf_rxdata;

void Communication_Init(void)
{
    nrf_rxdata.head = 0xFE;
    nrf_rxdata.cmd = 0x00;
    memset(&nrf_rxdata.data, (signed char)0, sizeof(nrf_rxdata.data)*sizeof(signed char));
    nrf_rxdata.checksum=0xFF;
}

u8 Cal_Checksum(struct data_frame frame)
{
    return ~(frame.cmd + frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3]);
}

u8 Handle_NRF_Data(u8* data)
{
    nrf_rxdata.head = data[0];
    nrf_rxdata.cmd = data[1];
    memcpy(nrf_rxdata.data,data,4);
    nrf_rxdata.checksum = data[6];
    
    if(Cal_Checksum(nrf_rxdata) == nrf_rxdata.checksum){
        return 0;
    }
    else{
        return 1;
    }
}

