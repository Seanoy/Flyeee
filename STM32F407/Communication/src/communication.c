#include "communication.h"

nrf_data_t nrf_rxdata;
nrf_ctrl_t nrf_ctrl;

void Communication_Init(void)
{
    nrf_rxdata.head = 0xFE;
    nrf_rxdata.cmd = 0x00;
    memset(&nrf_rxdata.data, (signed char)0, sizeof(nrf_rxdata.data));
    nrf_rxdata.checksum=0xFF;
    memset(&nrf_ctrl, 0, sizeof(nrf_ctrl_t));
}

u8 Cal_Checksum( nrf_data_t frame)
{
    return ~(frame.cmd + frame.data[0] + frame.data[1] + frame.data[2] + frame.data[3]);
}

u8 NRF24L01_Check_Data(u8* data)
{
    nrf_rxdata.head = data[0];
    nrf_rxdata.cmd = data[1];
    memcpy(nrf_rxdata.data,data,4);
    nrf_rxdata.checksum = data[6];
    
    if(Cal_Checksum(nrf_rxdata) == nrf_rxdata.checksum)
        return 0;
    else
        return 1;
}

void NRF24L01_Convert_Data(nrf_data_t *nrf_rxdata, nrf_ctrl_t *nrf_ctrl)
{
    nrf_ctrl->pitch = nrf_rxdata->data[0]*0.25;
    nrf_ctrl->roll = nrf_rxdata->data[1]*0.25;
    nrf_ctrl->yaw = nrf_rxdata->data[2]*1.25;
    nrf_ctrl->thrust = (100+nrf_rxdata->data[3])*300;
    nrf_ctrl->cmd = nrf_rxdata->cmd;
}
