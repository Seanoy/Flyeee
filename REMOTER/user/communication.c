#include "communication.h"

nrf_data nrf_txdata;

void Communication_Init(void)
{
    nrf_txdata.head = 0xFE;
    nrf_txdata.cmd = 0x00;
    memset(&nrf_txdata.data, (signed char)0, sizeof(nrf_txdata.data)*sizeof(signed char));
    nrf_txdata.checksum=0xFF;
}

static void Cal_Checksum(nrf_data *frame)
{
    frame->checksum = ~(frame->cmd + frame->data[0] + frame->data[1] + frame->data[2] + frame->data[3]);
}

static void Flush_Data(nrf_data *frame)
{
    memset(frame, 0, sizeof(nrf_data));
    frame->head = 0xFE;//²¹³äÍ·²¿
}

void Fill_Data(uint8_t cmd, signed char *buffer,nrf_data *frame)
{
    uint8_t i;
    Flush_Data(frame);//clear data
    frame->cmd = cmd;//fill cmd
    for(i=0 ;i<DATA_LENGTH ;i++)//fill data
        frame->data[i] = *(buffer+i);
    Cal_Checksum(frame);//cal checksum
}

//void Convert_Struct(struct data_frame *frame, uint8_t *buffer)
//{
//    int i;
//    buffer[0] = frame->head;
//    buffer[1] = frame->cmd;
//    for(i=0;i<DATA_LENGTH;i++)
//    {
//       buffer[i+2] =  frame->data[i];
//    }
//    buffer[DATA_LENGTH+2] = frame->checksum;
//}
