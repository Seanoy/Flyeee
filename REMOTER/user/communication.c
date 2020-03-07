#include "communication.h"

struct data_frame{
    uint8_t head;
    uint8_t cmd;
    signed char data[4];
    uint8_t checksum;
}txdata;

void Comm_Init(void)
{
    txdata.head = 0xFE;
    txdata.cmd = 0x00;
    memset(&txdata.data, 0, sizeof(txdata.data)*sizeof(signed char));
    txdata.checksum=0xFF;
}

static void Cal_Checksum(struct data_frame *frame)
{
    frame->checksum = ~(frame->cmd + frame->data[0] + frame->data[1] + frame->data[2] + frame->data[3]);
}

static void Flush_Data(struct data_frame *frame)
{
    memset(frame, 0, sizeof(struct data_frame));
    frame->head = 0xFE;//²¹³äÍ·²¿
}

void Fill_Data(uint8_t cmd, signed char *buffer,struct data_frame *frame)
{
    uint8_t i;
    Flush_Data(frame);//clear data
    frame->cmd = cmd;//fill cmd
    for(i=0 ;i<4 ;i++)//fill data
        frame->data[i] = *(buffer+i);
    Cal_Checksum(frame);//cal checksum
}
