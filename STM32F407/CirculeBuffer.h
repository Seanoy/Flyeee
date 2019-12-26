#ifndef __CIRCULEBUFFER_H
#define __CIRCULEBUFFER_H
#include "stdbool.h"
#include "stdint.h"
#include "string.h"

/*是(1)否(0)使用环形缓冲区*/
#define IF_USE_RING_BUFFER    1U

/*数据缓冲区长度*/
#define DATA_BUF_LENGTH              512U

/*环形缓冲区相关结构体*/
typedef struct {
    volatile uint8_t  head;/*缓冲区头部*/
    volatile uint8_t  tail;/*缓冲区尾部*/
    volatile uint8_t  dataLen;/*缓冲区内数据长度*/
    volatile uint8_t  readWriteMutexFlag;/*读写互斥标志*/
    uint8_t           aFrameLen[25];/*存储接收帧的长度*/
    volatile uint8_t  frameNum;/*缓冲区内桢数*/
    uint8_t           ringBuf[DATA_BUF_LENGTH];/*缓冲区*/
}ringBufType_t;

/*多缓冲*/
typedef struct {
    ringBufType_t RingBuf_1;
    ringBufType_t RingBuf_2;
}multRingBufType_t;

#ifndef COUNTOF
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#endif

#if IF_USE_RING_BUFFER
/*外部声明*/
extern multRingBufType_t multRingBufDeal;

/*写入数据到环形缓冲区*/
bool WriteDataToRingBuffer(ringBufType_t *pRingBuf, uint8_t *pBuf, uint8_t len);
/*从环形缓冲区读出数据*/
uint8_t ReadDataFromRingBuffer(ringBufType_t *pRingBuf, uint8_t *pBuf, uint8_t len);
#endif/*IF_USE_RING_BUFFER*/

#endif/*__RING_BUFFER_H*/