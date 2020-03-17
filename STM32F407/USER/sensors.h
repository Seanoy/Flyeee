#ifndef __SENSORS_H
#define __SENSORS_H



//3axis sensor type
typedef struct 
{
    short x;
    short y;
    short z;
}axis3f_t;


void Filter_Init(void);
void processSensordata(void);


#endif
