#ifndef __BSP_WATCHDOG_H
#define __BSP_WATCHDOG_H
#include "bsp_sys.h"

void IWDG_Init(u8 prer,u16 rlr);//IWDG³õÊ¼»¯
void IWDG_Feed(void);  //Î¹¹·º¯Êý

#endif 

