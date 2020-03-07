#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include <stm32f4xx.h>
#include "bsp_watchdog.h"

void Watchdog_TIMER_Init(u16 arr, u16 psc);

#endif
