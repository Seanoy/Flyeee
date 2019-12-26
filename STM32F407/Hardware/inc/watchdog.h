#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "bsp_sys.h"
#include <stdbool.h>

#define WATCHDOG_RESET_MS 	150	/*看门狗复位时间*/
#define watchdogReset() 	(IWDG_ReloadCounter())

void watchdogInit(u16 xms);
bool watchdogTest(void);


#endif 

