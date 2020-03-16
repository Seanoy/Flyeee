#ifndef __BSP_LED_H
#define __BSP_LED_H

#include <stm32f10x.h>
#include "bsp_sys.h"
void LED_GPIO_Config(void);
#define LED1 PCout(13)

#endif
