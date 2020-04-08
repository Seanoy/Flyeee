#ifndef __BSP_H
#define __BSP_H

//standard library
#include <stm32f4xx.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//firmware
#include "bsp_sys.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_led.h"
#include "bsp_mpu6500.h"
#include "bsp_ak8963.h"
#include "bsp_myiic.h"
#include "bsp_motor.h"
#include "bsp_watchdog.h"
#include "bsp_timer.h"
#include "bsp_mpu6500.h"
#include "bsp_24l01.h"
#include "bsp_bmp180.h"

#include "communication.h"
#include "niming.h"

//freertos
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

//attitude
#include "pid.h"
#include "attitude_pid.h"
#include "kalman_filter.h"
#include "imu.h"
#include "sensors.h"
#include "filter.h"
#include "stabilizer.h"

void BSP_Init(void);

#endif
