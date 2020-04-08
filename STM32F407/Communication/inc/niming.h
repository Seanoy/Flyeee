#ifndef __NIMING_H
#define __NIMING_H

#include <stm32f4xx.h>

void ANO_Send_F1(s16 _a, s16 _b, s32 _c);
void ANO_Send_01(s16 acc_x, s16 acc_y, s16 acc_z, s16 gyr_x, s16 gyr_y, s16 gyr_z, u8 shock_sta);
void ANO_Send_02(s16 mag_x, s16 mag_y, s16 mag_z, s32 bar, s16 tmp, s8 bar_sta, u8 mag_sta);
void ANO_Send_03(s16 roll, s16 pitch, s16 yaw, u8 fusion_sta);
void ANO_Send_04(s16 v0, s16 v1, s16 v2, s16 v3, u8 fusion_sta);
void ANO_Send_05(s32 alt_fu, s32 alt_add, u8 alt_sta);
void ANO_Send_20(u8 pwm1, u8 pwm2, u8 pwm3, u8 pwm4);

void usart1_send_string(u8 *dataToSend, u8 cnt);

#endif
