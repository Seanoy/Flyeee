#include "niming.h"
#include <stdbool.h>
#include <stdlib.h>

//匿名上位机数据格式为小端模式，先发送低字节数据
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	        //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	    //取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
unsigned char DataToSend[100]={0};

//发送自定义数据 例如滤波之后的数据
void ANO_Send_F1(s16 _a, s16 _b, s32 _c)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0xF1;
    DataToSend[_cnt++] = 8;
    
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    
    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);
    
    DataToSend[_cnt++] = BYTE0(_c);
    DataToSend[_cnt++] = BYTE1(_c);
    DataToSend[_cnt++] = BYTE2(_c);
    DataToSend[_cnt++] = BYTE3(_c);
    
    
    for(u8 i = 0; i<DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

/*
cmd:0x01 发送惯性传感器数据
数据类型  int16  int16  int16  int16  int16  int16  uint8
数据内容  ACC_X ACC_Y ACC_Z GYR_X GYR_Y GYR_Z SHOCK_STA
ACC、GYR：依次为加速度、陀螺仪、磁罗盘传感器数据。
SHOCK_ STA：震动状态
*/
void ANO_Send_01(s16 acc_x, s16 acc_y, s16 acc_z, s16 gyr_x, s16 gyr_y, s16 gyr_z, u8 shock_sta)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0x01;
    DataToSend[_cnt++] = 13;
    
    DataToSend[_cnt++] = BYTE0(acc_x);
    DataToSend[_cnt++] = BYTE1(acc_x);
    
    DataToSend[_cnt++] = BYTE0(acc_y);
    DataToSend[_cnt++] = BYTE1(acc_y);
    
    DataToSend[_cnt++] = BYTE0(acc_z);
    DataToSend[_cnt++] = BYTE1(acc_z);
    
    DataToSend[_cnt++] = BYTE0(gyr_x);
    DataToSend[_cnt++] = BYTE1(gyr_x);
    
    DataToSend[_cnt++] = BYTE0(gyr_y);
    DataToSend[_cnt++] = BYTE1(gyr_y);
    
    DataToSend[_cnt++] = BYTE0(gyr_z);
    DataToSend[_cnt++] = BYTE1(gyr_z);
    
    DataToSend[_cnt++] = shock_sta;
    
    for(u8 i = 0; i<DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

/*
cmd:0x02 发送罗盘、气压、温度传感器数据
数据类型  int16  int16  int16  Int32  int16  uint8  uint8
数据内容  MAG_X MAG_Y MAG_Z ALT_BAR  TMP BAR_STA MAG_STA
MAG：磁罗盘传感器数据。
TMP: 传感器温度，放大 10 倍传输，0.1 摄氏度。
ALT_BAR：气压计高度，单位 cm。
BAR_STA、MAG_STA：依次为气压状态、罗盘状态
*/
void ANO_Send_02(s16 mag_x, s16 mag_y, s16 mag_z, s32 bar, s16 tmp, s8 bar_sta, u8 mag_sta)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0x02;
    DataToSend[_cnt++] = 14;
    
    DataToSend[_cnt++] = BYTE0(mag_x);
    DataToSend[_cnt++] = BYTE1(mag_x);
    
    DataToSend[_cnt++] = BYTE0(mag_y);
    DataToSend[_cnt++] = BYTE1(mag_y);
    
    DataToSend[_cnt++] = BYTE0(mag_z);
    DataToSend[_cnt++] = BYTE1(mag_z);
    
    DataToSend[_cnt++] = BYTE0(bar);
    DataToSend[_cnt++] = BYTE1(bar);
    DataToSend[_cnt++] = BYTE2(bar);
    DataToSend[_cnt++] = BYTE3(bar);
    
    DataToSend[_cnt++] = BYTE0(tmp);
    DataToSend[_cnt++] = BYTE1(tmp);
    
    DataToSend[_cnt++] = bar_sta;
    
    DataToSend[_cnt++] = mag_sta;

    for(u8 i = 0; i < DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

/*
cmd:0x03 发送飞控姿态：欧拉角格式
数据类型  int16  int16  int16  uint8
数据内容  ROL*100 PIT*100 YAW*100 FUSION_STA
ROL、PIT、YAW：姿态角，依次为横滚、俯仰、航向，精确到 0.01。
FUSION _STA ：融合状态。
*/
void ANO_Send_03(s16 roll, s16 pitch, s16 yaw, u8 fusion_sta)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0x03;
    DataToSend[_cnt++] = 7;
    
    DataToSend[_cnt++] = BYTE0(roll);
    DataToSend[_cnt++] = BYTE1(roll);
    
    DataToSend[_cnt++] = BYTE0(pitch);
    DataToSend[_cnt++] = BYTE1(pitch);
    
    DataToSend[_cnt++] = BYTE0(yaw);
    DataToSend[_cnt++] = BYTE1(yaw);
    
    DataToSend[_cnt++] = BYTE0(fusion_sta);

    for(u8 i = 0; i<DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

/*
cmd:0x04 发送飞控姿态：四元数格式
数据类型  int16  int16  int16  int16  uint8
数据内容  V0*10000  V1*10000 V2*10000 V3*10000 FUSION _STA
V0、V1、V2、V3：四元数，传输时扩大 10000 倍。
FUSION _STA ：融合状态。
*/
void ANO_Send_04(s16 v0, s16 v1, s16 v2, s16 v3, u8 fusion_sta)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0x04;
    DataToSend[_cnt++] = 9;
    
    DataToSend[_cnt++] = BYTE0(v0);
    DataToSend[_cnt++] = BYTE1(v0);
    
    DataToSend[_cnt++] = BYTE0(v1);
    DataToSend[_cnt++] = BYTE1(v1);
    
    DataToSend[_cnt++] = BYTE0(v2);
    DataToSend[_cnt++] = BYTE1(v2);
    
    DataToSend[_cnt++] = BYTE0(v3);
    DataToSend[_cnt++] = BYTE1(v3);
    
    DataToSend[_cnt++] = BYTE0(fusion_sta);

    for(u8 i = 0; i<DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

/*
cmd:0x05 高度数据
数据类型  Int32  Int32  Uint8
数据内容  ALT_FU ALT_ADD ALT_STA
ALT_FU：融合后对地高度，单位厘米。
ALT_ADD：附加高度传感高度数据，如超声波、激光测距，单位厘米。
ALT_STA：测距状态。
*/
void ANO_Send_05(s32 alt_fu, s32 alt_add, u8 alt_sta)
{
    u8 _cnt = 0;
    u8 sc = 0;
    u8 ac = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0x04;
    DataToSend[_cnt++] = 9;
    
    DataToSend[_cnt++] = BYTE0(alt_fu);
    DataToSend[_cnt++] = BYTE1(alt_fu);
    DataToSend[_cnt++] = BYTE2(alt_fu);
    DataToSend[_cnt++] = BYTE3(alt_fu);
    
    DataToSend[_cnt++] = BYTE0(alt_add);
    DataToSend[_cnt++] = BYTE1(alt_add);
    DataToSend[_cnt++] = BYTE2(alt_add);
    DataToSend[_cnt++] = BYTE3(alt_add);
    
    DataToSend[_cnt++] = BYTE0(alt_sta);

    for(u8 i = 0; i<DataToSend[3]+4; i++)
    {
        sc += DataToSend[i];
        ac += sc;
    }
    
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;
    usart1_send_string(DataToSend, _cnt);
}

//检验数据正确与否
bool checksum(u8 *data_buf)
{
    u8 sumcheck = 0;
    u8 addcheck = 0;
    for(u8 i=0; i < (data_buf[3]+4); i++)
    {
        sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck;  //每一字节的求和操作，进行一次sumcheck的累加
    }
    //如果计算出的sumcheck和addcheck和接收到的check数据相等，代表校验通过，反之数据有误
    if(sumcheck == data_buf[data_buf[3]+4] && addcheck == data_buf[data_buf[3]+5])
        return true; //校验通过
    else
        return false; //校验失败
}

//发送数据到匿名上位机
//串口1发送1个字符 
//c:要发送的字符
void usart1_send_string(u8 *dataToSend, u8 cnt)
{
    int i;
    for(i = 0; i < cnt; i++)
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
        USART_SendData(USART1, dataToSend[i]);
    }
}

