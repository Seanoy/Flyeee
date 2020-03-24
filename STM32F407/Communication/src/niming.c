#include "niming.h"
#include <stdbool.h>
#include <stdlib.h>

//������λ�����ݸ�ʽΪС��ģʽ���ȷ��͵��ֽ�����
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	        //ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	    //ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
unsigned char DataToSend[100]={0};

//�����Զ������� �����˲�֮�������
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
cmd:0x01 ���͹��Դ���������
��������  int16  int16  int16  int16  int16  int16  uint8
��������  ACC_X ACC_Y ACC_Z GYR_X GYR_Y GYR_Z SHOCK_STA
ACC��GYR������Ϊ���ٶȡ������ǡ������̴��������ݡ�
SHOCK_ STA����״̬
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
cmd:0x02 �������̡���ѹ���¶ȴ���������
��������  int16  int16  int16  Int32  int16  uint8  uint8
��������  MAG_X MAG_Y MAG_Z ALT_BAR  TMP BAR_STA MAG_STA
MAG�������̴��������ݡ�
TMP: �������¶ȣ��Ŵ� 10 �����䣬0.1 ���϶ȡ�
ALT_BAR����ѹ�Ƹ߶ȣ���λ cm��
BAR_STA��MAG_STA������Ϊ��ѹ״̬������״̬
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
cmd:0x03 ���ͷɿ���̬��ŷ���Ǹ�ʽ
��������  int16  int16  int16  uint8
��������  ROL*100 PIT*100 YAW*100 FUSION_STA
ROL��PIT��YAW����̬�ǣ�����Ϊ��������������򣬾�ȷ�� 0.01��
FUSION _STA ���ں�״̬��
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
cmd:0x04 ���ͷɿ���̬����Ԫ����ʽ
��������  int16  int16  int16  int16  uint8
��������  V0*10000  V1*10000 V2*10000 V3*10000 FUSION _STA
V0��V1��V2��V3����Ԫ��������ʱ���� 10000 ����
FUSION _STA ���ں�״̬��
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
cmd:0x05 �߶�����
��������  Int32  Int32  Uint8
��������  ALT_FU ALT_ADD ALT_STA
ALT_FU���ںϺ�Եظ߶ȣ���λ���ס�
ALT_ADD�����Ӹ߶ȴ��и߶����ݣ��糬�����������࣬��λ���ס�
ALT_STA�����״̬��
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

//����������ȷ���
bool checksum(u8 *data_buf)
{
    u8 sumcheck = 0;
    u8 addcheck = 0;
    for(u8 i=0; i < (data_buf[3]+4); i++)
    {
        sumcheck += data_buf[i]; //��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
        addcheck += sumcheck;  //ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
    }
    //����������sumcheck��addcheck�ͽ��յ���check������ȣ�����У��ͨ������֮��������
    if(sumcheck == data_buf[data_buf[3]+4] && addcheck == data_buf[data_buf[3]+5])
        return true; //У��ͨ��
    else
        return false; //У��ʧ��
}

//�������ݵ�������λ��
//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_string(u8 *dataToSend, u8 cnt)
{
    int i;
    for(i = 0; i < cnt; i++)
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
        USART_SendData(USART1, dataToSend[i]);
    }
}

