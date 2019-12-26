#include "bsp_pid.h"    


#include <math.h>
//extern T_RC_Data                         Rc_D;                //ң��ͨ������;

attitude_t Rc_D;

//extern u8 txbuf[4];         //���ͻ���
//extern u8 rxbuf[4];         //���ջ���
//extern u16 test1[3]; //���յ�NRf24L01����

int Motor_Ele=0;                                           //��������
int Motor_Ail=0;                                           //�������

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;

float thr=20;//thrust

/*********************************/
float Pitch_i,Roll_i,Yaw_i;                       	//������
float Pitch_old,Roll_old,Yaw_old;                 	//�Ƕȱ���
float Pitch_d,Roll_d,Yaw_d;          								//΢����
float RC_Pitch,RC_Roll,RC_Yaw;                    	//��̬��
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;	//�⻷�����
        //�⻷PID����
float Pitch_shell_kp=280;//30 140
float Pitch_shell_kd=0;//
float Pitch_shell_ki=0;//
/*********************************/
float Roll_shell_kp=250;//30
float Roll_shell_kd=0;//10                 
float Roll_shell_ki=0;//0.08
/*********************************/
float Yaw_shell_kp=1.5;//10;//30
float Yaw_shell_kd=0;//10                 
float Yaw_shell_ki=0;//0.08;//0.08

float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//�����Ǳ���

float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_kd_out;//�ڻ��������
float Pitch_core_out,Roll_core_out,Yaw_core_out;//�ڻ������       
       
//�ڻ�PID����
//float Pitch_core_kp=0.040;
//float Pitch_core_kd=0.008;////0.007;//0.07;
float Pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;

float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;

float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;


int16_t motor1=0,motor2=0,motor3=0,motor4=0;

float tempjd=0;

void CONTROL(state_t state,Axis3f gyro_last)
{
       
  RC_Pitch=(Rc_D.pitch-1500)/20;
       
  ////////////////////////�⻷�ǶȻ�(PID)///////////////////////////////
  Pitch_i+=(state.attitude.pitch-RC_Pitch);
//-------------Pitch�����޷�----------------//
  if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
//-------------Pitch΢��--------------------//
  Pitch_d=state.attitude.pitch-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(state.attitude.pitch-RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//�Ƕȱ���
  Pitch_old=state.attitude.pitch;
/*********************************************************/       
       
  RC_Roll=(Rc_D.roll-1500)/20;
  Roll_i+=(state.attitude.roll-RC_Roll);
//-------------Roll�����޷�----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll΢��--------------------//
  Roll_d=state.attitude.roll-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(state.attitude.roll-RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll�Ƕȱ���------------------//
  Roll_old=state.attitude.roll;
       
       
        RC_Yaw=(Rc_D.yaw-1500)*10;
//-------------Yaw΢��--------------------//
  Yaw_d=gyro_last.z-Yaw_old;
//-------------Roll  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(gyro_last.z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll�Ƕȱ���------------------//
  Yaw_old=gyro_last.z;
       
       
  ////////////////////////�ڻ����ٶȻ�(PD)///////////////////////////////       
  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + gyro_last.y * 3.5);
  pitch_core_kd_out = Pitch_core_kd * (gyro_last.y   - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + gyro_last.x *3.5);
  Roll_core_kd_out  = Roll_core_kd  * (gyro_last.x   - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + gyro_last.z * 1);
  Yaw_core_kd_out  = Yaw_core_kd  * (gyro_last.z   - Gyro_radian_old_z);
       
       
	Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
	Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
	Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

  Gyro_radian_old_y = gyro_last.x;
  Gyro_radian_old_x = gyro_last.y;
  Gyro_radian_old_z = gyro_last.z;   //������ʷֵ
                             
  motor1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
	motor2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
	motor3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
	motor4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);                       
                       
  setMotorPWM(true,motor1,motor2,motor3,motor4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}






