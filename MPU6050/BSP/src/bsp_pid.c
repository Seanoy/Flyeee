#include "bsp_pid.h"    


#include <math.h>
//extern T_RC_Data                         Rc_D;                //遥控通道数据;

attitude_t Rc_D;

//extern u8 txbuf[4];         //发送缓冲
//extern u8 rxbuf[4];         //接收缓冲
//extern u16 test1[3]; //接收到NRf24L01数据

int Motor_Ele=0;                                           //俯仰期望
int Motor_Ail=0;                                           //横滚期望

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;

float thr=20;//thrust

/*********************************/
float Pitch_i,Roll_i,Yaw_i;                       	//积分项
float Pitch_old,Roll_old,Yaw_old;                 	//角度保存
float Pitch_d,Roll_d,Yaw_d;          								//微分项
float RC_Pitch,RC_Roll,RC_Yaw;                    	//姿态角
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;	//外环总输出
        //外环PID参数
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

float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//陀螺仪保存

float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_kd_out;//内环单项输出
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出       
       
//内环PID参数
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
       
  ////////////////////////外环角度环(PID)///////////////////////////////
  Pitch_i+=(state.attitude.pitch-RC_Pitch);
//-------------Pitch积分限幅----------------//
  if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
//-------------Pitch微分--------------------//
  Pitch_d=state.attitude.pitch-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(state.attitude.pitch-RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//角度保存
  Pitch_old=state.attitude.pitch;
/*********************************************************/       
       
  RC_Roll=(Rc_D.roll-1500)/20;
  Roll_i+=(state.attitude.roll-RC_Roll);
//-------------Roll积分限幅----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll微分--------------------//
  Roll_d=state.attitude.roll-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(state.attitude.roll-RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll角度保存------------------//
  Roll_old=state.attitude.roll;
       
       
        RC_Yaw=(Rc_D.yaw-1500)*10;
//-------------Yaw微分--------------------//
  Yaw_d=gyro_last.z-Yaw_old;
//-------------Roll  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(gyro_last.z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll角度保存------------------//
  Yaw_old=gyro_last.z;
       
       
  ////////////////////////内环角速度环(PD)///////////////////////////////       
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
  Gyro_radian_old_z = gyro_last.z;   //储存历史值
                             
  motor1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
	motor2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
	motor3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
	motor4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);                       
                       
  setMotorPWM(true,motor1,motor2,motor3,motor4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}






