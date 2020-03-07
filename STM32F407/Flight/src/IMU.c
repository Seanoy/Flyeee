#include "imu.h"

S_FLOAT_ANGLE Q_ANGLE;
float   halfT=0.002;;                    //����ʱ���һ�룬���ǵ�Ӧ����2ms

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   
//  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

  float delta_2=0;
//  float delta_4=0;

  const static float FACTOR = 0.002;

  float norm=0.0f;
  //float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  //float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  norm = sqrt(ax*ax + ay*ay +az*az);       // ���������� �Ѽ��ٶȼƵ���ά����ת�ɵ�λ������
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
    
  vx = 2*(q1q3 - q0q2);						// ���Ʒ��������						
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������


  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (ay*vz - az*vy);                           					
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
//axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
//axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
//������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
//������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
//�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

/*  
  exInt = exInt + ex * Ki ;					//�����Ӧ�û��ַ���			 
  eyInt = eyInt + ey * Ki ;
  ezInt = ezInt + ez * Ki ;


  gx = gx + Kp*ex + exInt;					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;		  */ 

//  halfT=GET_NOWTIME();			  				//���μ����ʱ��������λ��
  halfT=0.002;
  gx = gx + ex*FACTOR/halfT; 					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + ey*FACTOR/halfT; 
  gz = gz + ez*FACTOR/halfT;	 

/* 
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
  delta_4=delta_2*delta_2;

  q0 = (1-delta_2/8+delta_4/384)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT*(0.5-delta_2/48);			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨���ĽױϿ���
  q1 = (1-delta_2/8+delta_4/384)*q1 + (q0*gx + q2*gz - q3*gy)*halfT*(0.5-delta_2/48);
  q2 = (1-delta_2/8+delta_4/384)*q2 + (q0*gy - q1*gz + q3*gx)*halfT*(0.5-delta_2/48);
  q3 = (1-delta_2/8+delta_4/384)*q3 + (q0*gz + q1*gy - q2*gx)*halfT*(0.5-delta_2/48);	*/		 

 
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
 
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;			 
/* 												   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;							// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨��һ�����ⷨ
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   */
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// ��������Ԫ
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //ת��Ϊŷ����
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Q_ANGLE.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Q_ANGLE.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}


void AGMIMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;
  static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
 
  float delta_2=0;

  const static float FACTOR = 0.002;

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;

  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
//  wx = (float)(2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2));
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = (float)(2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2));  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);


 // halfT=GET_NOWTIME();			  //���μ����ʱ��������λ��

  gx = gx + ex*FACTOR/halfT; 					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + ey*FACTOR/halfT; 
  gz = gz + ez*FACTOR/halfT;	 
	  
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
 
  q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;			 
/* 												   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨��һ�����ⷨ
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;			   	   */
										   
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);		// ��������Ԫ
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //ת��Ϊŷ����
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  Q_ANGLE.Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  Q_ANGLE.Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}
