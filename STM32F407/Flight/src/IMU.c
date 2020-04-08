#include "imu.h"
#include "stabilizer_type.h"

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

float invSqrt(float x)	/*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

#define ACCZ_SAMPLE		350

float Kp = 0.4f;		/*��������*/
float Ki = 0.001f;		/*��������*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*��������ۼ�*/

static float q0 = 1.0f;	/*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*��ת����*/

static float maxError = 0.f;		/*������*/
bool isGravityCalibrated = false;	/*�Ƿ�УУ׼���*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*��̬���ٶ�*/


static float invSqrt(float x);	/*���ٿ�ƽ����*/

static void calBaseAcc(float* acc)	/*���㾲̬���ٶ�*/
{
	static u16 cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	
	for(u8 i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*��������*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;
		
		if(maxError < 0.015f)
		{
			for(u8 i=0; i<3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
			
			isGravityCalibrated = true;
			
		}
		
		for(u8 i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*������ת����*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(axis3f_t acc, axis3f_t gyro, state_t *state, float dt)
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

  norm = sqrt(acc.x*acc.x + acc.y*acc.y +acc.z*acc.z);       // ���������� �Ѽ��ٶȼƵ���ά����ת�ɵ�λ������
  acc.x = acc.x /norm;
  acc.y = acc.y / norm;
  acc.z = acc.z / norm;
    
  vx = 2*(q1q3 - q0q2);						// ���Ʒ��������						
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������


  // Error is sum of cross product between estimated direction and measured direction of field vectors
  ex = (acc.y*vz - acc.z*vy);                           					
  ey = (acc.z*vx - acc.x*vz);
  ez = (acc.x*vy - acc.y*vx);
//axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
//axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
//������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
//������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
//�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ�
//���������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣�
//���Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

/*  
  exInt = exInt + ex * Ki ;					//�����Ӧ�û��ַ���			 
  eyInt = eyInt + ey * Ki ;
  ezInt = ezInt + ez * Ki ;


  gx = gx + Kp*ex + exInt;					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;		  */ 

//  halfT=GET_NOWTIME();			  				//���μ����ʱ��������λ��
  gyro.x = gyro.x + ex*FACTOR/dt; 					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gyro.y = gyro.y + ey*FACTOR/dt; 
  gyro.z = gyro.z + ez*FACTOR/dt;	 

/* 
  delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);	
  delta_4=delta_2*delta_2;

  q0 = (1-delta_2/8+delta_4/384)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT*(0.5-delta_2/48);			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨���ĽױϿ���
  q1 = (1-delta_2/8+delta_4/384)*q1 + (q0*gx + q2*gz - q3*gy)*halfT*(0.5-delta_2/48);
  q2 = (1-delta_2/8+delta_4/384)*q2 + (q0*gy - q1*gz + q3*gx)*halfT*(0.5-delta_2/48);
  q3 = (1-delta_2/8+delta_4/384)*q3 + (q0*gz + q1*gy - q2*gx)*halfT*(0.5-delta_2/48);	*/		 

 
  delta_2=(2*dt*gyro.x)*(2*dt*gyro.x)+(2*dt*gyro.y)*(2*dt*gyro.y)+(2*dt*gyro.z)*(2*dt*gyro.z);	
 
  q0 = (1-delta_2/8)*q0 + (-q1*gyro.x - q2*gyro.y - q3*gyro.z)*dt;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gyro.x + q2*gyro.z - q3*gyro.y)*dt;
  q2 = (1-delta_2/8)*q2 + (q0*gyro.y - q1*gyro.z + q3*gyro.x)*dt;
  q3 = (1-delta_2/8)*q3 + (q0*gyro.z + q1*gyro.y - q2*gyro.x)*dt;			 
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
  state->attitude.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  state->attitude.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  state->attitude.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}

//void imuUpdate(axis3f_t acc, axis3f_t gyro, state_t *state , float dt)	/*�����ں� �����˲�*/
//{
//	float normalise;
//	float ex, ey, ez;
//	float halfT = 0.5f * dt;
//	float accBuf[3] = {0.f};
//	axis3f_t tempacc = acc;
//	
//	gyro.x = gyro.x * DEG2RAD;	/* ��ת���� */
//	gyro.y = gyro.y * DEG2RAD;
//	gyro.z = gyro.z * DEG2RAD;

//	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ���������*/
//	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
//	{
//		/*��λ�����ټƲ���ֵ*/
//		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
//		acc.x *= normalise;
//		acc.y *= normalise;
//		acc.z *= normalise;

//		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
//		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
//		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
//		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
//		
//		/*����ۼƣ�����ֳ������*/
//		exInt += Ki * ex * dt ;  
//		eyInt += Ki * ey * dt ;
//		ezInt += Ki * ez * dt ;
//		
//		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
//		gyro.x += Kp * ex + exInt;
//		gyro.y += Kp * ey + eyInt;
//		gyro.z += Kp * ez + ezInt;
//	}
//	/* һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
//	float q0Last = q0;
//	float q1Last = q1;
//	float q2Last = q2;
//	float q3Last = q3;
//	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
//	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
//	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
//	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
//	
//	/*��λ����Ԫ��*/
//	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= normalise;
//	q1 *= normalise;
//	q2 *= normalise;
//	q3 *= normalise;
//	
//	imuComputeRotationMatrix();	/*������ת����*/
//	
//	/*����roll pitch yaw ŷ����*/
//	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
//	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
//	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
//	
//	if (!isGravityCalibrated)	/*δУ׼*/
//	{		
////		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
////		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
//		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
//		calBaseAcc(accBuf);		/*���㾲̬���ٶ�*/				
//	}
//}

void agmImuUpdate(axis3f_t acc, axis3f_t gyro, axis3f_t mag, state_t *state, float dt)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
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

  norm = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);       
  acc.x = acc.x / norm;
  acc.y = acc.y / norm;
  acc.z = acc.z / norm;

  norm = sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);          
  mag.x = mag.x / norm;
  mag.y = mag.y / norm;
  mag.z = mag.z / norm;

  // compute reference direction of flux
  hx = 2*mag.x*(0.5f - q2q2 - q3q3) + 2*mag.y*(q1q2 - q0q3) + 2*mag.z*(q1q3 + q0q2);
  hy = 2*mag.x*(q1q2 + q0q3) + 2*mag.y*(0.5f - q1q1 - q3q3) + 2*mag.z*(q2q3 - q0q1);
  hz = 2*mag.x*(q1q3 - q0q2) + 2*mag.y*(q2q3 + q0q1) + 2*mag.z*(0.5f - q1q1 - q2q2);         
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
  ex = (acc.y*vz - acc.z*vy) + (mag.y*wz - mag.z*wy);
  ey = (acc.z*vx - acc.x*vz) + (mag.z*wx - mag.x*wz);
  ez = (acc.x*vy - acc.y*vx) + (mag.x*wy - mag.y*wx);


 // halfT=GET_NOWTIME();			  //���μ����ʱ��������λ��

  gyro.x = gyro.x + ex*FACTOR/dt; 					//У�������ǲ���ֵ	   �ò���������PI����������ƫ							
  gyro.y = gyro.y + ey*FACTOR/dt; 
  gyro.z = gyro.z + ez*FACTOR/dt;	 
	  
  delta_2=(2*dt*gyro.x)*(2*dt*gyro.x)+(2*dt*gyro.y)*(2*dt*gyro.y)+(2*dt*gyro.z)*(2*dt*gyro.z);	
 
  q0 = (1-delta_2/8)*q0 + (-q1*gyro.x - q2*gyro.y - q3*gyro.z)*dt;			// ������Ԫ����	 ��Ԫ��΢�ַ���	��Ԫ�������㷨�����ױϿ���
  q1 = (1-delta_2/8)*q1 + (q0*gyro.x + q2*gyro.z - q3*gyro.y)*dt;
  q2 = (1-delta_2/8)*q2 + (q0*gyro.y - q1*gyro.z + q3*gyro.x)*dt;
  q3 = (1-delta_2/8)*q3 + (q0*gyro.z + q1*gyro.y - q2*gyro.x)*dt;			 
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
  state->attitude.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 					// pitch
  state->attitude.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 	// roll
  state->attitude.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; // yaw
}
