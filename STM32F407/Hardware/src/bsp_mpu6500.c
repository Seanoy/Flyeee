#include "bsp_mpu6500.h"
#include "task_config.h"

//MPU9250读取的温度值
u16 temperature;

static const unsigned short mpu6500StTb[256] = {
  2620,2646,2672,2699,2726,2753,2781,2808, //7
  2837,2865,2894,2923,2952,2981,3011,3041, //15
  3072,3102,3133,3165,3196,3228,3261,3293, //23
  3326,3359,3393,3427,3461,3496,3531,3566, //31
  3602,3638,3674,3711,3748,3786,3823,3862, //39
  3900,3939,3979,4019,4059,4099,4140,4182, //47
  4224,4266,4308,4352,4395,4439,4483,4528, //55
  4574,4619,4665,4712,4759,4807,4855,4903, //63
  4953,5002,5052,5103,5154,5205,5257,5310, //71
  5363,5417,5471,5525,5581,5636,5693,5750, //79
  5807,5865,5924,5983,6043,6104,6165,6226, //87
  6289,6351,6415,6479,6544,6609,6675,6742, //95
  6810,6878,6946,7016,7086,7157,7229,7301, //103
  7374,7448,7522,7597,7673,7750,7828,7906, //111
  7985,8065,8145,8227,8309,8392,8476,8561, //119
  8647,8733,8820,8909,8998,9088,9178,9270,
  9363,9457,9551,9647,9743,9841,9939,10038,
  10139,10240,10343,10446,10550,10656,10763,10870,
  10979,11089,11200,11312,11425,11539,11654,11771,
  11889,12008,12128,12249,12371,12495,12620,12746,
  12874,13002,13132,13264,13396,13530,13666,13802,
  13940,14080,14221,14363,14506,14652,14798,14946,
  15096,15247,15399,15553,15709,15866,16024,16184,
  16346,16510,16675,16842,17010,17180,17352,17526,
  17701,17878,18057,18237,18420,18604,18790,18978,
  19167,19359,19553,19748,19946,20145,20347,20550,
  20756,20963,21173,21385,21598,21814,22033,22253,
  22475,22700,22927,23156,23388,23622,23858,24097,
  24338,24581,24827,25075,25326,25579,25835,26093,
  26354,26618,26884,27153,27424,27699,27976,28255,
  28538,28823,29112,29403,29697,29994,30294,30597,
  30903,31212,31524,31839,32157,32479,32804,33132
};

/** Evaluate the values from a MPU6500 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
bool MPU_EvaluateSelfTest(float low, float high, float value, char* string)
{
  if (value < low || value > high)
  {
    printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, (double)low, (double)high, (double)value);
    return false;
  }
  return true;
}

/** Do a MPU6500 self test.
 * @return True if self test passed, false otherwise
 */
bool MPU_SelfTest()
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t saveReg[5];
  uint8_t selfTest[6];
  int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
  int32_t factoryTrim[6];
  float aDiff[3], gDiff[3];
  uint8_t FS = 0;
  int i;

  // Save old configuration
  saveReg[0] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG);
  saveReg[1] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_CFG_REG);
  saveReg[2] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG);
  saveReg[3] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG2_REG);
  saveReg[4] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG);
  // Write test configuration
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, 0x00); // Set gyro sample rate to 1 kHz
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_CFG_REG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, 1<<FS); // Set full scale range for the gyro to 250 dps
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG2_REG, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for(i = 0; i < 200; i++)
  {
    // get average current values of gyro and acclerometer
    IIC_Read_NByte(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    IIC_Read_NByte(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
    gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
  }

  for (i = 0; i < 3; i++)
  { // Get average of 200 values and store as average current readings
    aAvg[i] /= 200;
    gAvg[i] /= 200;
  }

  // Configure the accelerometer for self-test
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  vTaskDelay(25); // Delay a while to let the device stabilize

  for(i = 0; i < 200; i++)
  {
    // get average self-test values of gyro and acclerometer
    IIC_Read_NByte(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, &rawData[0]); // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    IIC_Read_NByte(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (i =0; i < 3; i++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[i] /= 200;
    gSTAvg[i] /= 200;
  }

   // Configure the gyro and accelerometer for normal operation
   IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, 0x00);
   IIC_Write_One_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, 0x00);
   vTaskDelay(25); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_X_ACCEL_REG); // X-axis accel self-test results
   selfTest[1] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_Y_ACCEL_REG); // Y-axis accel self-test results
   selfTest[2] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_Z_ACCEL_REG); // Z-axis accel self-test results
   selfTest[3] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_X_GYRO_REG); // X-axis gyro self-test results
   selfTest[4] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_Y_GYRO_REG); // Y-axis gyro self-test results
   selfTest[5] = IIC_Read_One_Byte(MPU9250_ADDR, MPU_SELF_TEST_Z_GYRO_REG); // Z-axis gyro self-test results

   for (i = 0; i < 6; i++)
   {
      if (selfTest[i] != 0)
      {
        factoryTrim[i] = mpu6500StTb[selfTest[i] - 1];
      }
      else
      {
        factoryTrim[i] = 0;
      }
    }

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (i = 0; i < 3; i++)
  {
   aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i]) - factoryTrim[i]))/factoryTrim[i]; // Report percent differences
   gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i]) - factoryTrim[i+3]))/factoryTrim[i+3]; // Report percent differences
//   printf("a[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, aAvg[i], aSTAvg[i], aSTAvg[i] - aAvg[i], factoryTrim[i], aDiff[i]);
//   printf("g[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, gAvg[i], gSTAvg[i], gSTAvg[i] - gAvg[i], factoryTrim[i+3], gDiff[i]);
  }

  // Restore old configuration
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_SAMPLE_RATE_REG, saveReg[0]);
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_CFG_REG, saveReg[1]);
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, saveReg[2]);
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG2_REG, saveReg[3]);
  IIC_Write_One_Byte(MPU9250_ADDR, MPU_ACCEL_CFG_REG, saveReg[4]);

   // Check result
  if (MPU_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[0], "gyro X") &&
      MPU_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[1], "gyro Y") &&
      MPU_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[2], "gyro Z") &&
      MPU_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[0], "acc X") &&
      MPU_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[1], "acc Y") &&
      MPU_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[2], "acc Z"))
  {
    return true;
  }
  else
  {
    return false;
  }
}


//初始化MPU9250
//返回值:0,成功
//    其他,错误代码
u8 MPU9250_Init(void)
{
	IIC_Init();     //初始化IIC总线
    vTaskDelay(pdMS_TO_TICKS(10));
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    vTaskDelay(pdMS_TO_TICKS(20));
//    MPU_SelfTest();
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    vTaskDelay(pdMS_TO_TICKS(10));
//	IIC_Write_One_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X04);//
    IIC_Write_One_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X06);//开启温度 设置CLKSEL,PLL X轴为参考
    vTaskDelay(pdMS_TO_TICKS(10));
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计

	IIC_Write_One_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,3);    //陀螺仪传感器,±2000dps
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,3);   //加速度传感器,±2g
    
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,0); //设置采样率1000Hz
	IIC_Write_One_Byte(MPU9250_ADDR,MPU_CFG_REG,0x04);      //设置低通滤波器

//	res=IIC_Read_One_Byte(MPU9250_ADDR,MPU_WHO_AM_I_REG);  //读取MPU9250的ID
//		printf("res:%x\r\n",res);
//	if(res==MPU9250_ID) //器件ID正确
//	{
        IIC_Write_One_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
//        MPU_Set_Rate(50);						       	//设置采样率为50Hz   
//	}else return 1;

	return 0;
}

//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return IIC_Write_One_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return IIC_Write_One_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	IIC_Read_NByte(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(axis3f_t *gyro_s)
{
    u8 buf[6],res; 
	res=IIC_Read_NByte(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		gyro_s->x=((u16)buf[0]<<8)|buf[1];  
		gyro_s->y=((u16)buf[2]<<8)|buf[3];  
		gyro_s->z=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(axis3f_t *acc_s)
{
    u8 buf[6],res;  
	res=IIC_Read_NByte(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		acc_s->x=((u16)buf[0]<<8)|buf[1];  
		acc_s->y=((u16)buf[2]<<8)|buf[3];  
		acc_s->z=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


//发送数据到匿名上位机
//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}


