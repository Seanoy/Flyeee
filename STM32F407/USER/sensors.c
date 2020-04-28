#include "sensors.h"
#include "bsp_mpu6500.h"
#include "bsp_ak8963.h"
#include "imu.h"
#include "bsp_myiic.h"

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ    80
#define ACCEL_LPF_CUTOFF_FREQ   30
#define MAG_LPF_CUTOFF_FREQ     20

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static lpf2pData magLpf[3];
u8 rawDataBuf[64];
static axis3f_t gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

sensorData_t sensors;

static Axis3i16 gyroRaw;
static Axis3i16 accRaw;
static Axis3i16 magRaw;

typedef struct
{
	axis3f_t     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

BiasObj	gyroBiasRunning;

void Filter_Init(void)
{
    for (u8 i = 0; i < 3; i++)// 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
        lpf2pInit(&magLpf[i],  1000, MAG_LPF_CUTOFF_FREQ);
	}
}
/*上位机获取读取原始数据*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag)
{
	*acc = accRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
}
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}
/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}
/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, axis3f_t* varOut, axis3f_t* meanOut)
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		axis3f_t mean;
		axis3f_t variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}
/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, axis3f_t *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}
/*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) 
	{
		int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
		int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
		int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

		sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
		sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
		sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;		
		magRaw.x = headingx;/*用于上传到上位机*/
		
		magRaw.y = headingy;
		magRaw.z = headingz;
	}
}
/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	accRaw.x = ax;/*用于上传到上位机*/
	accRaw.y = ay;
	accRaw.z = az;
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
//	applyAxis3fLpf(gyroLpf, &sensors.gyro);	//低通滤波器有问题，需要改

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;
//	applyAxis3fLpf(accLpf, &sensors.acc);
}

/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

void sensorInit(void)
{
//    //创建二值信号量
    sensorsDataReady = xSemaphoreCreateBinary();
    sensorsBiasObjInit(&gyroBiasRunning);
//	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(axis3f_t));
	gyroDataQueue = xQueueCreate(1, sizeof(axis3f_t));
	magnetometerDataQueue = xQueueCreate(1, sizeof(axis3f_t));
//	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}

/*从队列读取陀螺数据*/
bool sensorsReadGyro(axis3f_t *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(axis3f_t *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(axis3f_t *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)	
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
//	sensorsReadBaro(&sensors->baro);
    xSemaphoreGive(sensorsDataReady);
}

//获取初始数据
void processSensordata(void)
{
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
        IIC1_Read_NByte(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,14,rawDataBuf);
        processAccGyroMeasurements(&(rawDataBuf[0]));//获取并处理陀螺仪和加速度计数据
        
        IIC1_Read_NByte(AK8963_ADDR,AK8963_XOUT_L,6,rawDataBuf+14);
        processMagnetometerMeasurements(&(rawDataBuf[14]));//获取并处理磁力计数据
        vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
        xQueueOverwrite(accelerometerDataQueue, &sensors.acc);//九轴数据写入队列，等待数据处理任务处理
        xQueueOverwrite(gyroDataQueue, &sensors.gyro);
        xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
//        xQueueOverwrite(barometerDataQueue, &sensors.baro);//气压数据
        xTaskResumeAll();//恢复任务
//        ANO_Send_01(accRaw.x, accRaw.y, accRaw.z, gyroRaw.x, gyroRaw.y, gyroRaw.z,0);//发送数据到匿名上位机处理
//        ANO_Send_02(magRaw.x, magRaw.y, magRaw.z, 0,0,0,0);
    }
}


//获取数据任务函数 
void sensor_task(void *pvParameters)
{    
    sensorInit();
    //filter init
    Filter_Init();
    //imu init
    MPU9250_Init();
    //magnitude sensor init
    AK8963_Init();
    //barometric sensor init
//    BMP_Init();
    
    while(1)
    {
//        LED0=~LED0;
        //处理传感器数据，获取姿态角
        processSensordata();
//        vTaskDelay(500);
    }
}
