#include "include.h"
#include "mpu6050.h"
#include "i2c_software.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "imu2.h"
#include "imu.h"


ahrs_t ahrs;
static uint16_t sec = 0;
static uint16_t msec = 0;
//static int16_t sendBuf[PACKET_LENS] = {0};

__IO static uint8_t flag = ERROR;


 void Quaternion_RungeKutta( quaternion_t *pNumQ, const float *gyro, const float helfTime )
{
  float q[4];
  float gyr[3];

  gyr[0] = gyro[0] * helfTime;
  gyr[1] = gyro[1] * helfTime;
  gyr[2] = gyro[2] * helfTime;

  q[0] = -pNumQ->q1 * gyr[0] - pNumQ->q2 * gyr[1] - pNumQ->q3 * gyr[2];
  q[1] =  pNumQ->q0 * gyr[0] - pNumQ->q3 * gyr[1] + pNumQ->q2 * gyr[2];
  q[2] =  pNumQ->q3 * gyr[0] + pNumQ->q0 * gyr[1] - pNumQ->q1 * gyr[2];
  q[3] = -pNumQ->q2 * gyr[0] + pNumQ->q1 * gyr[1] + pNumQ->q0 * gyr[2];

  pNumQ->q0 += q[0];
  pNumQ->q1 += q[1];
  pNumQ->q2 += q[2];
  pNumQ->q3 += q[3];
}


//static float qq0 = 1, qq1 = 0, qq2 = 0, qq3 = 0;

 void Quaternion_Norm( quaternion_t *pNumQ, const quaternion_t *pNumN )
{
  const float norm = invSqrtf(squa(pNumQ->q0) + squa(pNumQ->q1) + squa(pNumQ->q2) + squa(pNumQ->q3));
//  const float norm = invSqrt(squa(pNumQ->q0) + squa(pNumQ->q1) + squa(pNumQ->q2) + squa(pNumQ->q3));

	
  pNumQ->q0 = pNumN->q0 * norm;
  pNumQ->q1 = pNumN->q1 * norm;
  pNumQ->q2 = pNumN->q2 * norm;
  pNumQ->q3 = pNumN->q3 * norm;
	
//	qq0=pNumQ->q0;
//	qq1=pNumQ->q1;
//	qq2=pNumQ->q2;
//	qq3=pNumQ->q3;
	
}

void Quaternion_UpdateRotMatrix( quaternion_t *pNumQ )
{
  const float Mq0q0 = pNumQ->q0 * pNumQ->q0;
  const float Mq1q1 = pNumQ->q1 * pNumQ->q1;
  const float Mq2q2 = pNumQ->q2 * pNumQ->q2;
  const float Mq3q3 = pNumQ->q3 * pNumQ->q3;
  const float Mq0q1 = pNumQ->q0 * pNumQ->q1;
  const float Mq0q2 = pNumQ->q0 * pNumQ->q2;
  const float Mq0q3 = pNumQ->q0 * pNumQ->q3;
  const float Mq1q2 = pNumQ->q1 * pNumQ->q2;
  const float Mq1q3 = pNumQ->q1 * pNumQ->q3;
  const float Mq2q3 = pNumQ->q2 * pNumQ->q3;

  pNumQ->rMat[0][0] = 2.0f * (Mq0q0 + Mq1q1 - 0.5f);
  pNumQ->rMat[1][0] = 2.0f * (Mq1q2 - Mq0q3);
  pNumQ->rMat[2][0] = 2.0f * (Mq1q3 + Mq0q2);
  pNumQ->rMat[0][1] = 2.0f * (Mq1q2 + Mq0q3);
  pNumQ->rMat[1][1] = 2.0f * (Mq0q0 + Mq2q2 - 0.5f);
  pNumQ->rMat[2][1] = 2.0f * (Mq2q3 - Mq0q1);
  pNumQ->rMat[0][2] = 2.0f * (Mq1q3 - Mq0q2);
  pNumQ->rMat[1][2] = 2.0f * (Mq2q3 + Mq0q1);
  pNumQ->rMat[2][2] = 2.0f * (Mq0q0 + Mq3q3 - 0.5f);
}




 void Quaternion_ToAngE( eulerAngle_t *pAngE, const quaternion_t *pNumQ )
{
  pAngE->pitch = toDeg(asinf(pNumQ->rMat[0][2]));
  pAngE->roll  = toDeg(atan2f(pNumQ->rMat[1][2], pNumQ->rMat[2][2]));
  pAngE->yaw   = toDeg(atan2f(pNumQ->rMat[0][1], pNumQ->rMat[0][0]));
	
  rt_kprintf("roll:%d\r\n",(int)pAngE->roll);	
}


void IMU_InitData( imu_t *imux )
{
  if (imux->calibState != ENABLE) {
    memset(imux, 0, sizeof(imu_t));

    imux->gyrCalib[0] = 1.0f;
    imux->gyrCalib[1] = 1.0f;
    imux->gyrCalib[2] = 1.0f;
    imux->accStrength = 1.0f;
    imux->accCalib[0] = 1.0f;
    imux->accCalib[4] = 1.0f;
    imux->accCalib[8] = 1.0f;
    imux->magStrength = 1.0f;
    imux->magCalib[0] = 1.0f;
    imux->magCalib[4] = 1.0f;
    imux->magCalib[8] = 1.0f;

#if 0
    /* set gyroscope parameters */
    imux->gyrOffset[0] = -5.266666666666667f;
    imux->gyrOffset[1] = -7.883333333333334f;
    imux->gyrOffset[2] = -8.783333333333333f;

    /* set accelerometer parameters */
    imux->accStrength  =  9.8f;
    imux->accCalib[0]  =  1.000591271010064f;
    imux->accCalib[1]  =  0.000586893913887f;
    imux->accCalib[2]  = -0.003436732180952f;
    imux->accCalib[3]  = -0.001485988936797f;
    imux->accCalib[4]  =  1.000157846160545f;
    imux->accCalib[5]  = -0.011743938010525f;
    imux->accCalib[6]  = -0.007978426891497f;
    imux->accCalib[7]  = -0.000416245697376f;
    imux->accCalib[8]  =  0.995341160302247f;
    imux->accOffset[0] =  42.55588470081017f;
    imux->accOffset[1] = -97.96568104396205f;
    imux->accOffset[2] =  316.0875468994960f;

    /* set magnetometer parameters */
    imux->magStrength  =  212.4818615767788f;
    imux->magCalib[0]  =  1.019823067566470f;
    imux->magCalib[1]  = -0.003356411418474f;
    imux->magCalib[2]  = -0.014430324672665f;
    imux->magCalib[3]  = -0.003356411418474f;
    imux->magCalib[4]  =  1.015454920456150f;
    imux->magCalib[5]  =  0.018734747904013f;
    imux->magCalib[6]  = -0.014430324672665f;
    imux->magCalib[7]  =  0.018734747904013f;
    imux->magCalib[8]  =  0.966196977587016f;
    imux->magOffset[0] =  16.228349462609184f;
    imux->magOffset[1] =  170.7384291607371f;
    imux->magOffset[2] = -58.094694639781515f;
#endif
  }
}

void AHRS_Update( ahrs_t *ahrs,float gx, float gy, float gz, float ax, float ay, float az )
{
  float gVect[3];
  float gyr[3], acc[3];
float normAcc;
#if defined(MAHONY_FILTER_9DOF)
  float32_t hVect[3], wVect[3];
  float32_t mag[3];
#endif

  float err[3];
#if defined(MF_KI)
  static float32_t errInt[3] = {0};
#endif

//  gyr[0] = toRad(ahrs->imu.gyrData[0]);
//  gyr[1] = toRad(ahrs->imu.gyrData[1]);
//  gyr[2] = toRad(ahrs->imu.gyrData[2]);
//	
//  acc[0] = ahrs->imu.accData[0];
//  acc[1] = ahrs->imu.accData[1];
//  acc[2] = ahrs->imu.accData[2];
	
//  gyr[0] = toRad(gx)*GYRO_FS_2000;
//  gyr[1] = toRad(gy)*GYRO_FS_2000;
//  gyr[2] = toRad(gz)*GYRO_FS_2000;
//	
//  acc[0] = ax*ACCEL_FS_8;
//  acc[1] = ay*ACCEL_FS_8;
//  acc[2] = az*ACCEL_FS_8;



  gyr[0] = gx;
  gyr[1] = gy;
  gyr[2] = gz;
  acc[0] = ax;
  acc[1] = ay;
  acc[2] = az;	
	
//		rt_kprintf("gVect:%d\r\n",(int)acc[0]);	
	
  /* Normalise accelerometer measurement */
//  normAcc = invSqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
   normAcc = invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  
  
// rt_kprintf("gVect:%d\r\n",(int)(normAcc*100000));	


	
  acc[0] *= normAcc;
  acc[1] *= normAcc;
  acc[2] *= normAcc;
//rt_kprintf("gVect:%d\r\n",(int)acc[0]);	
  /* Estimated direction of gravity */
  gVect[0] = ahrs->numQ.rMat[0][2];
  gVect[1] = ahrs->numQ.rMat[1][2];
  gVect[2] = ahrs->numQ.rMat[2][2];

//  gVect[0] =  qq1 * qq3 - qq0 * qq2;
//  gVect[1] = qq0 * qq1 + qq2 * qq3;
//  gVect[2] =qq0 * qq0 - 0.5f + qq3 * qq3;

//halfvx = q1 * q3 - q0 * q2;
//			halfvy = q0 * q1 + q2 * q3;
//			halfvz = q0 * q0 - 0.5f + q3 * q3;
		

  err[0] = acc[1] * gVect[2] - acc[2] * gVect[1];
  err[1] = acc[2] * gVect[0] - acc[0] * gVect[2];
  err[2] = acc[0] * gVect[1] - acc[1] * gVect[0];

#if defined(MAHONY_FILTER_9DOF)
  mag[0] = ahrs->imu.magData[0];
  mag[1] = ahrs->imu.magData[1];
  mag[2] = ahrs->imu.magData[2];

//  const float32_t normMag = invSqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
//  if ((normMag < fusionAhrs->minMagneticFieldSquared) ||
//      (normMag > fusionAhrs->maxMagneticFieldSquared)) {
//    break;
//  }

  /* Reference direction of Earth's magnetic field */
  wVect[0] = acc[1] * mag[2] - acc[2] * mag[1];
  wVect[1] = acc[2] * mag[0] - acc[0] * mag[2];
  wVect[2] = acc[0] * mag[1] - acc[1] * mag[0];

  /* Normalise magnetometer measurement */
  const float32_t normVect = invSqrtf(wVect[0] * wVect[0] + wVect[1] * wVect[1] + wVect[2] * wVect[2]);
  wVect[2] *= normVect;
  wVect[2] *= normVect;
  wVect[2] *= normVect;

  /* Estimated direction of Earth's magnetic field  */
  hVect[0] = ahrs->numQ.rMat[0][1];
  hVect[1] = ahrs->numQ.rMat[1][1];
  hVect[2] = ahrs->numQ.rMat[2][1];

  /* Error is sum of cross product between estimated direction and measured direction of field vectors */
  err[0] += wVect[1] * hVect[2] - wVect[2] * hVect[1];
  err[1] += wVect[2] * hVect[0] - wVect[0] * hVect[2];
  err[2] += wVect[0] * hVect[1] - wVect[1] * hVect[0];

#endif

#if defined(MF_KI)
  /* Compute and apply integral feedback */
  errInt[0] += (ahrs->sampleTime * MF_KI) * err[0];
  errInt[1] += (ahrs->sampleTime * MF_KI) * err[1];
  errInt[2] += (ahrs->sampleTime * MF_KI) * err[2];

  /* Apply proportional feedback */
  gyr[0] += MF_KP * err[0] + errInt[0];
  gyr[1] += MF_KP * err[1] + errInt[1];
  gyr[2] += MF_KP * err[2] + errInt[2];

#else
  gyr[0] += MF_KP * err[0];
  gyr[1] += MF_KP * err[1];
  gyr[2] += MF_KP * err[2];

#endif

  /* Integrate rate of change of quaternion */
  Quaternion_RungeKutta(&ahrs->numQ, gyr, ahrs->helfSampleTime);
  Quaternion_Norm(&ahrs->numQ, &ahrs->numQ);
  Quaternion_UpdateRotMatrix(&ahrs->numQ);
  Quaternion_ToAngE(&ahrs->angE, &ahrs->numQ);
}	
	




/**
  * @brief  AHRS_GyroBiasCorrection
  */

void AHRS_GyroBiasCorrection( int16_t *gyro, float *bias )
{
  static const float alpha = TIME_CONST / (TIME_CONST + SAMPLE_TIME);
  static const float alpha_n = 1.0f - alpha;
  static uint16_t stirlessTime = 0;

  if ((gyro[0] > THRESHOLD) || (gyro[0] < -THRESHOLD) ||
      (gyro[1] > THRESHOLD) || (gyro[1] < -THRESHOLD) ||
      (gyro[2] > THRESHOLD) || (gyro[2] < -THRESHOLD)) {
    stirlessTime = 0.0f;
  }
  else {
    if (stirlessTime >= STIRLESS_TIME) {
      bias[0] = alpha * bias[0] + alpha_n * gyro[0];
      bias[1] = alpha * bias[1] + alpha_n * gyro[1];
      bias[2] = alpha * bias[2] + alpha_n * gyro[2];
    }
    else {
      stirlessTime++;
    }
  }
}


int8_t IMU_GetData_6050( imu_t *imux,float gx, float gy, float gz, float ax, float ay, float az  )
{
  imux->gyrRaw[0] = gx;    /* Gyr.X */
  imux->gyrRaw[1] = gy;    /* Gyr.Y */
  imux->gyrRaw[2] = gz;    /* Gyr.Z */
  imux->accRaw[0] = ax;    /* Acc.X */
  imux->accRaw[1] = ay;    /* Acc.Y */
  imux->accRaw[2] = az;    /* Acc.Z */

}


/**
  * @brief  IMU_GetRawData
  */
int8_t IMU_GetRawData( imu_t *imux )
{
  int8_t status;

#if defined(__MPU92)
  int16_t data16[10];
#endif

#if defined(__LPS22)
  int32_t data32[2];
#endif

#if defined(__MPU92)
  status = MPU92_GetRawData(data16);
  imux->ictempRaw = data16[0];    /* ICTemp */
  imux->gyrRaw[0] = data16[1];    /* Gyr.X */
  imux->gyrRaw[1] = data16[2];    /* Gyr.Y */
  imux->gyrRaw[2] = data16[3];    /* Gyr.Z */
  imux->accRaw[0] = data16[4];    /* Acc.X */
  imux->accRaw[1] = data16[5];    /* Acc.Y */
  imux->accRaw[2] = data16[6];    /* Acc.Z */

#if defined(__USE_MAGNETOMETER)
  if (status == 1) {
    imux->magRaw[0] = data16[7];  /* Mag.X */
    imux->magRaw[1] = data16[8];  /* Mag.Y */
    imux->magRaw[2] = data16[9];  /* Mag.Z */
  }
#endif

#endif

#if defined(__LPS22)
  status = LPS22_GetRawData(data32);
  if (status == 1) {
    imux->baroRaw[0] = data32[0];
    imux->baroRaw[1] = data32[1];
  }
#endif

  return status;
}



/**
  * @brief  IMU_GetCalibData
  */
void IMU_GetCalibData( imu_t *imux )
{
  float tmp[3] = {0};

//  IMU_GetRawData(imux);

  IMU_GetData_6050(imux,Gyro[0]+30,Gyro[1]+3,Gyro[2]+18,Accel[0],Accel[1],Accel[2]);
  
#if defined(__USE_GYROSCOPE)
//  tmp[0] = imux->gyrRaw[0] - imux->gyrOffset[0];  /* Gyr.X */
//  tmp[1] = imux->gyrRaw[1] - imux->gyrOffset[1];  /* Gyr.Y */
//  tmp[2] = imux->gyrRaw[2] - imux->gyrOffset[2];  /* Gyr.Z */
//  imux->gyrData[0] = imux->gyrCalib[0] * tmp[0];
//  imux->gyrData[1] = imux->gyrCalib[1] * tmp[1];
//  imux->gyrData[2] = imux->gyrCalib[2] * tmp[2];
  imux->gyrData[0] = imux->gyrRaw[0] - imux->gyrOffset[0];  /* Gyr.X */
  imux->gyrData[1] = imux->gyrRaw[1] - imux->gyrOffset[1];  /* Gyr.Y */
  imux->gyrData[2] = imux->gyrRaw[2] - imux->gyrOffset[2];  /* Gyr.Z */
  imux->gyrInt[0]  = imux->gyrData[0];
  imux->gyrInt[1]  = imux->gyrData[1];
  imux->gyrInt[2]  = imux->gyrData[2];
#endif

#if defined(__USE_ACCELEROMETER)
  tmp[0] = imux->accRaw[0] - imux->accOffset[0];  /* Acc.X */
  tmp[1] = imux->accRaw[1] - imux->accOffset[1];  /* Acc.Y */
  tmp[2] = imux->accRaw[2] - imux->accOffset[2];  /* Acc.Z */
  imux->accData[0] = imux->accCalib[0] * tmp[0] + imux->accCalib[1] * tmp[1] + imux->accCalib[2] * tmp[2];
  imux->accData[1] = imux->accCalib[3] * tmp[0] + imux->accCalib[4] * tmp[1] + imux->accCalib[5] * tmp[2];
  imux->accData[2] = imux->accCalib[6] * tmp[0] + imux->accCalib[7] * tmp[1] + imux->accCalib[8] * tmp[2];
  imux->accInt[0]  = imux->accData[0];
  imux->accInt[1]  = imux->accData[1];
  imux->accInt[2]  = imux->accData[2];
#endif

#if defined(__USE_MAGNETOMETER)
  tmp[0] = imux->magRaw[0] - imux->magOffset[0];  /* Mag.X */
  tmp[1] = imux->magRaw[1] - imux->magOffset[1];  /* Mag.Y */
  tmp[2] = imux->magRaw[2] - imux->magOffset[2];  /* Mag.Z */
  imux->magData[0] = imux->magCalib[0] * tmp[0] + imux->magCalib[1] * tmp[1] + imux->magCalib[2] * tmp[2];
  imux->magData[1] = imux->magCalib[3] * tmp[0] + imux->magCalib[4] * tmp[1] + imux->magCalib[5] * tmp[2];
  imux->magData[2] = imux->magCalib[6] * tmp[0] + imux->magCalib[7] * tmp[1] + imux->magCalib[8] * tmp[2];
  imux->magInt[0]  = imux->magData[0];
  imux->magInt[1]  = imux->magData[1];
  imux->magInt[2]  = imux->magData[2];
#endif

#if defined(__USE_ICTEMPERATURE)
  imux->ictempData = imux->ictempRaw * imux->ictempScale + imux->ictempOffset;
  imux->ictempInt  = imux->ictempData;
#endif

#if defined(__USE_BAROMETER)
  imux->baroData[0] = imux->baroRaw[0];
  imux->baroData[1] = imux->baroRaw[1];
  imux->baroInt[0]  = imux->baroData[0];
  imux->baroInt[1]  = imux->baroData[1];
#endif
}




/**
  * @brief  IMU_GetRealData
  */
void IMU_GetRealData( imu_t *imux )
{
  IMU_GetCalibData(imux);

#if defined(__USE_GYROSCOPE)
  imux->gyrData[0] = imux->gyrData[0] * imux->gyrFactor[0]; /* Gyr.X */
  imux->gyrData[1] = imux->gyrData[1] * imux->gyrFactor[1]; /* Gyr.Y */
  imux->gyrData[2] = imux->gyrData[2] * imux->gyrFactor[2]; /* Gyr.Z */
#endif

#if defined(__USE_ACCELEROMETER)
  imux->accData[0] = imux->accData[0] * imux->accFactor[0]; /* Acc.X */
  imux->accData[1] = imux->accData[1] * imux->accFactor[1]; /* Acc.Y */
  imux->accData[2] = imux->accData[2] * imux->accFactor[2]; /* Acc.Z */
#endif

#if defined(__USE_MAGNETOMETER)
  imux->magData[0] = imux->magData[0] * imux->magFactor[0]; /* Mag.X */
  imux->magData[1] = imux->magData[1] * imux->magFactor[1]; /* Mag.Y */
  imux->magData[2] = imux->magData[2] * imux->magFactor[2]; /* Mag.Z */
#endif

#if defined(__USE_BAROMETER)
  imux->baroData[0] = imux->baroData[0] * imux->baroFactor[0];  /* Pressure */
  imux->baroData[1] = imux->baroData[1] * imux->baroFactor[1];  /* Temperature */
#endif
}

static void IMU_UpdateDataFactorInit(void)
{


}


 void IMU_UpdateDataFactor( imu_t *imux )
{
	imux->accStrength = 1.0f;
	imux->gyrScale[0]=2000.0/32768;
	imux->gyrScale[1]=2000.0/32768;
	imux->gyrScale[2]=2000.0/32768;
	
	imux->accScale[0] = 4.0/32768;
	imux->accScale[1] = 4.0/32768;
	imux->accScale[2] = 4.0/32768;
	
#if defined(__USE_GYROSCOPE)
  /* Combine gyroscope scale and sensitivity (dps/LSB) */
  imux->gyrFactor[0] = imux->gyrScale[0];
  imux->gyrFactor[1] = imux->gyrScale[1];
  imux->gyrFactor[2] = imux->gyrScale[2];
#endif

#if defined(__USE_ACCELEROMETER)
  /* Combine accelerometer scale and sensitivity (g/LSB) */
  imux->accFactor[0] = imux->accScale[0] * imux->accStrength;
  imux->accFactor[1] = imux->accScale[1] * imux->accStrength;
  imux->accFactor[2] = imux->accScale[2] * imux->accStrength;
#endif

#if defined(__USE_MAGNETOMETER)
  /* Combine magnetometer scale and sensitivity (uT/LSB) */
  imux->magFactor[0] = imux->magScale[0] * imux->accStrength;
  imux->magFactor[1] = imux->magScale[1] * imux->accStrength;
  imux->magFactor[2] = imux->magScale[2] * imux->accStrength;
#endif

#if defined(__USE_BAROMETER)
  /* Combine barometer scale and sensitivity (hPa/LSB) (degC/LSB) */
  imux->baroFactor[0] = imux->baroScale[0];
  imux->baroFactor[1] = imux->baroScale[1];
#endif
}

//void MPU92_GetSensitivity( MPU_ConfigTypeDef *mpux, float *sensitivity )
//{
//  double scale;

//  /* Set gyroscope sensitivity (dps/LSB) */
//#if defined(__USE_GYROSCOPE)
//  switch (mpux->MPU_Gyr_FullScale) {
//    case MPU_GyrFS_250dps:    scale = 250.0;    break;
//    case MPU_GyrFS_500dps:    scale = 500.0;    break;
//    case MPU_GyrFS_1000dps:   scale = 1000.0;   break;
//    case MPU_GyrFS_2000dps:   scale = 2000.0;   break;
//    default:                  scale = 0.0;      break;
//  }
//  sensitivity[0] = scale / 32768;
//#else
//  sensitivity[0] = 0.0f;
//#endif

//  /* Set accelerometer sensitivity (g/LSB) */
//#if defined(__USE_ACCELEROMETER)
//  switch (mpux->MPU_Acc_FullScale) {
//    case MPU_AccFS_2g:    scale = 2.0;    break;
//    case MPU_AccFS_4g:    scale = 4.0;    break;
//    case MPU_AccFS_8g:    scale = 8.0;    break;
//    case MPU_AccFS_16g:   scale = 16.0;   break;
//    default:              scale = 0.0;    break;
//  }
//  sensitivity[1] = scale / 32768;
//#else
//  sensitivity[1] = 0.0f;
//#endif

//  /* Set magnetometer sensitivity (uT/LSB) */
//#if defined(__USE_MAGNETOMETER)
//  sensitivity[2] = 0.6;   /* +-4800uT */
//#else
//  sensitivity[2] = 0.0f;
//#endif

//  /* Set ictemperature sensitivity (degC/LSB) */
//#if defined(__USE_ICTEMPERATURE)
//  sensitivity[3] = 1.0 / 333.87;
//  sensitivity[4] = 21.0;
//#else
//  sensitivity[3] = 0.0f;
//  sensitivity[4] = 0.0f;
//#endif
//}

void AHRS_update(void)
{
//IMU_GetRealData(&ahrs.imu);
//  AHRS_GyroBiasCorrection(ahrs.imu.gyrRaw, ahrs.imu.gyrOffset);		
	AHRS_Update(&ahrs,Gyro[0]+30,Gyro[1]+3,Gyro[2]+18,Accel[0],Accel[1],Accel[2]);	

}
