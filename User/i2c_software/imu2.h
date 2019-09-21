#ifndef __IMU2_H
#define __IMU2_H
#include "include.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MF_KP 0.4f
//#define MF_KI 0.001f


//#define __MPU92
#define __USE_GYROSCOPE
#define __USE_ACCELEROMETER
//#define __USE_MAGNETOMETER
//#define __USE_ICTEMPERATURE




#define invSqrtf( __iSq )  (1.0f / sqrtf((float)(__iSq)))
#define squa( __Sq )       (((float)(__Sq)) * ((float)(__Sq)))
#define toRad( __mathD )   ((__mathD) * 0.0174532925f)
#define toDeg( __mathR )   ((__mathR) * 57.2957795f)
#define absInt( __n )      (((__n) > 0) ? (__n) : -(__n))
#define arrayLens(__arr)   (sizeof(__arr) / sizeof((__arr)[0]))



#define DELTA_Q31          (0x100)
#define DELTA_Q15          0x5
#define INDEX_MASK         0x0000003F
#define SAMPLE_RATE   ((uint16_t)1000)      // 1KHz
#define SAMPLE_TIME   ((float)0.001f)   // 1.0ms

#define PI_MATH                 3.14159265358979f

#define CUTOFF_FREQ     (0.02f)
#define TIME_CONST      (1.0f / (2.0f * PI_MATH * CUTOFF_FREQ))
#define THRESHOLD       (30)
#define STIRLESS_TIME   (SAMPLE_RATE << 1)  // 2 second

typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
  float rMat[3][3];
} __attribute__((aligned(4))) quaternion_t;

typedef struct {
  float pitch;
  float roll;
  float yaw;
} __attribute__((aligned(4))) eulerAngle_t;

typedef struct {
  int8_t calibState;

  int16_t gyrRaw[3];      /* x = raw[0], y = raw[1], z = raw[2] */
  int16_t accRaw[3];
  int16_t magRaw[3];
  int16_t ictempRaw;
  int32_t baroRaw[2];     /* p = raw[0], t = raw[1] */

  int16_t gyrInt[3];
  int16_t accInt[3];
  int16_t magInt[3];
  int16_t ictempInt;
  int32_t baroInt[2];

  float gyrData[3];
  float accData[3];
  float magData[3];
  float ictempData;
  float baroData[2];

  float accMotion[3];

  float gyrScale[3];
  float accScale[3];
  float magScale[3];
  float ictempScale;
  float baroScale[2];

  float gyrFactor[3];
  float accFactor[3];
  float magFactor[3];
  float baroFactor[2];

  float gyrCalib[3];
  float accCalib[9];
  float magCalib[9];    /* c11 = calib[0], c12 = calib[1], c13 = calib[2],
                               c21 = calib[3], c22 = calib[4], c23 = calib[5],
                               c31 = calib[6], c32 = calib[7], c33 = calib[8] */
  float gyrOffset[3];
  float accOffset[3];
  float magOffset[3];
  float ictempOffset;

  float accStrength;
  float magStrength;

} __attribute__((aligned)) imu_t;
typedef struct {
  float sampleTime;
  float helfSampleTime;
  quaternion_t numQ;
  eulerAngle_t angE;
  imu_t imu;
} __attribute__((aligned)) ahrs_t;

extern ahrs_t ahrs;


void AHRS_Update( ahrs_t *ahrs,float gx, float gy, float gz, float ax, float ay, float az );
void    IMU_Config( void );
//int8_t  IMU_Init( IMU_InitTypeDef *imux );
void    IMU_InitData( imu_t *imux );
int8_t  IMU_GetRawData( imu_t *imux );
void    IMU_GetCalibData( imu_t *imux );
void    IMU_GetRealData( imu_t *imux );
void    IMU_PrintData( imu_t *imux );

void AHRS_GyroBiasCorrection( int16_t *gyro, float *bias );

void IMU_UpdateDataFactor( imu_t *imux );

void AHRS_update(void);
#endif  /*imu*/
