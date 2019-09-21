#ifndef __IMU_H
#define __IMU_H
#include "include.h"

 
//#define twoKpDef	  (2.0f * 0.9f)	// 2 * proportional gain
//#define twoKiDef	(2.0f * 0.01f)	// 2 * integral gain

#define twoKpDef	0.01f	// 2 * proportional gain
#define twoKiDef	0.02f	// 2 * integral gain

#define UPDATE_LAST_TIME(currentTv,lastTv) do{ lastTv.tv_usec = currentTv.tv_usec; lastTv.tv_sec = currentTv.tv_sec; }while(0)
#define TIME_IS_UPDATED(tv) (tv.tv_usec>0?1:0)
#define GET_SEC_TIMEDIFF(currentTv,lastTv) ((float) (currentTv.tv_sec - lastTv.tv_sec)+(float) (currentTv.tv_usec - lastTv.tv_usec) * 0.000001f)

#define __MINGW_NOTHROW __attribute__((__nothrow__))

//int __cdecl __MINGW_NOTHROW gettimeofday
//(struct timeval *__restrict__, void *__restrict__ /* tzp (unused) */);
#define  ACCEL_FS_8   0.000244140625f
#define  GYRO_FS_2000 0.0609756097560f

#define DE_TO_RA 0.01745329251f  // PI/180

#define PITCH_BALANCE (-1.5f)
#define ROLL_BALANCE  (2.5f)


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) ;

void IMUupdate_new(float gyro[3] , float acc[3],float angle[3]); 
void IMUupdate_new_second(float gyro[3] , float acc[3],float angle[3]) ;
float invSqrt(float x);
#endif  /*imu*/
