#include "include.h"
#include "mpu6050.h"
#include "i2c_software.h"
#include "imu.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//int __cdecl __MINGW_NOTHROW gettimeofday
//(struct timeval *__restrict__, void *__restrict__ /* tzp (unused) */);

struct timeval
{ /* Nominally a BSD or POSIX.1 structure, (with tv_sec declared as
   * time_t), but subverted by Microsoft with tv_sec declared as long,
   * to avoid __time32_t vs. __time64_t ambiguity; (tv_sec is ALWAYS
   * a 32-bit entity in Windows' use of this structure).  Addionally,
   * POSIX.1-2001 mandates that tv_usec should be suseconds_t, (which
   * is nominally an alias for long), but we retain long to maintain
   * consistency with Microsoft usage.
   */
  long tv_sec;		/* whole number of seconds in interval */
  long tv_usec; 	/* additional fraction as microseconds */
};

static struct timeval last_tv;



volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki






void getMotion6(float* gx, float* gy,float* gz, float* ax, float* ay, float* az) 
{
	short sax = 0;
	short say = 0;
	short saz = 0;
	short sgx = 0;
	short sgy = 0;
	short sgz = 0;

//	getMotion6RawData(&sax, &say, &saz, &sgx, &sgy, &sgz);

	*ax = (float) sax * ACCEL_FS_8;
	*ay = (float) say * ACCEL_FS_8;
	*az = (float) saz * ACCEL_FS_8;
	*gx = (float) sgx * GYRO_FS_2000 * DE_TO_RA; // rad/sec
	*gy = (float) sgy * GYRO_FS_2000 * DE_TO_RA; // rad/sec
	*gz = (float) sgz * GYRO_FS_2000 * DE_TO_RA; // rad/sec
}



//#define q30     1073741824.0f
#define q30     1.0f
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
//	float q[4];
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float timeDiff = 0.f;
	struct timeval tv;
	
//	getMotion6(&gx, &gy, &gz, &ax, &ay,&az);
	


//	tv.tv_sec=1;
//	tv.tv_usec=5;
	

	
//	gettimeofday(&tv, NULL);

	if (TIME_IS_UPDATED(last_tv)) {

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

//			timeDiff = GET_SEC_TIMEDIFF(tv,last_tv);
			
			timeDiff=0.001f;
			
			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;		
	
			// Estimated direction of gravity and vector perpendicular to magnetic flux
			halfvx = q1 * q3 - q0 * q2;
			halfvy = q0 * q1 + q2 * q3;
			halfvz = q0 * q0 - 0.5f + q3 * q3;
		
			// Error is sum of cross product between estimated and measured direction of gravity
			halfex = (ay * halfvz - az * halfvy);
			halfey = (az * halfvx - ax * halfvz);
			halfez = (ax * halfvy - ay * halfvx);
	
			// Compute and apply integral feedback if enabled
			if(twoKiDef > 0.0f) {
				integralFBx += twoKiDef * halfex * (timeDiff);	// integral error scaled by Ki
				integralFBy += twoKiDef * halfey * (timeDiff);
				integralFBz += twoKiDef * halfez * (timeDiff);
				gx += integralFBx;	// apply integral feedback
				gy += integralFBy;
				gz += integralFBz;
			}
			else {
				integralFBx = 0.0f; // prevent integral windup
				integralFBy = 0.0f;
				integralFBz = 0.0f;
			}
	
			// Apply proportional feedback
			gx += twoKpDef * halfex;
			gy += twoKpDef * halfey;
			gz += twoKpDef * halfez;
		}
		
		// Integrate rate of change of quaternion
		gx *= (0.5f * timeDiff); 	// pre-multiply common factors
		gy *= (0.5f * timeDiff);
		gz *= (0.5f * timeDiff);
		qa = q0;
		qb = q1;
		qc = q2;
		q0 += (-qb * gx - qc * gy -q3 * gz);
		q1 += (qa * gx + qc * gz - q3 * gy);
		q2 += (qa * gy - qb * gz + q3 * gx);
		q3 += (qa * gz + qb * gy - qc * gx); 
		
		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		
		
		q0=  q0/q30;
		q1 = q1/q30;
		q2 = q2/q30;
		q3 = q3/q30;

		Angle[ROLL]=asin(-2*q1*q3 + 2*q0*q2) * 57.3;
		Angle[PITCH] = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.3;  
        Angle[YAW] = atan2(2*(q1*q2+q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3;  

        rt_kprintf("roll:%d\r\n",(int)Angle[ROLL]);
	}

	UPDATE_LAST_TIME(tv,last_tv);
	
}
		

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));

	return y;
	}

	
	//快速平方根算法
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度
#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
void IMUupdate_new(float gyro[3] , float acc[3],float angle[3]) 
{	
	float gx,gy,  gz,  ax,  ay,  az;
	
	gx = gyro[0];
	gy = gyro[1];
	gz = gyro[2];
	ax = acc[0];
	ay = acc[1];
	az = acc[2];
	
//	float Kp =10.0f ;
//	float Ki =0.008f ;
//		
	float Kp =80.0f ;
	float Ki =50.0f ;

	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
	static float exInt = 0, eyInt = 0, ezInt = 0;
	float norm;
	float vx, vy, vz;// wx, wy, wz;	
	float ex, ey, ez;

	float halfT =0.001f ;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	 {
			
		
			
			// Normalise accelerometer measurement
//			norm = sqrt(ax * ax + ay * ay + az * az);
			norm = my_sqrt(ax * ax + ay * ay + az * az);
		 
			norm = 1/norm;
			ax *= norm;
			ay *= norm;
			az *= norm;		
	
			// Estimated direction of gravity and vector perpendicular to magnetic flux
			vx = (q1 * q3 - q0 * q2)*2;
			vy = (q0 * q1 + q2 * q3)*2;
			vz = q0 * q0 - q1*q1-q2*q2 + q3 * q3;
		
			// Error is sum of cross product between estimated and measured direction of gravity
			ex = (ay * vz - az * vy);
			ey = (az * vx - ax * vz);
			ez = (ax * vy - ay * vx);
	
			// Compute and apply integral feedback if enabled
//		    exInt = exInt + ex * Ki*halfT;
//			eyInt = eyInt + ey * Ki*halfT;
//			ezInt = ezInt + ez * Ki*halfT;
	
	        exInt = exInt + ex * Ki;
			eyInt = eyInt + ey * Ki;
			ezInt = ezInt + ez * Ki;
			
//	        exInt = LIMIT(exInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
//			eyInt = LIMIT(eyInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
//			ezInt = LIMIT(ezInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
	
			// Apply proportional feedback
			gx = gx + Kp*ex + exInt; 
			gy = gy + Kp*ey + eyInt; 
			gz = gz + Kp*ez + ezInt; 
		}
		
		// Integrate rate of change of quaternion
	
		q0 += (-q1 * gx - q2 * gy -q3 * gz)*halfT;
		q1 += (q0 * gx + q2 * gz - q3 * gy)*halfT;
		q2 += (q0 * gy - q1 * gz + q3 * gx)*halfT;
		q3 += (q0 * gz + q1 * gy - q2 * gx)*halfT; 
		
		// Normalise quaternion
//		norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			norm = my_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		
		norm = 1/norm;
		q0 *= norm;
		q1 *= norm;
		q2 *= norm;
		q3 *= norm;
				

		angle[PITCH]=asin(-2*q1*q3 + 2*q0*q2) * 57.3;
		angle[ROLL] = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.3;  
        angle[YAW] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1) * 57.3;  

		angle[PITCH]+=PITCH_BALANCE;
		angle[ROLL]+=ROLL_BALANCE;
		
		angle[PITCH]*=1.2;
		angle[ROLL]*=1.2;
		
//		printf("Target:%f   measure:%f      PwmOut:%f\r\n",measure.Angle[ROLL],measure.Angle[PITCH],measure.Angle[YAW]);
// Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* Rad; // yaw
//Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)* Rad; // pitch
//Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * Rad; //
//roll
//if(Q_ANGLE.X>90||Q_ANGLE.X<-90)
//{
//if(Q_ANGLE.Y>0)
//Q_ANGLE.Y=180-Q_ANGLE.Y;
//if(Q_ANGLE.Y<0)
//Q_ANGLE.Y=-(180+Q_ANGLE.Y);
  	   	
}
		
void IMUupdate_new_second(float gyro[3] , float acc[3],float angle[3]) 
{
	
	float gx,gy,  gz,  ax,  ay,  az;
	
	gx = gyro[1];
	gy = gyro[0];
	gz = gyro[2];
	ax = acc[1];
	ay = acc[0];
	az = acc[2];
	
//	float Kp =10.0f ;
//	float Ki =0.008f ;
//		
//	float Kp =80.0f ;
//	float Ki =50.0f ;
	
	float Kp =80.0f ;
	float Ki =50.1f ;
	
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
	static float exInt = 0, eyInt = 0, ezInt = 0;
	float norm;
	float vx, vy, vz;// wx, wy, wz;	
	float ex, ey, ez;

	float halfT =0.001f ;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	 {
			
		
			
			// Normalise accelerometer measurement
//			norm = sqrt(ax * ax + ay * ay + az * az);
			norm = my_sqrt(ax * ax + ay * ay + az * az);
		 
			norm = 1/norm;
			ax *= norm;
			ay *= norm;
			az *= norm;		
	
			// Estimated direction of gravity and vector perpendicular to magnetic flux
			vx = (q1 * q3 - q0 * q2)*2;
			vy = (q0 * q1 + q2 * q3)*2;
			vz = q0 * q0 - q1*q1-q2*q2 + q3 * q3;
		
			// Error is sum of cross product between estimated and measured direction of gravity
			ex = (ay * vz - az * vy);
			ey = (az * vx - ax * vz);
			ez = (ax * vy - ay * vx);
	
			// Compute and apply integral feedback if enabled
//		    exInt = exInt + ex * Ki*halfT;
//			eyInt = eyInt + ey * Ki*halfT;
//			ezInt = ezInt + ez * Ki*halfT;
	
	        exInt = exInt + ex * Ki;
			eyInt = eyInt + ey * Ki;
			ezInt = ezInt + ez * Ki;
			
//	        exInt = LIMIT(exInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
//			eyInt = LIMIT(eyInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
//			ezInt = LIMIT(ezInt,-IMU_INTEGRAL_LIM,IMU_INTEGRAL_LIM);
	
			// Apply proportional feedback
			gx = gx + Kp*ex + exInt; 
			gy = gy + Kp*ey + eyInt; 
			gz = gz + Kp*ez + ezInt; 
		}
		
		// Integrate rate of change of quaternion
	
		q0 += (-q1 * gx - q2 * gy -q3 * gz)*halfT;
		q1 += (q0 * gx + q2 * gz - q3 * gy)*halfT;
		q2 += (q0 * gy - q1 * gz + q3 * gx)*halfT;
		q3 += (q0 * gz + q1 * gy - q2 * gx)*halfT; 
		
		// Normalise quaternion
//		norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		norm = my_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		
		norm = 1/norm;
		q0 *= norm;
		q1 *= norm;
		q2 *= norm;
		q3 *= norm;
				

//		angle[ROLL]=asin(-2*q1*q3 + 2*q0*q2) * 57.3;
		angle[PITCH] = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.3;  
//        angle[YAW] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1) * 57.3;  

		angle[PITCH]+=PITCH_BALANCE;
//		angle[ROLL]+=ROLL_BALANCE;
		
		angle[PITCH]*=1.2;
//		angle[ROLL]*=1.2;
		
//		printf("Target:%f   measure:%f      PwmOut:%f\r\n",measure.Angle[ROLL],measure.Angle[PITCH],measure.Angle[YAW]);
// Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* Rad; // yaw
//Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)* Rad; // pitch
//Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * Rad; //
//roll
//if(Q_ANGLE.X>90||Q_ANGLE.X<-90)
//{
//if(Q_ANGLE.Y>0)
//Q_ANGLE.Y=180-Q_ANGLE.Y;
//if(Q_ANGLE.Y<0)
//Q_ANGLE.Y=-(180+Q_ANGLE.Y);
  
	   
	
}

	
	
	
	
	