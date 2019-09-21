#ifndef __ANGLE_FUSION_H
#define __ANGLE_FUSION_H
//#define lvbo_K1 0.02

#define GYRO_K    0.005
#define ANGLE_K   0.02
#define GYRO_TRANSlATE_K 16.4f

#define GYRO_ROLL_CORRECT   28.0f
#define GYRO_PITCH_CORRECT  3.0f
#define GYRO_YAW_CORRECT    19.0f

#define YAW_DEAD 10

typedef struct
{
float Angle[3];
float Gyro[3];	
float Acc[3];	
float Mag[3];
float Altitude;
	
}measure_t;

enum	
{	
	ROLL=0,
	PITCH,
	YAW
};


extern measure_t measure;

float angle_get( float gyro_y,float gyro_z,float accel_x,float accel_z);
void angle_measure_get(void);



# endif 
