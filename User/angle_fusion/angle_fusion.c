#include "some_filter.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "angle_fusion.h"
#include "include.h"

measure_t measure;


float angle_get( float gyro_y,float gyro_z,float accel_x,float accel_z)
{ 
	float Acc_angle_Y,angle_err,angle_out;
	static  float gyro_integration;
    
	if(gyro_z>32768)  gyro_z -=65536;                       //数据类型转换
	if(accel_x>32768) accel_x-=65536;                      //数据类型转换
	if(accel_z>32768) accel_z-=65536;                      //数据类型转换
					
	Acc_angle_Y=atan2(accel_x,accel_z)*180/PI_MATH;  

//		rt_kprintf("angle %d\r\n",(int)Acc_angle_Y);

//	    gyro_y=gyro_y/16.4;   //陀螺仪量程转换     
	
	gyro_integration+=gyro_y*GYRO_K;

	 if(Acc_angle_Y>=gyro_integration+10) Acc_angle_Y=gyro_integration+10 ;
	 if(Acc_angle_Y<=gyro_integration-10) Acc_angle_Y=gyro_integration-10 ;
//		rt_kprintf("Gyro_Y %d\r\n",(int)gyro_integration);
	angle_err = Acc_angle_Y - gyro_integration;

	gyro_integration+=angle_err*ANGLE_K;

	
	angle_out=gyro_integration;
//		rt_kprintf("angle %d  Gyro_Y:%d\r\n",(int)Acc_angle_Y,(int)Gyro_Y);
	
//        rt_kprintf("angle %d\r\n",(int)(angle_out*10));
		
	return   angle_out;
	
}


void angle_measure_get(void)
{
//    measure.Gyro[ROLL]   = (Gyro[ROLL]+ROLL_CORRECT)/GYRO_TRANSlATE_K;
//	measure.Gyro[PITCH]  = (Gyro[PITCH]+PITCH_CORRECT) /GYRO_TRANSlATE_K;	
//	measure.Gyro[YAW]    = (Gyro[YAW]+YAW_CORRECT)/GYRO_TRANSlATE_K;	
	
//    measure.Gyro[ROLL]   = (Gyro[ROLL]+ROLL_CORRECT);
//	measure.Gyro[PITCH]  = (Gyro[PITCH]+PITCH_CORRECT) ;	
//	measure.Gyro[YAW]    = (Gyro[YAW]+YAW_CORRECT);
//	
//	measure.Gyro[YAW] = DEAD(measure.Gyro[YAW] ,0,YAW_DEAD);
//	measure.Gyro[YAW] = DEAD(measure.Gyro[YAW] ,-YAW_DEAD,0);
	
	measure.Angle[ROLL]  = angle_get( measure.Gyro[ROLL],measure.Gyro[PITCH],Accel[PITCH],Accel[YAW]);
	measure.Angle[PITCH] = angle_get( measure.Gyro[PITCH],measure.Gyro[ROLL],Accel[ROLL],Accel[YAW]);	
}

