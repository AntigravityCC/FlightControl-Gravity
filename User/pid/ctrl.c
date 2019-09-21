#include "include.h"


void angle_target_get(void)
{	
	sbus_deal();
	PID[roll_angle].Target = ANGLE_MAX * rc_convert (  CH_get[roll_ctrl]-RC_MID,  MAX_ABS, DEAD_LIMIT);
	PID[pitch_angle].Target = ANGLE_MAX * rc_convert (CH_get[pitch_ctrl]-RC_MID,  MAX_ABS, DEAD_LIMIT);	
		
}

void gyro_target_get(void)
{
	
	PID[roll_rate].Target = GYRO_MAX*rc_convert(PID[roll_angle].PwmOut,ANGLE_MAX,DEAD_LIMIT);
	PID[pitch_rate].Target = GYRO_MAX*rc_convert(PID[pitch_angle].PwmOut,ANGLE_MAX,DEAD_LIMIT);
	PID[yaw_rate].Target = YAW_GYRO_MAX * rc_convert (CH_get[yaw_ctrl]-RC_MID,  MAX_ABS, DEAD_LIMIT);		
    
}

int throttle_target_get(void)
{
	float k = 0.7;
    int throttle=0;
	static int throttle_last=0;
	
    throttle = CH_get[throttle_ctrl] * k + (1-k)*throttle_last;
	
	throttle_last = throttle;
	
	throttle -= 1000;
	
	throttle = LIMIT(throttle,0,1000);
	
	return throttle;	
}


void angle_ctrl(void)
{		
	
	position_PID(&PID[roll_angle],PID[roll_angle].Target,measure.Angle[ROLL],P,0) ;
	position_PID(&PID[pitch_angle],PID[pitch_angle].Target,measure.Angle[PITCH],P,0) ;	
	
	PID[roll_angle].PwmOut=LIMIT(PID[roll_angle].PwmOut,-ANGLE_MAX,ANGLE_MAX);
	PID[pitch_angle].PwmOut =LIMIT(PID[	pitch_angle].PwmOut,-ANGLE_MAX,ANGLE_MAX);

}

void rate_ctrl(void)
{		
		
	position_PID(&PID[roll_rate], PID[roll_rate].Target, measure.Gyro[ROLL],PID_ALL,0) ;
	position_PID(&PID[pitch_rate],PID[pitch_rate].Target, measure.Gyro[PITCH],PID_ALL,0) ;	
	position_PID(&PID[yaw_rate], PID[yaw_rate].Target, measure.Gyro[YAW] , PID_ALL,0) ;		
	
	PID[roll_rate].PwmOut = LIMIT(PID[roll_rate].PwmOut,-PID_OUT_MAX,PID_OUT_MAX);
	PID[pitch_rate].PwmOut = LIMIT(PID[pitch_rate].PwmOut,-PID_OUT_MAX,PID_OUT_MAX);
	PID[yaw_rate].PwmOut = LIMIT(PID[yaw_rate].PwmOut,-YAW_PWM_MAX,YAW_PWM_MAX);
	 
//	printf("Target:%f   measure:%f      PwmOut:%f   \r\n",PID[roll_rate].Target,measure.Angle[ROLL],PID[roll_rate].PwmOut);
//	printf("Target:%f   measure:%f      PwmOut:%f   \r\n",PID[pitch_rate].Target,measure.Angle[PITCH],PID[pitch_rate].PwmOut);
	
//	printf("Target:%f   measure:%f      PwmOut:%f   \r\n",PID[roll_rate].Target,PID[pitch_rate].Target,PID[yaw_rate].Target);
//	printf("Target:%f   measure:%f      PwmOut:%f   \r\n",PID[roll_angle].Target,PID[pitch_angle].Target,PID[yaw_angle].Target);
}


int Motor[motor_amount]={0};  
  
void motor_out(void)      //电机输出函数
{   

 
#ifdef ANG_DEBUG 
    Motor[m1]= + PID[roll_angle].PwmOut + PID[pitch_angle].PwmOut + 0*PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m2]= - PID[roll_angle].PwmOut - PID[pitch_angle].PwmOut + 0*PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m3]= - PID[roll_angle].PwmOut + PID[pitch_angle].PwmOut - 0*PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m4]= + PID[roll_angle].PwmOut - PID[pitch_angle].PwmOut - 0*PID[yaw_rate].PwmOut + throttle_target_get();  

#elif ESC_DEBUG		
	Motor[m1]= CH_get[throttle_ctrl]-1000;
    Motor[m2]= CH_get[throttle_ctrl]-1000;
    Motor[m3]= CH_get[throttle_ctrl]-1000;
    Motor[m4]= CH_get[throttle_ctrl]-1000;  
	
#else	
	
//	Motor[m1]= + PID[roll_rate].PwmOut*0 + PID[pitch_rate].PwmOut + PID[yaw_rate].PwmOut*0 + throttle_target_get();
//    Motor[m2]= - PID[roll_rate].PwmOut*0 - PID[pitch_rate].PwmOut + PID[yaw_rate].PwmOut*0 + throttle_target_get();
//    Motor[m3]= - PID[roll_rate].PwmOut*0 + PID[pitch_rate].PwmOut - PID[yaw_rate].PwmOut*0 + throttle_target_get();
//    Motor[m4]= + PID[roll_rate].PwmOut*0 - PID[pitch_rate].PwmOut - PID[yaw_rate].PwmOut*0 + throttle_target_get();  

	Motor[m1]= + PID[roll_rate].PwmOut + PID[pitch_rate].PwmOut + PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m2]= - PID[roll_rate].PwmOut - PID[pitch_rate].PwmOut + PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m3]= - PID[roll_rate].PwmOut + PID[pitch_rate].PwmOut - PID[yaw_rate].PwmOut + throttle_target_get();
    Motor[m4]= + PID[roll_rate].PwmOut - PID[pitch_rate].PwmOut - PID[yaw_rate].PwmOut + throttle_target_get();  
	
//	rt_kprintf("target：%d  target2：%d\r\n"  ,(int)PID[roll_rate].PwmOut  ,(int)PID[pitch_rate].PwmOut); 

#endif	
	

    for(int i=0;i<motor_amount;i++)
    {
		Motor[i] = LIMIT( Motor[i],0,950);  
    }
     
     if(!flight.Ready)	
     ESC_Out(0,0,0,0);	
     else	 
     ESC_Out(Motor[m1],Motor[m2],Motor[m3],Motor[m4]);
//		printf("control is %d\r\n",Motor[m1]); 	
	
	
	 

}

