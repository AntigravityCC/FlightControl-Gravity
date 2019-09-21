#include "include.h"
#include "pid_control.h"


pid_t PID[GROUPS]={0};

//int  Pwm_Out_MAX=0;
//u8  remove_i=0;



//int Attitude__Control(int x)
//{
//  Stablize.Roll_ctr.Ideal=0;


//}

void position_PID(pid_t *PID,float target,float measure,uint8_t mode,float T)  //×ËÌ¬pidµ÷½Úº¯Êý
{
	switch(mode)
	{		
		case P: { PID->Ki=0;PID->Kd=0;break; }
        case PD:{ PID->Ki=0;break; }
		case PI:{ PID->Kd=0;break; }
		defaut:;
	}

    PID->ErrNow = target - measure;  
	PID->Integral += PID->ErrLast;
	
	if(PID->ErrNow > REMOVE_I || PID->ErrNow< -(REMOVE_I) )  
    {
		PID->RemoveIFlag = 0; 
		PID->Integral = 0;
    }  
    else 
    {
		PID->RemoveIFlag = 1; 
    }    

	PID->PwmOut =  PID->Kp * PID->ErrNow;	
	PID->PwmOut += PID->Ki * PID->Integral*PID->RemoveIFlag; 	
	PID->PwmOut += PID->Kd * (PID->ErrNow-PID->ErrLast);
	
	PID->ErrLast =PID->ErrNow;
  	    
	
}
    
