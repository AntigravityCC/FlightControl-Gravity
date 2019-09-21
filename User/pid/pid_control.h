#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "include.h"


typedef struct{
    float   Kp;
    float   Kd;
    float   Ki;
    float   ErrNow;
    float   ErrLast;
    
    float   Integral;
    float   Target;
    float   PwmOut;
    uint8_t  RemoveIFlag ;

//    float  err;
} pid_t;


//extern pid_t   Roll_ctr;
//extern pid_t   Pitch_ctr;
//extern  pid_t   Yaw_ctr;



enum
{
roll_angle=0,
pitch_angle,
yaw_angle,
roll_rate,
pitch_rate,
yaw_rate,
GROUPS
};


enum
{	
P=0,
PD,
PI,
PID_ALL
};

extern pid_t PID[GROUPS];

//extern ctr_t  Rate;
//extern ctr_t  Stablize;
void position_PID(pid_t *PID,float target,float measure,uint8_t mode,float T) ;


#define  REMOVE_I 5


#endif



