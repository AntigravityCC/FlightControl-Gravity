#ifndef __CTRL_H
#define	__CTRL_H

#include "include.h"

#define ANGLE_MAX 30


#define GYRO_MAX 300
#define YAW_GYRO_MAX 250
#define PID_OUT_MAX 900
#define YAW_PWM_MAX 600

#define PWM_MAX 950

#define RUN  

#ifdef RUN

//#define ANG_DEBUG  
//#define ESC_DEBUG   
//#define RATE_DEBUG  

#else

#define ALL_DEBUG 


#endif

enum
{
m1 = 0,
m2,
m3,
m4,
motor_amount	
};

extern int Motor[motor_amount]; 

void angle_target_get(void);
void gyro_target_get(void);
void angle_ctrl(void);
void rate_ctrl(void);

int throttle_target_get(void);
void motor_out(void)   ; //电机输出函数

#endif

