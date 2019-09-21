#ifndef __PWM_OUT_H
#define	__PWM_OUT_H

#include "include.h"
void  ESC_Out(int a,int b,int c,int d);
void ESC_Init(void);   
void ESC_Set(void);
void ESC_pwm_enable(void);
void ESC_pwm_disable(void);
#endif

