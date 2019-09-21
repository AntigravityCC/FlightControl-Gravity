
#ifndef __SOME_FILTER_H
#define __SOME_FILTER_H
#include "include.h"

char filter_limit(void);
char filter_mid(void);
char filter_average(void);
char filter_slip(void);
char filter_mid_average(void);
char filter_later(void);
char filter_push(void);
char filter_shake_eliminate(void);
float filter_slip_use(float get_ad,uint8_t n);
extern float filter_buf[255];
	
# endif 
