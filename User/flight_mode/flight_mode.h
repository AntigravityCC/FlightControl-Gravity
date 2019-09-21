#ifndef __FLIGHT_MODE_H
#define __FLIGHT_MODE_H

#include "include.h"

#define  RC_SAFE  1020


typedef struct
{
uint8_t Ready;
uint8_t Lock;	
uint8_t Altitude_hold;
uint8_t Position_hold;	
uint8_t Take_off;
uint8_t Land;	
uint8_t Search;	
uint8_t Check;
}flight_mode_t;

extern flight_mode_t flight;

void flight_mode_set(void);


	
# endif 
