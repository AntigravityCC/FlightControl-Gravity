#include "flight_mode.h"

flight_mode_t flight;

void fly_ready_check(void)
{
	if(CH_get[mode5_ctrl] < RC_MID)
	flight.Lock	 = 0;
	else 
	flight.Lock	 = 1;	
	
	if((CH_get[throttle_ctrl] > RC_SAFE)  && (flight.Lock==0))
    flight.Ready = 1;
	else 
	flight.Ready = 0;	
}

void flight_mode_set(void)
{
	fly_ready_check();

}