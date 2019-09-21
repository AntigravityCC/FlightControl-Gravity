#ifndef __SBUS_H
#define	__SBUS_H

#include "include.h"

#define RC_ERR 500	
#define MAX_ABS 500	
#define RC_MID 1500	
#define DEAD_LIMIT 5	



enum
{
roll_ctrl=0,
pitch_ctrl,	
throttle_ctrl,
yaw_ctrl,
mode1_ctrl,
mode2_ctrl,	
mode3_ctrl,	
mode4_ctrl,
mode5_ctrl,	
ch_ammount	
};
	

extern uint16_t CH_get[ch_ammount] ;	
	
//extern u16 sbus_rc_in[8];
extern uint8_t sbus_buffer[256];
extern uint16_t Rc_Pwm_In[8];
extern uint16_t _channels[16];
extern uint16_t  channel3;
void sbus_deal(void);
void rc_deal(uint16_t *rc_buffer, uint16_t *dealed_buffer,uint8_t amount);
float rc_convert (float rc_get, float max_abs, float dead_limit);


#endif

