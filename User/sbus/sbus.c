#include "sbus.h"
#include "include.h"
uint16_t CH_get[ch_ammount] = {0};
//uint16_t _channels[16]={0};	
uint8_t sbus_buffer[256];
//uint8_t sbus_flag;

void sbus_deal(void)	
{

uint16_t _channels[16]={0};	
if(sbus_buffer[0] == 0x0F&&(sbus_buffer[24] == 0x00))
{	
_channels[0]  = ((sbus_buffer[1]|sbus_buffer[2]<<8)   & 0x07FF);
_channels[1]  = ((sbus_buffer[2]>>3 |sbus_buffer[3]<<5)                 & 0x07FF);
_channels[2]  = ((sbus_buffer[3]>>6 |sbus_buffer[4]<<2 |sbus_buffer[5]<<10)  & 0x07FF);
_channels[3]  = ((sbus_buffer[5]>>1 |sbus_buffer[6]<<7)                 & 0x07FF);
_channels[4]  = ((sbus_buffer[6]>>4 |sbus_buffer[7]<<4)                 & 0x07FF);
_channels[5]  = ((sbus_buffer[7]>>7 |sbus_buffer[8]<<1 |sbus_buffer[9]<<9)   & 0x07FF);
_channels[6]  = ((sbus_buffer[9]>>2 |sbus_buffer[10]<<6)                & 0x07FF);
_channels[7]  = ((sbus_buffer[10]>>5|sbus_buffer[11]<<3)                & 0x07FF);
_channels[8]  = ((sbus_buffer[12]   |sbus_buffer[13]<<8)                & 0x07FF);
_channels[9]  = ((sbus_buffer[13]>>3|sbus_buffer[14]<<5)                & 0x07FF);
_channels[10] = ((sbus_buffer[14]>>6|sbus_buffer[15]<<2|sbus_buffer[16]<<10) & 0x07FF);
_channels[11] = ((sbus_buffer[16]>>1|sbus_buffer[17]<<7)                & 0x07FF);
_channels[12] = ((sbus_buffer[17]>>4|sbus_buffer[18]<<4)                & 0x07FF);
_channels[13] = ((sbus_buffer[18]>>7|sbus_buffer[19]<<1|sbus_buffer[20]<<9)  & 0x07FF);
_channels[14] = ((sbus_buffer[20]>>2|sbus_buffer[21]<<6)                & 0x07FF);
_channels[15] = ((sbus_buffer[21]>>5|sbus_buffer[22]<<3)                & 0x07FF);
//rt_kprintf("ccc:%d \r\n",_channels[2] );
}	
rc_deal(_channels,CH_get,ch_ammount);	



}


void rc_deal(uint16_t *rc_buffer, uint16_t *dealed_buffer,uint8_t amount)
{   
	int i; 

	for(i=0; i<amount;i++)
	{
	  *(dealed_buffer+i) = *(rc_buffer+i)+RC_ERR;
	  *(dealed_buffer+i) = LIMIT( *(dealed_buffer+i),1000,2000);  
    }

}


float rc_convert (float rc_get, float max_abs, float dead_limit)
{
    float proportion;
	
    if (rc_get > dead_limit)
	{
	 proportion = (rc_get - dead_limit)/ (max_abs - dead_limit);
	}	
    else if (rc_get < -dead_limit)
	{
	 proportion = (rc_get + dead_limit)/ (max_abs + dead_limit);
	}
	else proportion = 0;
	
	return proportion;
}


