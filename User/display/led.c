
#include "include.h" 
#include "led.h" 


void led_toggle(int16_t time)
{

	rt_pin_write(LED0_PIN, PIN_HIGH);
	rt_thread_mdelay(time);
	rt_pin_write(LED0_PIN, PIN_LOW);
	rt_thread_mdelay(time);	    
}


void led_instructions(void)
{
    if(flight.Check)		
	{
		rt_pin_write(LED0_PIN, PIN_LOW);
	
	}
	
    else if(flight.Lock)	
	{
		led_toggle(50);
	}
	
	else 
	{
		led_toggle(500);		
	}

}




/*********************************************END OF FILE**********************/
