#include "include.h"
#include "parameter.h"




int character_to_digital(char * parameter_buffer,uint8_t len,uint8_t n)
{
  int result;
  char cut_buffer[len];

  strncpy(cut_buffer, parameter_buffer+n,len);
//  rt_kprintf("cut_buffer:%s\r\n",cut_buffer);
  result=atoi(cut_buffer);
  return result;

}

void parameter_set(uint8_t *parameter_buffer,uint8_t len)
{
	
	int a;
	
	if(*parameter_buffer == '{')
	{
		
		switch(*(parameter_buffer+1))
	    {		
			case '0': 
			{ 
				PID[roll_rate].Kp=(float)(character_to_digital((char*)parameter_buffer,3,3))/100;	 
				printf("PID[roll_rate].Kp:%.2f\r\n",PID[roll_rate].Kp);break; 
			}
			
			case '1': 
			{ PID[pitch_rate].Kp=((float)character_to_digital((char*)parameter_buffer,3,3))/100;	break; }
			
			case '2': 
			{ PID[yaw_rate].Kp=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '3': 
			{ PID[roll_rate].Kd=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }	
			
			case '4': 
			{ PID[pitch_rate].Kd=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '5': 
			{ PID[yaw_rate].Kd=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '6': 
			{ PID[roll_rate].Ki=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '7': 
			{ PID[pitch_rate].Ki=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '8': 
			{ PID[yaw_rate].Ki=(float)character_to_digital((char*)parameter_buffer,3,3)/100;	break; }
			
			case '+': 
			{ PID[roll_angle].Kp=(float)character_to_digital((char*)parameter_buffer,3,3)/10;	break; }
			
			case '-': 
			{ PID[pitch_angle].Kp=(float)character_to_digital((char*)parameter_buffer,3,3)/10;	
			printf("PID[pitch_angle].Kp:%.2f\r\n",PID[pitch_angle].Kp);break; }
			case '*': 
			{ PID[yaw_angle].Kp=(float)character_to_digital((char*)parameter_buffer,3,3)/10;	break; }
			
			defaut:;
	   }				
      
	}

}


void  parameter_init(float set)
{    
	
	PID[roll_angle].Kp=set;
	PID[pitch_angle].Kp=set;
	PID[yaw_angle].Kp=set;
		
	PID[roll_rate].Kp=0.22f;
	PID[pitch_rate].Kp=0.22f;
//	PID[yaw_rate].Kp=0.16f;
	PID[yaw_rate].Kp=0.22f;	
	
	PID[roll_rate].Kd=0.67;
	PID[pitch_rate].Kd=0.67;
	PID[yaw_rate].Kd=0.06;
    
	PID[roll_rate].Ki=0.001;
	PID[pitch_rate].Ki=0.001;
	PID[yaw_rate].Ki=0;
//    remove_i= 0;
}

void parameter_save(void)
{




}
