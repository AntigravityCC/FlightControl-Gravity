#ifndef __RTT_TASK_H
#define __RTT_TASK_H


#include "rtthread.h"
#include "board.h"
#include "rthw.h"
#include "rtconfig.h"

/* 定义线程控制块 */

// static struct rt_thread led1_thread;

rt_thread_t start_thread=RT_NULL;
#define START_STK_SIZE 		  1500  
#define START_THREAD_PRIO		1
#define START_THREAD_TIME		2	
void start_thread_entry(void* parameter);

rt_thread_t led_thread=RT_NULL;
#define LED_STK_SIZE 		1000  
#define LED_THREAD_PRIO		7
#define LED_THREAD_TIME		2	
  void led_thread_entry(void* parameter);
	
	
rt_thread_t receive_thread = RT_NULL;
#define RECEIVE_STK_SIZE 		1000  
#define RECEIVE_THREAD_PRIO		2
#define RECEIVE_THREAD_TIME		2	
void receive_thread_entry(void* parameter);


rt_thread_t send_thread = RT_NULL;	
#define SEND_STK_SIZE 		    1000  
#define SEND_THREAD_PRIO		8
#define SEND_THREAD_TIME		2		
void send_thread_entry(void* parameter);	
		

rt_thread_t control_thread = RT_NULL;	
#define CONTROL_STK_SIZE 		1000  
#define CONTROL_THREAD_PRIO		4
#define CONTROL_THREAD_TIME		2		
void control_thread_entry(void* parameter);	
	
	
rt_thread_t angle_control_thread = RT_NULL;	
#define ANGLE_CONTROL_STK_SIZE 		 1000  
#define ANGLE_CONTROL_THREAD_PRIO		5
#define ANGLE_CONTROL_THREAD_TIME		2		
void angle_control_thread_entry(void* parameter);	


rt_thread_t flight_mode_thread = RT_NULL;	
#define MODE_STK_SIZE 		2000  
#define MODE_THREAD_PRIO		3
#define MODE_THREAD_TIME		2		
void mode_thread_entry(void* parameter);


rt_thread_t sensors_thread = RT_NULL;	
#define SENSORS_STK_SIZE 		1500  
#define SENSORS_THREAD_PRIO		6
#define SENSORS_THREAD_TIME		2		
void sensors_thread_entry(void* parameter);


void RTT_startup_all(void);




#endif
