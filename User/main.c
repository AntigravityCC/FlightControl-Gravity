 /**
  * @brief     main
  * @param     none
  * @retval    none
  * @attention The code is based on RT_Thread
  * @author    irving1412(chong)
  * @version   V1.0
  * @date      2018-11-xx
  **/

#include "include.h"
#include "RTT_task.h"

// #define   ESC_calibrate 

/**
  * @brief  This function handles init.
  * @param  None
  * @retval None
  */

void RTT_startup_thread(rt_thread_t thread, char *task);

int main(void)
{ 
	
	InitAll();
		
	printf("beginning \r\n");

	start_thread	=  rt_thread_create("start",
                                     start_thread_entry,
									 RT_NULL,
									 START_STK_SIZE,
									 START_THREAD_PRIO,
									 START_THREAD_TIME);	
      RTT_startup_thread(start_thread,"start_thread_entry");	
//	RTT_startup_all();
// rt_thread_suspend(receive_thread);
  
	return  0 ;
} 


/**
  * @brief  This function handles creat thread.
  * @param  None
  * @retval None
  */
void start_thread_entry(void* parameter)
{
	 	 	
	test_sem =        rt_sem_create("test_sem",         	 			/*  信号量名字 */
							 1,     									/*  信号量初始值，默认有一个信号量 */
							 RT_IPC_FLAG_FIFO);							/*  信号量模式 FIFO(0x00)*/
		 
	

	receive_thread =   rt_thread_create( "receive", 	                /* 线程名字 */
										 receive_thread_entry, 			/* 线程入口函数 */
										 RT_NULL, 						/* 线程入口函数参数 */
										 RECEIVE_STK_SIZE, 				/* 线程栈大小 */
										 RECEIVE_THREAD_PRIO, 			/* 线程的优先级 */
										 RECEIVE_THREAD_TIME);		    /* 线程时间片 */
	RTT_startup_thread(receive_thread,"receive_thread_entry");
	
	
	receive_thread =   rt_thread_create( "sensors", 	                /* 线程名字 */
										 sensors_thread_entry, 			/* 线程入口函数 */
										 RT_NULL, 						/* 线程入口函数参数 */
										 SENSORS_STK_SIZE, 				/* 线程栈大小 */
										 SENSORS_THREAD_PRIO, 			/* 线程的优先级 */
										 SENSORS_THREAD_TIME);		    /* 线程时间片 */

	 RTT_startup_thread(receive_thread,"sensors_thread_entry");	 
										 
//	control_thread =   rt_thread_create( "control", 					/* 线程名字 */
//										 control_thread_entry, 			/* 线程入口函数 */
//										 RT_NULL, 						/* 线程入口函数参数 */
//										 CONTROL_STK_SIZE, 				/* 线程栈大小 */
//										 CONTROL_THREAD_PRIO, 			/* 线程的优先级 */
//										 CONTROL_THREAD_TIME); 			/* 线程时间片 */
//	RTT_startup_thread(control_thread,"control_thread_entry");

	angle_control_thread =   rt_thread_create( "control_angle", 					/* 线程名字 */
										 angle_control_thread_entry, 			/* 线程入口函数 */
										 RT_NULL, 						/* 线程入口函数参数 */
										 ANGLE_CONTROL_STK_SIZE, 				/* 线程栈大小 */
										 ANGLE_CONTROL_THREAD_PRIO, 			/* 线程的优先级 */
										 ANGLE_CONTROL_THREAD_TIME); 			/* 线程时间片 */
   
RTT_startup_thread(angle_control_thread,"angle_control_thread_entry");
//	 
	send_thread   =    rt_thread_create( "send", 				    	/* 线程名字 */
										 send_thread_entry, 			/* 线程入口函数 */
										 RT_NULL, 						/* 线程入口函数参数 */
										 SEND_STK_SIZE, 				/* 线程栈大小 */
										 SEND_THREAD_PRIO, 				/* 线程的优先级 */
										 SEND_THREAD_TIME); 			/* 线程时间片 */
RTT_startup_thread(send_thread,"send_thread_entry");

//	flight_mode_thread = rt_thread_create( "flight_mode",				/* 线程名字 */
//										 mode_thread_entry, 			/* 线程入口函数 */
//										 RT_NULL, 						/* 线程入口函数参数 */
//										 MODE_STK_SIZE, 				/* 线程栈大小 */
//										 MODE_THREAD_PRIO, 				/* 线程的优先级 */
//										 MODE_THREAD_TIME);				/* 线程时间片 */	
//	RTT_startup_thread(flight_mode_thread,"flight_mode_thread_entry");			


	led_thread   =    rt_thread_create("led",                           /* 线程入口函数 */
								         led_thread_entry,        		/* 线程入口函数参数 */
										 RT_NULL,                 		/* 线程栈大小 */
										 LED_STK_SIZE,            		/* 线程的优先级 */
										 LED_THREAD_PRIO,        		/* 线程时间片 */
										 LED_THREAD_TIME);	 		

RTT_startup_thread(led_thread,"led_thread_entry"); 
      printf("ssssssss \r\n");	
	  
	
	rt_thread_delete(start_thread);								 
//  rt_thread_suspend(start_thread);
}
 		
void RTT_startup_thread(rt_thread_t thread, char *task)
{ 
	
	if (thread != RT_NULL)
	{
		rt_thread_startup(thread);
	    printf("%s created success \r\n",task);	   
	}
	else	 
	{
		printf("%s created failed \r\n",task);	   
		while(1);
	}
}
 
  /* 启动线程，开启调度 */
 void RTT_startup_all(void)
 {
//	RTT_startup_thread(start_thread,"start_thread_entry");
//	RTT_startup_thread(receive_thread,"receive_thread_entry");
//    RTT_startup_thread(receive_thread,"sensors_thread_entry");	 
	 
//	RTT_startup_thread(send_thread,"send_thread_entry");
//	RTT_startup_thread(led_thread,"led_thread_entry"); 
//	RTT_startup_thread(control_thread,"control_thread_entry");
//	RTT_startup_thread(angle_control_thread,"angle_control_thread_entry");
//   	RTT_startup_thread(flight_mode_thread,"flight_mode_thread_entry");
	
	 
	if (test_sem != RT_NULL)
	printf("信号量创建成功！\n\n");
	else  printf("信号量创建失败！\n\n");
 }
 
/*********************************************END OF FILE**********************/

