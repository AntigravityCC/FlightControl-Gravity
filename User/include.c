#include "include.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rtconfig.h>


void init_all(void)
{
	rt_kprintf("init start\r\n");
	flight.Check=-1;
	HAL_Init();
	MX_GPIO_Init();

	MPU6050_Init();
	
//	while((flight.Check = mpu_dmp_init()))
//	{  		
//		rt_kprintf("MPU6050 Error:%d\r\n",flight.Check);
//	}	

	uart3_init();
	RTT_uart1_init();
	RTT_uart2_init();
	pwm4_init();
	ESC_Init();
    parameter_init(1.0f);

    flight.Check=0;
	rt_kprintf("init success\r\n"); 	
}

int fputc(int ch, FILE *f)
{
	rt_device_write(serial1, 0, &ch, sizeof(ch));
	return ch;
}



 rt_device_t serial1; 
 rt_device_t serial2; 
 rt_sem_t test_sem = RT_NULL;

 uint8_t rx_temp;
 
void RTT_uart1_init(void)
{
	serial1 = rt_device_find(UART1_NAME);
}

struct serial_configure config2 = RT_SERIAL_CONFIG_DEFAULT;

void RTT_uart2_init(void)
{
	serial2 = rt_device_find(UART2_NAME);
	rt_device_open(serial2, RT_DEVICE_FLAG_INT_RX);
	rt_device_control(serial2, RT_DEVICE_CTRL_CONFIG, &config2);
}


void pwm4_init(void )
{
	
	MX_TIM4_Init();
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
}

void uart3_init(void)
{	
	HAL_UART_MspDeInit(&huart3);
	MX_USART3_UART_Init();
	HAL_UART_Receive_IT(&huart3, &rx_temp, 1); 
}



void USART3_IRQHandler(void)
{
	 /* enter interrupt */
	rt_interrupt_enter();
// rt_kprintf("initer\r\n");
	static uint8_t i=0;		
 
	HAL_UART_IRQHandler(&huart3);

	
	sbus_buffer[i++]=rx_temp;	
	if (i==25)
	{	
        rt_sem_release(test_sem);		
		i=0;		
	}
	
	HAL_UART_Receive_IT(&huart3, &rx_temp, 	1); 
		
	    /* leave interrupt */
	rt_interrupt_leave();      
}



 void receive_thread_entry(void* parameter)
 {
                             /*  线程 都是一个无限循环，不能返回 */
	while (1) 
	{
		rt_sem_take(test_sem,           /*  获取信号量 */
		RT_WAITING_FOREVER); /*  等待时间：一直等 */
       	
		sbus_deal();
  
		angle_target_get();
//		rt_kprintf("target：%d   target2：%d\r\n",(int)PID[roll_angle].Target,(int)PID[pitch_angle].Target); 
//		rt_kprintf("target：%d\r\n",(int)CH_get[throttle_ctrl]); 
			
		rt_thread_mdelay (10);  //  
	}
 
 }
 


 void led_thread_entry(void* parameter)
 {
    while(1)
    {	
		
		led_instructions();
	  
		rt_thread_mdelay(10);
	}
 }
 
 
  void send_thread_entry(void* parameter)
 {

	while (1)
		{	
			//        rt_kprintf("display is running\n"); 
			display_task();

			rt_thread_mdelay ( 10 ); 
		 }
 }


 void control_thread_entry(void* parameter)
 {
    while(1)
    {	
          
//        rt_kprintf("control is running\n"); 		 

#ifndef ANG_DEBUG 	       
		   rate_ctrl();	
#endif		  
		   motor_out();
		    
	
		
		
		
//		    printf("control is %d\r\n",throttle_target()); 	
//		  printf("control is %d\r\n",CH_get[throttle_ctrl]); 	
//		  printf("control is %d\r\n",(int)PID[yaw_rate].Target ); 
		
		   rt_thread_mdelay(2);
		
	}
 }
  
 void angle_control_thread_entry(void* parameter)
 {
    while(1)
    {		
//        rt_kprintf("angle is running\n"); 		 

		
		angle_ctrl();
		gyro_target_get();
//		motor_out();
	
		rt_thread_mdelay(5);		
	}
 }

 void mode_thread_entry(void* parameter)
 {
    while(1)
    {	
		uint8_t parameter_buffer[9];
		
//        rt_kprintf("mode is running\n"); 			
        flight_mode_set();
	 	rt_device_read(serial2, 0, parameter_buffer, sizeof(parameter_buffer));
	    parameter_set (parameter_buffer,sizeof(parameter_buffer));
//		rt_kprintf("%c\n",parameter_buffer[0]);
		
//			rt_kprintf("mode is running\n"); 	
//		rt_device_write(serial2, 0, parameter_buffer, sizeof (parameter_buffer));
		memset(parameter_buffer,'0',sizeof(parameter_buffer));
		rt_thread_mdelay(10);
		
	}
 }

 void sensors_thread_entry(void* parameter)
 {
	while(1)
    {		
		
//		rt_kprintf("sensors is running\n"); 	
		

//		printf("accx:%f\r\n",measure.Gyro[0]);
//		printf("accx:%f\r\n",measure.Acc[0]);
        MPU6050_task(measure.Gyro,measure.Acc);	
//		measure.Acc[PITCH]=filter_slip_use(measure.Acc[PITCH],FILTER_COUNT);
//		measure.Acc[ROLL]=filter_slip_use(measure.Acc[ROLL],FILTER_COUNT);
		
		IMUupdate_new(measure.Gyro,measure.Acc,measure.Angle);

//		if(mpu_dmp_get_data(&measure.Angle[PITCH],&measure.Angle[ROLL],&measure.Angle[YAW])==0)
//		{
			measure.Angle[ROLL] += ROLL_BALANCE;
			measure.Angle[PITCH] += PITCH_BALANCE;
			
//			printf("Target:%f   measure:%f      PwmOut:%f\r\n",measure.Angle[ROLL],measure.Angle[PITCH],measure.Angle[YAW]);			
//		}
				
					
//		angle_measure_get();
//		printf("roll:%.2f       pitch:%.2f      yaw:%.2f  \r\n",measure.Acc[ROLL],measure.Acc[PITCH],measure.Acc[YAW]);
//		printf("roll:%.2f       pitch:%.2f      yaw:%.2f  \r\n",measure.Gyro[ROLL],measure.Gyro[PITCH],measure.Gyro[YAW]);
//		printf("Target:%f   measure:%f      PwmOut:%f\r\n",measure.Angle[ROLL],measure.Angle[PITCH],measure.Angle[YAW]);

			    
		rt_thread_mdelay(5);
		
	}
 }
  

////		上位机发送任务函数
void display_task(void)
{
   ANO_DT_Send_Senser(measure.Acc[ROLL],measure.Acc[PITCH],measure.Acc[YAW],measure.Gyro[ROLL],measure.Gyro[PITCH],measure.Gyro[YAW],0,0,0,0);
////   ANO_DT_Send_Status(measure.Angle[ROLL],measure.Angle[PITCH],measure.Angle[YAW],0, 0, 0);
   ANO_DT_Send_MotoPWM(Motor[m1],Motor[m2],Motor[m3],Motor[m4],PID[yaw_rate].Target,PID[yaw_rate].PwmOut,PID[roll_rate].Target,PID[pitch_rate].Target);

//   ANO_DT_Send_RCData(CH_get[roll_ctrl],CH_get[pitch_ctrl],CH_get[throttle_ctrl],CH_get[yaw_ctrl],CH_get[mode1_ctrl],CH_get[mode2_ctrl],PID[roll_angle].PwmOut,PID[pitch_angle].PwmOut,PID[roll_rate].PwmOut,PID[pitch_rate].PwmOut);
//   ANO_DT_Send_RCData(PID[roll_angle].PwmOut,PID[roll_angle].PwmOut,PID[roll_angle].PwmOut,CH_get[yaw_ctrl],CH_get[mode1_ctrl],CH_get[mode2_ctrl],0,0,0,0);
}
