#ifndef __INCLUDE_H
#define __INCLUDE_H
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "some_math.h"
#include "sbus.h"
//#include "usart.h"
//#include "tim.h"
//#include "gpio.h"
#include "drivers_from_stm32cubemx.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

#include "i2c_software.h"
#include "imu.h"
#include "imu2.h"
#include "angle_fusion.h"
#include "some_math.h"
#include "ctrl.h"
#include "pid_control.h"
#include "flight_mode.h"
#include "display.h" 
#include "pwm_out.h"
#include "parameter.h"
#include "led.h" 
#include "some_filter.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


#define UART1_NAME "uart1"
#define UART2_NAME "uart2"
#define UART3_NAME "uart3"

extern rt_sem_t test_sem;
extern rt_device_t serial1; 
extern rt_device_t serial2; 

void RTT_startup_all(void);
void uart3_init(void);
void pwm4_init(void );
// void InitAll(void);
void RTT_uart1_init(void);
void RTT_uart2_init(void);
void display_task(void);
 
void init_all(void);

#endif
