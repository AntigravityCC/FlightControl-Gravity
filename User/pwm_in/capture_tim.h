#ifndef __CAPTURE_TIM_H
#define	__CAPTURE_TIM_H

#include "stm32f4xx.h"
#include "include.h"




#define CAPTURE_ICPWM_PIN             GPIO_Pin_6     
#define CAPTURE_ICPWM_PIN2            GPIO_Pin_7  
//#define CAPTURE_ICPWM_PIN3            GPIO_Pin_8  
//#define CAPTURE_ICPWM_PIN4            GPIO_Pin_9
#define CAPTURE_ICPWM_PIN3            GPIO_Pin_0  
#define CAPTURE_ICPWM_PIN4            GPIO_Pin_1


#define CAPTURE_ICPWM_PIN11            GPIO_Pin_0    
#define CAPTURE_ICPWM_PIN21            GPIO_Pin_1 
#define CAPTURE_ICPWM_PIN31            GPIO_Pin_2  
#define CAPTURE_ICPWM_PIN41            GPIO_Pin_3


#define CAPTURE_ICPWM_GPIO_PORT       GPIOC 
#define CAPTURE_ICPWM_GPIO_PORT2      GPIOB 
#define CAPTURE_ICPWM_GPIO_PORT1      GPIOA 


#define CAPTURE_ICPWM_GPIO_CLK        RCC_AHB1Periph_GPIOC
#define CAPTURE_ICPWM_GPIO_CLK1        RCC_AHB1Periph_GPIOA

#define CAPTURE_ICPWM_PINSOURCE			 	GPIO_PinSource6
#define CAPTURE_ICPWM_PINSOURCE2			GPIO_PinSource7
#define CAPTURE_ICPWM_PINSOURCE3			GPIO_PinSource8
#define CAPTURE_ICPWM_PINSOURCE4			GPIO_PinSource9


#define CAPTURE_ICPWM_PINSOURCE11			GPIO_PinSource0
#define CAPTURE_ICPWM_PINSOURCE21			GPIO_PinSource1
#define CAPTURE_ICPWM_PINSOURCE31			GPIO_PinSource2
#define CAPTURE_ICPWM_PINSOURCE41			GPIO_PinSource3


//#define ADVANCE_ICPWM_AF							GPIO_AF_TIM8
#define CAPTURE_ICPWM_AF							 GPIO_AF_TIM3
#define CAPTURE_ICPWM_AF1							 GPIO_AF_TIM2

#define CAPTURE_IC1PWM_CHANNEL        TIM_Channel_1
#define CAPTURE_IC2PWM_CHANNEL        TIM_Channel_2
#define CAPTURE_IC3PWM_CHANNEL        TIM_Channel_3
#define CAPTURE_IC4PWM_CHANNEL        TIM_Channel_4




#define CAPTURE_TIM           		    TIM3
#define CAPTURE_TIM1           		    TIM2

#define CAPTURE_TIM_CLK       		    RCC_APB1Periph_TIM3
#define CAPTURE_TIM_CLK1       		    RCC_APB1Periph_TIM2

#define CAPTURE_TIM_IRQn					    TIM3_IRQn
#define CAPTURE_TIM_IRQn1					    TIM2_IRQn

#define CAPTURE_TIM_IRQHandler        TIM3_IRQHandler
#define CAPTURE_TIM_IRQHandler1       TIM2_IRQHandler



void PWM_Input_Init(void);

extern u16 Rc_Pwm_In[8];



#endif