#ifndef __GENERAL_TIM_H
#define	__GENERAL_TIM_H

#include "stm32f4xx.h"

//#define GENERAL_OCPWM_PIN1             GPIO_Pin_6     
//#define GENERAL_OCPWM_PIN2             GPIO_Pin_7   
//#define GENERAL_OCPWM_PIN3             GPIO_Pin_8  
//#define GENERAL_OCPWM_PIN4             GPIO_Pin_9

//#define GENERAL_OCPWM_GPIO_PORT         GPIOC                      
//#define GENERAL_OCPWM_GPIO_CLK          RCC_AHB1Periph_GPIOC

//#define GENERAL_OCPWM_PINSOURCE1		  GPIO_PinSource6
//#define GENERAL_OCPWM_PINSOURCE2		  GPIO_PinSource7
//#define GENERAL_OCPWM_PINSOURCE3		  GPIO_PinSource8
//#define GENERAL_OCPWM_PINSOURCE4		  GPIO_PinSource9


//#define GENERAL_OCPWM_AF			  GPIO_AF_TIM3

//#define GENERAL_TIM           		  TIM3
//#define GENERAL_TIM_CLK       		  RCC_APB1Periph_TIM3

//#define GENERAL_TIM_IRQn		      TIM3_IRQn
//#define GENERAL_TIM_IRQHandler        TIM3_IRQHandler


void TIMx_Configuration(void);
void TIM4_PWM_Init(u32 arr,u32 psc);
#endif /* __GENERAL_TIM_H */

