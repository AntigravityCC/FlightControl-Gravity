#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32f4xx.h"

//Òý½Å¶¨Òå
/*******************************************************/
//#define KEY1_INT_GPIO_PORT                GPIOA
//#define KEY1_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOA
//#define KEY1_INT_GPIO_PIN                 GPIO_Pin_0
//#define KEY1_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOA
//#define KEY1_INT_EXTI_PINSOURCE           EXTI_PinSource0
//#define KEY1_INT_EXTI_LINE                EXTI_Line0


//#define KEY2_INT_GPIO_PORT                GPIOA
//#define KEY2_INT_GPIO_CLK                 RCC_AHB1Periph_GPIOA
//#define KEY2_INT_GPIO_PIN                 GPIO_Pin_15
//#define KEY2_INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOA
//#define KEY2_INT_EXTI_PINSOURCE           EXTI_PinSource15
//#define KEY2_INT_EXTI_LINE                EXTI_Line15
//#define KEY2_INT_EXTI_IRQ                 EXTI15_10_IRQn

//#define KEY2_IRQHandler                   EXTI15_10_IRQHandler

/*******************************************************/


#define CAPTURE_EXIT_IRQ                    EXTI9_5_IRQn

#define CAPTURE_EXIT_IRQHandler             EXTI9_5_IRQHandler

void CAPTURE_EXTI_Config(void);

#endif /* __EXTI_H */
