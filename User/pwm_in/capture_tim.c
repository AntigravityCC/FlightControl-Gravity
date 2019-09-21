/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   通用定时器定时范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "capture_tim.h"

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
/*******************************************************************************************
 * 文件名  ：pwm_in.c
 * 描述    ：PWM输入捕获
 * 官网    ：SANYE.TECH
 * 淘宝    ：SANYE.TECH
 * 技术Q群 ：515957331
**********************************************************************************/


#include "include.h"

void PWM_Input_Init(void)

{
	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	 
	
//////////////////////////////////////////////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
  
	TIM3->PSC = (168/2)-1;
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////
}
  /* TIM enable counter */


//void PWM_Input_Init(void)
//{
////	
//	GPIO_InitTypeDef GPIO_InitStructure;
//  TIM_ICInitTypeDef  TIM_ICInitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;

//  RCC_APB1PeriphClockCmd(CAPTURE_TIM_CLK, ENABLE);
//	
////  RCC_AHB1PeriphClockCmd(CAPTURE_ICPWM_GPIO_CLK, ENABLE);
//	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
//	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_TIM_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	  GPIO_InitStructure.GPIO_Pin =  CAPTURE_ICPWM_PIN|CAPTURE_ICPWM_PIN2|CAPTURE_ICPWM_PIN3|CAPTURE_ICPWM_PIN4;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//  GPIO_Init(CAPTURE_ICPWM_GPIO_PORT, &GPIO_InitStructure);
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

//  GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT, GPIO_PinSource6, CAPTURE_ICPWM_AF);
//	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT, GPIO_PinSource7, CAPTURE_ICPWM_AF);
////	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT, GPIO_PinSource8, CAPTURE_ICPWM_AF);
////	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT, GPIO_PinSource9, CAPTURE_ICPWM_AF);
//			GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
//  
//		
//		CAPTURE_TIM->PSC = (168/2)-1;
//		
//		
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);


//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  TIM_ICInit(CAPTURE_TIM, &TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM_ICInitStructure.TIM_ICFilter = 0x0;
//  


//	
//	
//	 TIM_Cmd(CAPTURE_TIM, ENABLE);


////  /* Enable the CC2 Interrupt Request */
//  TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC1, ENABLE);
//	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC2, ENABLE);
//	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC3, ENABLE);
//	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC4, ENABLE);
//	
///////////////////////////////////////////////////////////////////////////////////////////////
////   RCC_APB1PeriphClockCmd(CAPTURE_TIM_CLK1, ENABLE);
////	
////  RCC_AHB1PeriphClockCmd(CAPTURE_ICPWM_GPIO_CLK1, ENABLE);
////	
////	NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_TIM_IRQn1;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
////	
////	
////	
//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
////  GPIO_InitStructure.GPIO_Pin =  CAPTURE_ICPWM_PIN11|CAPTURE_ICPWM_PIN21|CAPTURE_ICPWM_PIN31|CAPTURE_ICPWM_PIN41;
////  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
////  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
////  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
////  GPIO_Init(CAPTURE_ICPWM_GPIO_PORT1, &GPIO_InitStructure);


////  GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT1, GPIO_PinSource0, CAPTURE_ICPWM_AF1);
////	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT1, GPIO_PinSource1, CAPTURE_ICPWM_AF1);
////	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT1, GPIO_PinSource2, CAPTURE_ICPWM_AF1);
////	GPIO_PinAFConfig(CAPTURE_ICPWM_GPIO_PORT1, GPIO_PinSource3, CAPTURE_ICPWM_AF1);



////  CAPTURE_TIM->PSC = (168/2)-1;
//////  CAPTURE_TIM1->PSC = (168)-1;
//		
//	
////  
////	
////		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
////  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
////  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
////  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
////  TIM_ICInitStructure.TIM_ICFilter = 0x0;
////  TIM_ICInit(CAPTURE_TIM1, &TIM_ICInitStructure);
////	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
////  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
////  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
////  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
////  TIM_ICInitStructure.TIM_ICFilter = 0x0;
////  TIM_ICInit(CAPTURE_TIM1, &TIM_ICInitStructure);
////	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
////  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
////  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
////  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
////  TIM_ICInitStructure.TIM_ICFilter = 0x0;
////  TIM_ICInit(CAPTURE_TIM1, &TIM_ICInitStructure);
////	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
////  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
////  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
////  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
////  TIM_ICInitStructure.TIM_ICFilter = 0x0;
////  TIM_ICInit(CAPTURE_TIM1, &TIM_ICInitStructure);
////	
////	
////	
////  /* TIM enable counter */
////  TIM_Cmd(CAPTURE_TIM, ENABLE);
////  TIM_Cmd(CAPTURE_TIM1, ENABLE);
////	
////  /* Enable the CC2 Interrupt Request */
////  TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC1, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC2, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC3, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM, TIM_IT_CC4, ENABLE);
////	
////	
////	  /* Enable the CC2 Interrupt Request */
////  TIM_ITConfig(CAPTURE_TIM1, TIM_IT_CC1, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM1, TIM_IT_CC2, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM1, TIM_IT_CC3, ENABLE);
////	TIM_ITConfig(CAPTURE_TIM1, TIM_IT_CC4, ENABLE);
/////////////////////////////////////////////////////////////////////////////////////////////////

//}





/*********************************************END OF FILE**********************/
