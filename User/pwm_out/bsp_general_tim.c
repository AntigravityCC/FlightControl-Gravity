#include "bsp_general_tim.h"
#include "./systick/bsp_SysTick.h"

/**
 * @brief TIM4 PWM部分初始化(oc1 和 oc2)   
 * @param[in]：自动重装值
 * @param[in]：时钟预分频数
 * @attention: TIM4定时器默认时钟为84M  若50Hz 则参数应为(20000-1, 84-1)
*/
void TIM4_PWM_Init(u32 arr,u32 psc)
{		 					 

	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTF时钟	
    
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); //GPIOB6复用为定时器4
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); //GPIOB7复用为定时器4
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); //GPIOB6复用为定时器4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); //GPIOB7复用为定时器4
    
    
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOB6
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB6 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;           //GPIOB7
    GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOB7
    GPIO_Init(GPIOD,&GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;           //GPIOB7
    GPIO_Init(GPIOD,&GPIO_InitStructure);     
    
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器4
	
	//初始化TIM4 Channel PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;	//空闲电平
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1  
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1  
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
    
    
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
 	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM	
  ESC_pwm_disable();    
}  

/**
 * @brief TIM9 PWM部分初始化(oc1 和 oc2)   
 * @param[in]：自动重装值
 * @param[in]：时钟预分频数
 * @attention: TIM9定时器默认时钟为168M  若50Hz 则参数应为(20000-1, 168-1)
*/
 void TIM9_PWM_Init(u32 arr,u32 psc)
{		 					 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM9时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); //GPIOE5复用为定时器4
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); //GPIOE6复用为定时器4
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;           //GPIOE5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE5
	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOE6
    GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE6        
    
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//初始化定时器9
	
	//初始化TIM9 Channel PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;	//空闲电平
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 OC1  
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 OC2
    
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM9在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM9在CCR2上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM9, ENABLE);  //使能TIM 										  
}  






