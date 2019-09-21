
#include "include.h" 


 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
  
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置NVIC为优先级组1 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置中断源：按键1 */
  NVIC_InitStructure.NVIC_IRQChannel = CAPTURE_EXIT_IRQ;
  /* 配置抢占优先级：1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 配置子优先级：1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* 配置中断源：按键2，其他使用上面相关配置 */  
//  NVIC_InitStructure.NVIC_IRQChannel = KEY2_INT_EXTI_IRQ;
//  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  配置 PA0 为线中断口，并设置中断优先级
  * @param  无
  * @retval 无
  */


//void CAPTURE_EXTI_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure; 
//	EXTI_InitTypeDef EXTI_InitStructure;
//  
//	/*开启按键GPIO口的时钟*/
////	RCC_AHB1PeriphClockCmd(KEY1_INT_GPIO_CLK|KEY2_INT_GPIO_CLK ,ENABLE);
//  	RCC_AHB1PeriphClockCmd(CAPTURE_ICPWM_GPIO_CLK,ENABLE);
//  /* 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟*/
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//  
//  /* 配置 NVIC */
//  NVIC_Configuration();
//  
//	/* 选择按键1的引脚 */ 
//  GPIO_InitStructure.GPIO_Pin =CAPTURE_ICPWM_PIN|CAPTURE_ICPWM_PIN2|CAPTURE_ICPWM_PIN3|CAPTURE_ICPWM_PIN4;
//	
//	
//	
//  /* 设置引脚为输入模式 */ 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	    		
//  /* 设置引脚下拉 */
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  /* 使用上面的结构体初始化按键 */
//  GPIO_Init(CAPTURE_ICPWM_GPIO_PORT, &GPIO_InitStructure); 

//	/* 连接 EXTI 中断源 到key1引脚 */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource6|EXTI_PinSource7|EXTI_PinSource8|EXTI_PinSource9);




//  /* 选择 EXTI 中断源 */
//  EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line8|EXTI_Line9;
//  /* 中断模式 */
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  /* 下降沿触发 */
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
//  /* 使能中断/事件线 */
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//  
////  /* 选择按键2的引脚 */ 
////  GPIO_InitStructure.GPIO_Pin = KEY2_INT_GPIO_PIN;  
////  /* 其他配置与上面相同 */
////  GPIO_Init(KEY2_INT_GPIO_PORT, &GPIO_InitStructure);      

////	/* 连接 EXTI 中断源 到key2 引脚 */
////  SYSCFG_EXTILineConfig(KEY2_INT_EXTI_PORTSOURCE,KEY2_INT_EXTI_PINSOURCE);

////  /* 选择 EXTI 中断源 */
////  EXTI_InitStructure.EXTI_Line = KEY2_INT_EXTI_LINE;
////  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
////  /* 上升沿触发 */
////  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
////  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
////  EXTI_Init(&EXTI_InitStructure);
//}
/*********************************************END OF FILE**********************/
