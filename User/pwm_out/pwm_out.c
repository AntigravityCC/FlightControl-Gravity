#include "include.h"



void ESC_pwm_enable(void)
{    
//    TIM_CCxCmd( TIM4, TIM_Channel_1,  TIM_CCx_Enable);
//    TIM_CCxCmd( TIM4, TIM_Channel_2,  TIM_CCx_Enable);
//    TIM_CCxCmd( TIM4, TIM_Channel_3,  TIM_CCx_Enable);
//    TIM_CCxCmd( TIM4, TIM_Channel_4,  TIM_CCx_Enable);
}
void ESC_pwm_disable(void)
{  
//    TIM_CCxCmd( TIM4, TIM_Channel_1,  TIM_CCx_Disable);
//    TIM_CCxCmd( TIM4, TIM_Channel_2,  TIM_CCx_Disable);
//    TIM_CCxCmd( TIM4, TIM_Channel_3,  TIM_CCx_Disable);
//    TIM_CCxCmd( TIM4, TIM_Channel_4,  TIM_CCx_Disable);
}

//    Delay_ms(100);
//    TIM_SetCompare1(TIM3,2000);      
//    TIM_SetCompare2(TIM3,2000);  
//    TIM_SetCompare3(TIM3,2000); 
//    TIM_SetCompare4(TIM3,2000);  
//    
// 


void  ESC_Out(int a,int b,int c,int d)
{

	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1000+a);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1000+b);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1000+c);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,1000+d);
	
}

void ESC_Init(void)
{
    HAL_Delay(1000);

	
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1000);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1000);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1000);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,1000);
     HAL_Delay(1000);
	HAL_Delay(1000);
}

void ESC_Set(void)
{

//       ESC_pwm_enable();
//     Delay_ms(1000);
//   ESC_Out(1000,1000,1000,1000);
////   ESC_pwm_disable();

//	 Delay_ms(2000);
//   ESC_pwm_enable();
	
//   ESC_Out(0,0,0,0);
//   Delay_ms(2000);
//	   Delay_ms(2000);
//		   Delay_ms(2000);
//		   Delay_ms(2000);
}






