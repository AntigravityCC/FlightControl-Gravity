#include "include.h"

u16 rc_in[8];
u16 Rc_Pwm_In[8];



void CAPTURE_EXIT_IRQHandler(void)
{
//	static  uint16_t IC1Value0 ,IC2Value0, IC3Value0,IC4Value0=0;
//  static  uint16_t IC1Value1 ,IC2Value1, IC3Value1,IC4Value1=0;
//	
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(EXTI_Line6) != RESET) 
	{
//		 printf("rc_in[0] %d\r\n" ,rc_in[0]);
//		if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT,CAPTURE_ICPWM_PIN))
//				{
//					IC1Value0++;
//				}
//		else
//				{
//					IC1Value1 =IC1Value0;
//					
//					rc_in[0] = IC1Value1;
//				}
		
		
		// LED1 取反		
		LED1_TOGGLE;
    //清除中断标志位
		EXTI_ClearITPendingBit(EXTI_Line6);     
	}  
	
//		if(EXTI_GetITStatus(EXTI_Line7) != RESET) 
//	{
//		// LED1 取反		
//		LED1_TOGGLE;
//    //清除中断标志位
//		EXTI_ClearITPendingBit(EXTI_Line7);     
//	} 
	
}




void TIM3_IRQHandler(void)	
{
		static  uint16_t IC1Value0 ,IC2Value0, IC3Value0,IC4Value0=0;
	  static  uint16_t IC1Value1 ,IC2Value1, IC3Value1,IC4Value1=0;


			if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC1) != RESET)
			{
			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC1);
      CAPTURE_TIM->SR = ~TIM_FLAG_CC1OF;
//				
				if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT,CAPTURE_ICPWM_PIN))
				{
					IC1Value0 = TIM_GetCapture1(CAPTURE_TIM);
				}
				else
				{
					IC1Value1 = TIM_GetCapture1(CAPTURE_TIM);
					if(IC1Value1>=IC1Value0)
						rc_in[0] = IC1Value1-IC1Value0;
					else
						rc_in[0] = 0xffff-IC1Value1+IC1Value0+1;				
				}
//				 printf("rc_in1[0] %d\r\n",rc_in[0]);
					
			}

			
			
			
			
	////////////////////////////////////////////////////////////////////////
		
//		if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC2) != RESET)
//		{
//			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC2);
//			CAPTURE_TIM->SR = ~TIM_FLAG_CC2OF;
//			
//			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT,CAPTURE_ICPWM_PIN2))
//			{
//				IC2Value0 = TIM_GetCapture2(CAPTURE_TIM);
//			}
//			else
//			{
//				IC2Value1 = TIM_GetCapture2(CAPTURE_TIM);
//				if(IC2Value1>=IC2Value0)
//					rc_in[1] = IC2Value1-IC2Value0;
//				else
//					rc_in[1] = 0xffff-IC2Value1+IC2Value0+1;				
//			}
////			 printf("rc_in[1] %d\r\n",rc_in[1]);
//			
//		}
//		

//		////////////////////////////////////////////////////////////////////////
//	
			if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC3) != RESET)
		{
			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC3);
			CAPTURE_TIM->SR = ~TIM_FLAG_CC3OF;
			
			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT2,CAPTURE_ICPWM_PIN3))
			{
				IC3Value0 = TIM_GetCapture3(CAPTURE_TIM);
			}
			else
			{
				IC3Value1 = TIM_GetCapture3(CAPTURE_TIM);
				if(IC3Value1>=IC3Value0)
					rc_in[2] = IC3Value1-IC3Value0;
				else
					rc_in[2] = 0xffff-IC3Value1+IC3Value0+1;				
			}
//			 printf("rc_in[2] %d\r\n",rc_in[2]);
			
		}
}
//		////////////////////////////////////////////////////////////////////////

//			if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC4) != RESET)
//		{
//			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC4);
//			CAPTURE_TIM->SR = ~TIM_FLAG_CC4OF;
//			
//			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT2,CAPTURE_ICPWM_PIN4))
//			{
//				IC4Value0 = TIM_GetCapture4(CAPTURE_TIM);
//			}
//			else
//			{
//				IC4Value1 = TIM_GetCapture4(CAPTURE_TIM);
//				if(IC4Value1>=IC4Value0)
//					rc_in[3] = IC4Value1-IC4Value0;
//				else
//					rc_in[3] = 0xffff-IC4Value1+IC4Value0+1;				
//			}
//			 printf("rc_in[3] %d\r\n",rc_in[3]);
//			
//		}
//		

//}



//void CAPTURE_TIM_IRQHandler1(void)	
//{
//		static  uint16_t IC1Value0 ,IC2Value0, IC3Value0,IC4Value0=0;
//	  static  uint16_t IC1Value1 ,IC2Value1, IC3Value1,IC4Value1=0;


//			if( TIM_GetITStatus(CAPTURE_TIM1, TIM_IT_CC1) != RESET)
//			{
//				TIM_ClearITPendingBit(CAPTURE_TIM1, TIM_IT_CC1);
//				CAPTURE_TIM1->SR = ~TIM_FLAG_CC1OF;
//				
//				if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT1,CAPTURE_ICPWM_PIN11))
//				{
//					IC1Value0 = TIM_GetCapture1(CAPTURE_TIM1);
//				}
//				else
//				{
//					IC1Value1 = TIM_GetCapture1(CAPTURE_TIM1);
//					if(IC1Value1>=IC1Value0)
//						rc_in[2] = IC1Value1-IC1Value0;
//					else
//						rc_in[2] = 0xffff-IC1Value1+IC1Value0+1;				
//				}
//				 printf("rc_in[2] %d\r\n",rc_in[2]);
//				
//			}

//	////////////////////////////////////////////////////////////////////////
//		
//		if( TIM_GetITStatus(CAPTURE_TIM1, TIM_IT_CC2) != RESET)
//		{
//			TIM_ClearITPendingBit(CAPTURE_TIM1, TIM_IT_CC2);
//			CAPTURE_TIM1->SR = ~TIM_FLAG_CC2OF;
//			
//			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT1,CAPTURE_ICPWM_PIN21))
//			{
//				IC2Value0 = TIM_GetCapture2(CAPTURE_TIM1);
//			}
//			else
//			{
//				IC2Value1 = TIM_GetCapture2(CAPTURE_TIM1);
//				if(IC2Value1>=IC2Value0)
//					rc_in[3] = IC2Value1-IC2Value0;
//				else
//					rc_in[3] = 0xffff-IC2Value1+IC2Value0+1;				
//			}
//			 printf("rc_in[3] %d\r\n",rc_in[3]);
//			
//		}
//		

		////////////////////////////////////////////////////////////////////////
	
//			if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC3) != RESET)
//		{
//			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC3);
//			CAPTURE_TIM->SR = ~TIM_FLAG_CC3OF;
//			
//			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT,CAPTURE_ICPWM_PIN3))
//			{
//				IC3Value0 = TIM_GetCapture3(CAPTURE_TIM);
//			}
//			else
//			{
//				IC3Value1 = TIM_GetCapture3(CAPTURE_TIM);
//				if(IC3Value1>=IC3Value0)
//					rc_in[2] = IC3Value1-IC3Value0;
//				else
//					rc_in[2] = 0xffff-IC3Value1+IC3Value0+1;				
//			}
//			 printf("rc_in[2] %d\r\n",rc_in[2]);
//			
//		}

//		////////////////////////////////////////////////////////////////////////

//			if( TIM_GetITStatus(CAPTURE_TIM, TIM_IT_CC4) != RESET)
//		{
//			TIM_ClearITPendingBit(CAPTURE_TIM, TIM_IT_CC4);
//			CAPTURE_TIM->SR = ~TIM_FLAG_CC4OF;
//			
//			if (GPIO_ReadInputDataBit(CAPTURE_ICPWM_GPIO_PORT,CAPTURE_ICPWM_PIN4))
//			{
//				IC4Value0 = TIM_GetCapture4(CAPTURE_TIM);
//			}
//			else
//			{
//				IC4Value1 = TIM_GetCapture4(CAPTURE_TIM);
//				if(IC4Value1>=IC4Value0)
//					rc_in[3] = IC4Value1-IC4Value0;
//				else
//					rc_in[3] = 0xffff-IC4Value1+IC4Value0+1;				
//			}
//			 printf("rc_in[3] %d\r\n",rc_in[3]);
//			
//		}
		


//}

//}

