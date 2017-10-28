/**
  ******************************************************************************
  * @file    timer.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of time
  ******************************************************************************
  * @attention
  * three are some great and accurency time delay function in timer.h
  *
**/

#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"


/**
  * @brief  Configures the TIMx's interrupt time.
  * @param  TIMx: where x can be 1-14. 
			@note TIM1 and TIM 9,10,11;TIM2 and TIM12,13,14 have some sharing IRQ handle functions.
  * @param  arr: the period value to be loaded into the active
                 Auto-Reload Register at the next update event, range from 1 to 65535
  * @param  psr: the prescaler value used to divide the TIM clock, range from 1 to 65535
  * @note   1.the TIMx's PreemptionPriority and SubPriority has been predefined in this function,
			change them according to your need.
			2.Tout= ((arr+1)*(psc+1))/Tclk s, 
			  for TIM2-7 and TIM12-14, Tclk=84M
			  for TIM1,8,9,10,11 Tclk=168M
			  exp: for TIM2, if arr=999, psc=83, then Tout=(1000*84)/84M=1ms
  * @retval None
  * @author Calcus Lee
  */
void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri)
{
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn ;     
			
			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn ;    
			
			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(TIM1_UP_TIM10_IRQn, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_TRG_COM_TIM11_IRQn ;   
			
			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn ;     

			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn ;    

			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn ;    

			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn ;     

			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn ;     

			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn ;    

			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn ;    

			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;   

			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn ;     

			break;
		}
				
		default: break;
	}
	
	//��ʱ��TIMx��ʼ��
	TIMx_TimeBaseStructure.TIM_Period=arr;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_Prescaler=psr;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIMx
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);                 //��ʼ��ʱ���뽫����ж���0,�����ڿ�����ж�֮ǰ
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);                    //��������ж�
	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=prepri;		//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=subpri;            //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;               //ʹ��IRQͨ��
	NVIC_Init(&NVIC_InitStructure);                             //��ʼ��NVIC�Ĵ���
	
	TIM_Cmd(TIMx,ENABLE);                                       //ʹ��TIMx
}


/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n us you want to delay, range from 1 to 65535
			@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer;
					every time when time= 168/168M s,TIMCounter++ 
  * @retval None
  * @authonr Calcus Lee
  */
void TIM_Delayus(TIM_TypeDef * TIMx, uint16_t Delayus)
{
  uint16_t  TIMCounter = Delayus;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=168;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=84;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		default: break;
	}

	TIMx_TimeBaseStructure.TIM_Period=1;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIM1
	 									
	TIM_Cmd(TIMx,ENABLE);
	TIM_SetCounter(TIMx, 65535-TIMCounter);
	
	while (TIMCounter<65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);	
}


/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n 100us you want to delay, range from 1 to 65535
			@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer
  * @retval None
  * @authonr Calcus Lee
  */


void TIM_Delay100us(TIM_TypeDef * TIMx, uint16_t Delay100us)
{
  uint32_t  TIMCounter=0;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	TIMCounter= Delay100us;
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=16800;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ

			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			TIMx_TimeBaseStructure.TIM_Prescaler=8400;        		    //����ʱ�ӷ�Ƶ������Ԥ��Ƶֵ
			
			break;
		}
		default: break;
	}

	TIMx_TimeBaseStructure.TIM_Period=1;						//�����Զ���ת�ؼĴ������ڵ�ֵ
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //����ʱ�ӷָ�
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM���ϼ���
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //��ʼ��TIM1
 					
	
	TIM_SetCounter(TIMx, 65535-TIMCounter);
	TIM_Cmd(TIMx,ENABLE);
	while (TIMCounter<65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);

}

/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs:
  * @retval None
  * @authonr Calcus Lee
  */
void TIM_Delayms(TIM_TypeDef * TIMx, uint32_t DelayMs)
{
    uint32_t i=0;

	for(i=0;i<DelayMs;i++)
	{
		TIM_Delay100us(TIMx,10);
	}
}



