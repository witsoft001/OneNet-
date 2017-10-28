/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "update.h"
#include "debug.h"
#include "string.h"
#include <stdlib.h>
#include "elmo.h"
#include "user.h"
#include "update.h"
#include "MotionCard.h"
#include "stm32f4xx.h"
#include "sim800c.h"
#include "threeWheelMove.h"
#include "stm32f4xx_it.h"
//����ģʽ
#define NORMAL_MODE 1
//�ɼ�ģʽ
#define SAMPLING_MODE 2
#define MOTOR_USE 1
//�ж����ݴ�Ŷ���  500��ֹ��������ʾ��վ����
extern uint8_t Receive_Buffer[500]; 

static int  mode = 1;

_Bool GyroscopeFlag = 0;


void Init(void)
{
	int i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	//CAN1��ʼ�������ڵ������
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	
	
	//�����ǽ��մ���
	GyroscopeUsartInit(115200);
	

	//��������
//	WirelessBluetoothUsartInit(115200);
//	WirelessBluetoothUsartInit(9600);
	
	//��ʪ�ȴ���������
	Usart6Init(9600);
	//SIM800����
	uart_initPA2(115200);
	//���Դ���
	uart_initpb10(115200);

	//��������ʼ��
	MyBeefInit();	
	
	//�г̿���
	MySwitchInit();
	
	
	//��ʱ����ʼ�� 10ms
	TIM_Init(TIM2, 999, 839, 3, 3);
	
	//��ʼ���ڴ��
	BufferZizeInit(500);
	
	BeefON();

	TIM_Delayms(TIM5, 1000);
	
	BeefOFF();
	
	
	while(GyroscopeFlag == 0)
	{
		//����ȴ������ǳ�ʼ���ڼ䷢������������������в���ģʽ
		if(ReadSwitch() == 1)
		{
				BeefON();
				mode = SAMPLING_MODE;
		}
		else
		{
				BeefOFF();
		}
	}
	
	
	//�����ʼ��
if(mode == NORMAL_MODE)
{
		elmo_Enable(1);
		elmo_Enable(2);
	  elmo_Enable(3);
		
		Vel_cfg(1,13000,13000);
		Vel_cfg(2,13000,13000);
		Vel_cfg(3,13000,13000);

}
	
	//�������Ϊ
	for(i=0;i<SIM800_BUFFER_SIZE;i++)
	{
		Receive_Buffer[i]=NO_DATA;
	}
	//���TC��־λ�������͵ĵ�һ������������
	USART_ClearFlag(SEND_TO_SIM800, USART_FLAG_TC);
	USART_ClearFlag(RECEIVE_SIM800, USART_FLAG_TC);
BeefON();
}






extern _Bool USARTSEND;
extern long Time_count_10ms;
uint8_t number_char[10];
	static int pos_x_last=0;

int main(void)
{
	Pose_t pos_end,pos_end2;
	uint32_t numCount = 0;
	int i=0;
	v_three dipan;
	static uint8_t state=0;
	static int count_sim=0;
	int pos_x_temp=0;
	static int state_cir=0;
	//��ʼ������
	Init();
	
	while(1)
	{
		if(ReadSwitch() == 1)
		{
			numCount++;
			if(numCount > 1000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	BeefOFF();
	while(1)
	{
		if(ReadSwitch() == 0)
		{
			numCount++;
			if(numCount > 2000)
			{
				break;
			}
		}
		else
		{
			numCount = 0;
		}
	}
	
	while(1)
	{
		

//	switch(mode)
//		{
//			
//			case NORMAL_MODE:

//					ReadFlashPointInformation();
////						numCount = GetCount();
////						USART_OUT(SENDUSART,(uint8_t *)"%d\r\n",numCount);	
////						for(int i = 1;i <= numCount;i++)
////						{
////							USART_OUT(SENDUSART,(uint8_t *)"%d\t %d\t %d\t %d\t %d\t %d\t %d\t\r\n",
////							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i),
////							(int)GetRingBufferPointPoseAngle(i),(int)GetRingBufferPointLen(i),(int)GetRingBufferAverCurvature(i),(int)GetRingBufferPointVell(i));	
////						}
//					mode = 3;
//				break;
//			
//			//�ɼ�ģʽ
//			case SAMPLING_MODE:
//				
//				PoseSampling();
//			
//				if(ReadSwitch() == 1)
//				{
//					static int timeCount = 0;
//					
//					timeCount++;
//					
//					//����г̿��ذ���һ��ʱ�䣬������¼����Ϣд��flash
//					if(timeCount > 2000)
//					{						

//						PoseSamplingDone();
////						for(int i = 1;i <= numCount;i++)
////						{
////							USART_OUT(SENDUSART,(uint8_t *)"%dx%dy%da",																								
////							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i));
////						}
//						
////						USART_OUT(SENDUSART,(uint8_t *)"%d\r\n",numCount);	
////						for(int i = 1;i <= numCount;i++)
////						{
////							USART_OUT(SENDUSART,(uint8_t *)"%d\t %d\t %d\t %d\t %d\t %d\t %d\t\r\n",
////							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i),
////							(int)GetRingBufferPointPoseAngle(i),(int)GetRingBufferPointLen(i),(int)GetRingBufferAverCurvature(i),(int)GetRingBufferPointVell(i));	
////						}
//							
//						BeefON();
//						while(1);
//					}
//				}

//				break;
//				
//				
//			case 3:
//						PathFollowing(1);
//		
//						if(USARTSEND == 1)
//						{
//							USARTSEND = 0;
//						}

//						
//				break;					
//			default:
//				
//				break;
//		
//		}


						USART_OUT(RECEIVE_SIM800,(uint8_t *)"x=%d\ty=%d\r\n",(int)GetPosx(),(int)GetPosy());	


		switch(state_cir)
		{
			case 0:
					dipan=SpeedControlWheel(300.0,45.0);state_cir=1;
					VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	

				break;
			case 1:
				if(GetPosy()>600.0)
				{
					dipan=SpeedControlWheel(300.0,-45.0);state_cir=2;
							VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	

				}
					
				break;
			case 2:
				if(GetPosy()<0.0)
				{
					dipan=SpeedControlWheel(300.0,180.0);state_cir=3;
							VelControlTriWheel(dipan.v1,dipan.v2,dipan.v3);	

				}
				break;
			case 3:
				if(GetPosx()<0.0)
				{
					state_cir=0;
				}
				break;
			default :
				break;
			
		}

		sim800c_main();


	}



	
	

	

	


}



