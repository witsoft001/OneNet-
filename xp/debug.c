/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   debug.c
*Author��      Peng Xu
*Date��        2016/10/28
*Description�� ���ڴ��ڷ���
*		       
*
*Version��     
*
********************************************************************/


#include "debug.h"
#include "update.h"




/*********************************************************************
* @name 		USART_OUT_XYAngle
* @brief  	����x��y�����zangle ������ȫ�ֱ�������debug
* @param  	��
* @retval 	��
********************************************************************/
	float g_zA;
	float g_X;
	float g_Y;
void USART_OUT_XYAngle(void)
{

	g_zA = GetAngleZ();
	g_X  = GetPosx();
	g_Y  = GetPosy();
	USART_OUT(SENDUSART,(uint8_t *)"%d\t%d\t%d\r\n",(int)g_X,(int)g_Y,(int)g_zA);
}


void UsartSendXP3(int a,int b,int c)
{
	USART_OUT(SENDUSART,(uint8_t *)"%d\t%d\t%d\t\r\n",(int)a,(int)b,(int)c);
}


void UsartSendXP2(int a,int b)
{
	USART_OUT(SENDUSART,(uint8_t *)"%d\t%d\t\r\n",(int)a,(int)b);
}

void UsartSendXP1(int a)
{
	USART_OUT(SENDUSART,(uint8_t *)"%d\t\r\n",(int)a);
}




