/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   dubug.h
*Author��      Peng Xu
*Date��        2016/10/21
*Description�� debug.c��ͷ�ļ�
*
*Version��     V1.0
*
********************************************************************/


#ifndef _DEBUG_H
#define _DEBUG_H


#define SENDUSART  USART2
#include "usart.h"





/*********************************************************************
* @name 	USART_OUT_XYAngle
* @brief  	����x��y�����zangle ������ȫ�ֱ�������debug
* @param  	��
* @retval 	��
********************************************************************/
void USART_OUT_XYAngle(void);

void UsartSendXP3(int a,int b,int c);
void UsartSendXP2(int a,int b);
void UsartSendXP1(int a);

#endif
