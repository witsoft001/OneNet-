#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void MyBeefInit(void);

//�г̿��س�ʼ��
void MySwitchInit(void);


void BeefON(void);

void BeefOFF(void);

//����1 ������
//����0 ����δ����
_Bool ReadSwitch(void);

void LedTwinkle(void);

#endif
