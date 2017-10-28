#ifndef _THREEWHEELMOVE_H
#define _THREEWHEELMOVE_H
#include "MotionCard.h"


//�Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN    0.017453f   
//������ת��Ϊ�Ƕ���ϵ��
#define CHANGE_TO_ANGLE     57.2958f				
//Բ����
#define PI                  3.1415926f

#define NULL 0
typedef struct v_three{
	float v1;
	float v2;
	float v3;
}v_three;

//���ٱ�
#define REDUCTION (1.0f)
//���ְ뾶 ��λ:mm
#define WHEELRADIUS 50.0f
//ÿȦ������
#define STDPULSE 4096.0f

float angle_to_hudu(float angle);
v_three SpeedControlWheel(float v,float angle);

#endif

