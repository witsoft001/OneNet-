#ifndef _THREEWHEELMOVE_H
#define _THREEWHEELMOVE_H
#include "MotionCard.h"


//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    0.017453f   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     57.2958f				
//圆周率
#define PI                  3.1415926f

#define NULL 0
typedef struct v_three{
	float v1;
	float v2;
	float v3;
}v_three;

//减速比
#define REDUCTION (1.0f)
//车轮半径 单位:mm
#define WHEELRADIUS 50.0f
//每圈脉冲数
#define STDPULSE 4096.0f

float angle_to_hudu(float angle);
v_three SpeedControlWheel(float v,float angle);

#endif

