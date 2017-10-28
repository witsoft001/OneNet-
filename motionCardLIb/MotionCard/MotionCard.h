#ifndef _MOTIONCARD_H
#define _MOTIONCARD_H
#include "stm32f4xx.h"

//ע�� ʾ��ģʽ�������洢�� 0x08040000 ��ַ��  stm32f407��Ƭ��



//��Ľṹ�� ��λmm
typedef struct
{
	float x;
	float y;
}Point_t;



typedef struct 
{
	float v1;
	float v2;
	float v3;
}TriWheelVel_t;



//��бʽ�ṹ�� ��б���ýǶ��ƵĽǶȴ���
typedef struct
{
	Point_t point;
	//�Ƕ���
	float   direction;
}Pose_t;



//�û���Ҫ����һ�����ֵĿ��Ƴ���,�����뿴�û��ֲ�
//vel       ��λmm/s
//direction �ٶȵķ���  ��λ ��
//rotationVell ��ת�ٶ� ��λ ��/s 
extern void ThreeWheelVellControl(float vel, float direction, float rotationVell);


//����������������
//vel       ��λmm/s
//direction �ٶȵķ���  ��λ ��
//rotationVell ��ת�ٶ� ��λ ��/s 
//posAngle �����˵���̬  ��λ ��
extern TriWheelVel_t CaculateThreeWheelVel(float vel, float direction, float rotationVell,float posAngle);


//ACCMAXĦ�����͵�����ṩ�������ٶ� mm/s^2
extern float GetAccMax(void);
//VELLMAX�������ܴﵽ������ٶ� mm/s  ��ֵ��ͨ���Ƚ�Ħ�����ṩ�������ٶ��������ṩ�������˵������ٶȣ�ȡ��Сֵ��
extern float GetVelMax(void);



//��Ҫ�ⲿʵ�֣������������̬
extern float GetAngleZ(void);
extern float GetPosx(void);
extern float GetPosy(void);


//�����ڴ棬ʹ��ǰ�������ȵ���  num ����ȡ�������һ����ռ28�ֽ�  
void BufferZizeInit(int num);
//·�����溯�� percent: ��ΧΪ0~1.1����滮������ٶ�
void PathFollowingNew(float percent);
//ʾ�̲ɵ㺯�����ɵ�ʱ����Ҫһֱ����
void FunSampling(void);
//��ȡflash���ʾ�̵���Ϣ������ӵ�ringBuffer��
void ReadFlashPointInformation(void);
//�ɵ����ʱ��Ҫ���ã������ɵ���Ϣ����flash��
void FunsamplingOk(void);
//���ringBuffer,һ����գ����̽���ͣ������
void ClearRingBuffer(void);


//���˵�������
//int GetCount(void);
//float GetRingBufferPointPoseAngle(int num);
//float GetRingBufferPointAngle(int num);
//float GetRingBufferPointVell(int num);
//float GetRingBufferPointLen(int num);
//float GetRingBufferAverCurvature(int num);
//float GetLength(void);

#endif



