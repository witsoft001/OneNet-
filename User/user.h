#ifndef _USER_H
#define _USER_H

//�û���Ҫ����һ�����ֵĿ��Ƴ���,�����뿴�û��ֲ�
//speed ��λm/s
//angle �ٶȵķ���
//angularVell ��ת�ٶ� ��λ����/s
extern void ThreeWheelVellControl(float speed, float angle, float angularVell);

//ACCMAXĦ�����͵�����ṩ�������ٶ� mm/s^2
#define ACCMAX 179.0f
//VELLMAX�������ܴﵽ������ٶ� mm/s  ��ֵ��ͨ���Ƚ�Ħ�����ṩ�������ٶ��������ṩ�������˵������ٶȣ�ȡ��Сֵ��
#define VELLMAX  600.0f












#endif
