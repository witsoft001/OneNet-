/********************************************************************
*Copyright(C��2014-2016,��������������˼��������������ι�˾
*FileName��	   ringBuffer.c
*Author��      Peng Xu
*Date��        2016/10/28
*Description�� ��������Ľ���
*		       
*
*Version��     V1.0
*
********************************************************************/

#include "ringbuffer.h"
#include "MotionCard.h"
#include <stdlib.h>



//���λ������е�Ԫ��������  ע����Щֵ�ڿ��ƿ�����ʹ��ʱ���ܻ������������
static int      upPointer = 0;
static int      g_iput = 0;
static int      downPointer = 0;
static float    lengthSum = 0.0f;
static int      countMax = 0;



//������
KeyPointInf_t *ringBuffer;
	
	
	
/*********************************************************************************
* @name 	BufferZizeInit
* @brief	����س�ʼ������
* @param	num ����ȡ�������һ����ռ28�ֽ�;
* @retval ��
**********************************************************************************/
void BufferZizeInit(int num)
{
	countMax = num;
	ringBuffer = (KeyPointInf_t*)malloc(sizeof(KeyPointInf_t) * countMax);
}
	
/*********************************************************************
* @name 		addring
* @brief  	���λ������ĵ�ַ��ż��㺯����������﻽�ѻ�������β�������ƻص�ͷ����
						���λ���������Ч��ַ���Ϊ��0��(COUNTMAX-1)
* @param  	i��������ָ��ֵ
* @retval 	��Ӧ����ı��
********************************************************************/
static int addring(int i)
{
	return (i + 1) == countMax ? 0 : i + 1;
}



/*********************************************************************
* @name 		addring
* @brief  	���λ������з���һ����
* @param  	pose����ӵ�Ԫ��
* @retval 	�����Ƿ񽫻����������� 1��δ���� 0���Ѿ�����
********************************************************************/
int PutRingBuffer(KeyPointInf_t pose)
{
	if ((upPointer - downPointer) < countMax - 2)
	{
		ringBuffer[g_iput] = pose;
		g_iput = addring(g_iput);
		upPointer++;
		return 1;
	}
	//buffer����
	else
	{
		return 0;
	}
}



/*********************************************************************
* @name 		GetRingBufferPoint
* @brief  	����һ����    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ������
********************************************************************/
Point_t GetRingBufferPoint(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].point;
}


//�����ٶ�
void SetRingBufferPointVell(int num,float vell)
{
	int realNum;
	realNum = num - 1 + downPointer;
	ringBuffer[realNum % countMax].vellMax = vell;
}


//�����ٶ�
float GetRingBufferPointVell(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].vellMax;
}


/*********************************************************************
* @name 		GetRingBufferPointAngle
* @brief  	����һ���Ƕ�    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ��Ƕ���Ϣ
********************************************************************/
float GetRingBufferPointAngle(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].angle;
}


/*********************************************************************
* @name 		GetRingBufferPointPoseAngle
* @brief  	����һ�����ֵ���̬�Ƕ�    
* @param  	numȡ1����ȡ������ĵ�һ������û�е�0����
* @retval 	������ȡ��Ƕ���Ϣ
********************************************************************/
float GetRingBufferPointPoseAngle(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].poseAngle;
}

//���ظĵ������·������
float GetRingBufferPointLen(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].length;
}


//���ظöε����ʰ뾶ƽ��ֵ
float GetRingBufferAverCurvature(int num)
{
	int realNum;
	realNum = num - 1 + downPointer;
	return ringBuffer[realNum % countMax].curvatureR;
}



/*********************************************************************
* @name 		GetCount
* @brief  	���ش�����Ԫ�ظ���    
* @param  	��
* @retval 	��
********************************************************************/
int GetCount(void)
{
	return (upPointer - downPointer);
}




/*********************************************************************
* @name 		GetUpPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
int GetUpPointer(void)
{
	return upPointer;
}


/*********************************************************************
* @name 		GetDownPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
int GetDownPointer(void)
{
	return downPointer;
}


/*********************************************************************
* @name 		DeleteData
* @brief  	ɾ�����ݵ�  
* @param  	num ��Ҫɾ����ĸ������ӵ�һ���㿪ʼɾ��
* @retval 	��
********************************************************************/
void DeleteData(int num)
{
	downPointer = num + downPointer;
}



/*********************************************************************
* @name 		SetUpPointer
* @brief  	������ָ��   
* @param  	��
* @retval 	��
********************************************************************/
void SetUpPointer(int num)
{
	upPointer = num;
} 

//��ȡ����ص��׵�ַ
u32* GetFristAdress(void)
{
	return (u32 *)ringBuffer;
}


//��ȡ�յ������·������
float GetLength(void)
{
	return lengthSum;
}


//�����յ������·������
void SetLength(float len)
{
	lengthSum = len;
}


//���ringBuffer
void ClearRingBuffer(void)
{
	upPointer = 0;
	g_iput = 0;
	downPointer = 0;
	lengthSum = 0.0f;
}
