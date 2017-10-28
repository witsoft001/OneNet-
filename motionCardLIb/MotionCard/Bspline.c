#include "Bspline.h"
#include "math.h"
#include "ringBuffer.h"



/*********************************************************************************
* @name 	CaculateBsplineLen
* @brief    ���ڼ�������ȷ����һ��B�������ߵĳ���
* @param	point1 ��ʼ��				��λ mm
* @param	point2 ������				��λ mm
* @param	angle1 ��ʼ�㷽��    ��λ ��
* @param	angle2 �����㷽��    ��λ ��
* @retval   ���ظ��������߳���  ��λ mm
*********************************************************************************/
float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2)
{

		//������
		Point_t P[2];
		//��ֵ��
		//Point_t *passPoint = NULL;
		//ϵ������������
		float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
		//���Ƶ�����ʸ�������ǰ��
		float length = 0.0f;
		//��ʱ���Ƶ�����
		Point_t dataPoint[2];
		//���տ��Ƶ�����
		Point_t finalDataPoint[6];
		Point_t tempPoint;
		Point_t tempPointOld;
		int i, j;
		//��B�������߳���
		float u, b1, b2, b3, b0;



		length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

		//�Գ����ֵ    angle��90��Ϊ�˴��������ǽǶ�������ϵ�ǶȲ���
		P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
		P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
		P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
		P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

		//����
		for (i = 0; i < 2; i++)
		{
			dataPoint[i].x = 0;
			dataPoint[i].y = 0;
		}


		//d = inv(a).*P  ����ʱ���Ƶ�����
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
			{
				dataPoint[i].x += invA[i][j] * P[j].x;
				dataPoint[i].y += invA[i][j] * P[j].y;
			}
		}

		//�������տ��Ƶ�����
		finalDataPoint[0] = point1;
		finalDataPoint[1] = point1;
		finalDataPoint[2] = dataPoint[0];
		finalDataPoint[3] = dataPoint[1];
		finalDataPoint[4] = point2;
		finalDataPoint[5] = point2;


		length = 0.0f;
		//��B�����ϵ�һ�㵽�׶˵��ֵ����
		tempPointOld = point1;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 50; j++)
			{
				u = (float)(j)* 0.02f;
				// ������b0
				b0 = (1.0f / 6.0f) * pow(1.0f - u, 3);
				// ������b1
				b1 = (1.0f / 6.0f) * (3.0f * pow(u, 3) - 6 * pow(u, 2) + 4);
				// ������b2��
				b2 = (1.0f / 6.0f) * (-3.0f * pow(u, 3) + 3 * pow(u, 2) + 3 * u + 1);
				// ������b3��
				b3 = (1.0f / 6.0f) * pow(u, 3);

				tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
				tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

				length += CalculatePoint2PointDistance(tempPoint,tempPointOld);

				tempPointOld = tempPoint;
			}
		}
		//��B���������һ�㵽ĩ�˵�ľ������
		length += CalculatePoint2PointDistance(tempPoint, point2);
		
		return length;
	}


	
	
/*********************************************************************************
* @name 	SerchVirtualPoint
* @brief  ���ݾ�������·������ȷ������Ӧ�������
* @param	robotLen ��������·������  ��λ mm 
* @retval ���� ��������� ����Ӧ���������������ϵı���ϵ��
*********************************************************************************/
PointU_t SerchVirtualPoint(float robotLen)
{
	int i = 0;
	PointU_t result;

	//ȷ��Ŀ�곤������һ����
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//ȷ�����ڵ�i-1 �� i �ϵľ���
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	if((i-2) > 0)
	{
			DeleteData(i-2);
	}
	return result;

}



/*********************************************************************************
* @name 	SerchBsplineVirtualPoint
* @brief  ���ݾ��������������߳���ȷ������Ӧ�������
* @param	robotLen ���������������߳���  ��λ mm
* @retval ���� ��������� ����Ӧ���������������ϵı���ϵ��
*********************************************************************************/
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2,float robotlen)
{
	//������
	Point_t P[2];
	//ϵ������������
	float invA[2][2] = { { 0.333333333f, 0.00000f }, { 0.0000f, 0.333333333f } };
	//���Ƶ�����ʸ�������ǰ��
	float length = 0.0f;
	//��ʱ���Ƶ�����
	Point_t dataPoint[2];
	//���տ��Ƶ�����
	Point_t finalDataPoint[6];
	Point_t tempPoint;
	Point_t tempPointOld;
	int i, j;
	//��B�������߳���
	float u, b1, b2, b3, b0;
	PointU_t result;



	length = 0.60f*sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));

	//�Գ����ֵ    angle��90��Ϊ�˴��������ǽǶ�������ϵ�ǶȲ���
	P[0].x = 3 * point1.x + length*cos((angle1) * CHANGE_TO_RADIAN);
	P[0].y = 3 * point1.y + length*sin((angle1) * CHANGE_TO_RADIAN);
	P[1].x = 3 * point2.x - length*cos((angle2) * CHANGE_TO_RADIAN);
	P[1].y = 3 * point2.y - length*sin((angle2) * CHANGE_TO_RADIAN);

	//����
	for (i = 0; i < 2; i++)
	{
		dataPoint[i].x = 0;
		dataPoint[i].y = 0;
	}


	//d = inv(a).*P  ����ʱ���Ƶ�����
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			dataPoint[i].x += invA[i][j] * P[j].x;
			dataPoint[i].y += invA[i][j] * P[j].y;
		}
	}


	//�������տ��Ƶ�����
	finalDataPoint[0] = point1;
	finalDataPoint[1] = point1;
	finalDataPoint[2] = dataPoint[0];
	finalDataPoint[3] = dataPoint[1];
	finalDataPoint[4] = point2;
	finalDataPoint[5] = point2;


	length = 0.0f;
	//��B�����ϵ�һ�㵽�׶˵��ֵ����
	tempPointOld = point1;
	int num = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 50; j++)
		{
			//���ڼ���Uֵ
			num++;

			u = (float)(j)* 0.02f;

			// ������b0
			b0 = (1.0f / 6.0f) * pow(1.0f - u, 3);
			// ������b1
			b1 = (1.0f / 6.0f) * (3.0f * pow(u, 3) - 6 * pow(u, 2) + 4);
			// ������b2��
			b2 = (1.0f / 6.0f) * (-3.0f * pow(u, 3) + 3 * pow(u, 2) + 3 * u + 1);
			// ������b3��
			b3 = (1.0f / 6.0f) * pow(u, 3);

			tempPoint.x = b0*finalDataPoint[i].x + b1*finalDataPoint[i + 1].x + b2*finalDataPoint[i + 2].x + b3*finalDataPoint[i + 3].x;
			tempPoint.y = b0*finalDataPoint[i].y + b1*finalDataPoint[i + 1].y + b2*finalDataPoint[i + 2].y + b3*finalDataPoint[i + 3].y;

			length += CalculatePoint2PointDistance(tempPoint, tempPointOld);
			if (robotlen < length)
			{
				result.point = tempPoint;
				result.u = (float)num / 150.0f;
				return result;
			}

			tempPointOld = tempPoint;
		}
	}

	//��B���������һ�㵽ĩ�˵�ľ������
	result.point = point2;

	result.u = 1.0f;

	return result;
}

//ȷ������Ŀ���
PointU_t SerchVirtualPoint2(float robotLen)
{
	int i = 0;
	PointU_t result;

	//ȷ��Ŀ�곤������һ����
	for (i = 1; i < GetCount(); i++)
	{
		if (robotLen < GetRingBufferPointLen(i))
			break;
	}
	
	
	
	//ȷ�����ڵ�i-1 �� i �ϵľ���
	robotLen -= GetRingBufferPointLen(i-1);
	
	result = SerchBsplineVirtualPoint(GetRingBufferPoint(i - 1), GetRingBufferPoint(i), GetRingBufferPointAngle(i - 1), GetRingBufferPointAngle(i), robotLen);

	return result;

}

