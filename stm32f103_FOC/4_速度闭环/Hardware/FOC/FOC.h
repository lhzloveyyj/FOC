#ifndef __FOC_H
#define __FOC_H

#include "math.h"

extern float Ua,Ub,Uc;
float Angle_limit(float angle);								 	// �ѽǶ�ֵ������ [0, 2pi]
void setPhaseVoltage(float Uq, float Ud, float angle_el);		// ��������
float limit(float in_vo,float low,float high);					//��ֵ���ƺ���
void angle_init(float jixie_angle);								//��ʼ��

#endif
