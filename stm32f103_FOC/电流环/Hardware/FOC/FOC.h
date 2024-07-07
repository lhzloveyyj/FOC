#ifndef __FOC_H
#define __FOC_H

#include "math.h"

extern float Ua,Ub,Uc,Uq,Ud,Iq,Id,Ialpha,Ibate;

float Angle_limit(float angle);								 	// �ѽǶ�ֵ������ [0, 2pi]
void setPhaseVoltage(float Ia,float Ib,float Ic,float angle_el);		// ��������
float limit(float in_vo,float low,float high);

#endif
