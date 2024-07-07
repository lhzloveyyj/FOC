#ifndef __FOC_H
#define __FOC_H

#include "math.h"
#include "at32f403a_407.h"              // Device header


extern float Ua,Ub,Uc;
extern uint16_t AD_Value[3];
extern float Ialpha,Ibate;
extern float Iq,Id;
extern float Uq,Ud;
extern float Iq_tar;

float Angle_limit(float angle);								 	// �ѽǶ�ֵ������ [0, 2pi]
void setPhaseVoltage(uint16_t Ia, uint16_t Ib, uint16_t Ic,float angle_el);		// ��������
float limit(float in_vo,float low,float high);
void angle_init(void);
void first_get(uint16_t *First_a,uint16_t *First_b,uint16_t *First_c);

#endif
