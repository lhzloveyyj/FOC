#ifndef __FOC_H
#define __FOC_H

#include "math.h"

extern float Ua,Ub,Uc;
float Angle_limit(float angle);								 	// 把角度值限制在 [0, 2pi]
void setPhaseVoltage(float Uq, float Ud, float angle_el);		// 驱动函数
float limit(float in_vo,float low,float high);
void angle_init(void);

#endif
