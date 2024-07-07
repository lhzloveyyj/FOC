#include "stm32f10x.h"                  // Device header
#include "PID.H"
#include "foc.H"

//Œª÷√ª∑PID
float angle_err=0.0f,angle_last_err=0.0f,angle_out=0.0f,p=15.0f,d=2.0f;
float angle_pid(float now_angle,float tar_angle)
{
	angle_err = tar_angle-now_angle;
	angle_out=p*angle_err+d*(angle_err-angle_last_err);
	//angle_control(angle_out);
	angle_out = limit(angle_out,-6,6);
    angle_last_err=angle_err;
    return angle_out;
}

void  angle_clear(void)
{
	angle_err=0;
	angle_last_err=0;
	angle_out=0;
}

