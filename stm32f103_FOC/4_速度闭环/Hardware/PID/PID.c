#include "stm32f10x.h"                  // Device header
#include "PID.H"
#include "FOC.H"

//位置环PID
float angle_err=0.0f,angle_last_err=0.0f,angle_out=0.0f,p=10.0f,d=1.0f;
float angle_pid(float now_angle,float tar_angle)
{
	angle_err = tar_angle-now_angle;
	angle_out=p*angle_err+d*(angle_err-angle_last_err);
    angle_last_err=angle_err;
	angle_out = limit(angle_out,-10,10);
    return angle_out;
}

void  angle_clear(void)
{
	angle_err=0;
	angle_last_err=0;
	angle_out=0;
}


//速度环PID
float Err=0,last_err=0,next_err=0,out=0,add=0,P=1,I=0.1;
float speed_pid(float speed,float tar)
{
    Err=tar-speed;
    add=P*(Err-last_err)+I*(Err);
	add=limit(add,-6,6);
    out+=add;
	out = limit(out,-6,6);
    next_err=last_err;
    last_err=Err;
    return out;
}

void  speedd_clear(void)
{
	Err=0;
	last_err=0;
	next_err=0;
	out=0;
	add=0;
}
