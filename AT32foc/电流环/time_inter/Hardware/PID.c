#include "at32f403a_407.h"              // Device header

#include "PID.H"
#include "FOC.H"

//???PID
float Err=0.0f,last_err=0.0f,next_err=0.0f,out=0.0f,add=0,P=-0.1f,I=-0.05f;
float Iq_pid(float speed,float tar)
{
    Err=tar-speed;
    add=P*(Err-last_err)+I*(Err);
	add = limit(add,-4.0f,4.0f);
    out+=add;
	out = limit(out,-6.0f,6.0f);
    next_err=last_err;
    last_err=Err;
    return out;
}

float Err_2=0.0f,last_err_2=0.0f,next_err_2=0.0f,out_2=0.0f,add_2=0.0f,P_2=-1.0f,I_2=-0.5f;
float Id_pid(float speed,float tar)
{
    Err_2=tar-speed;
    add_2=P_2*(Err_2-last_err_2)+I_2*(Err_2);
	add_2 = limit(add_2,-4.0f,4.0f);
    out_2+=add_2;
	out_2 = limit(out_2,-6.0f,6.0f);
    next_err_2=last_err_2;
    last_err_2=Err_2;
    return out_2;
}

//?????
static float y_prev,y_prev2;
float LowPass_Filter(float x,int flag)
{	
	float ret;
	if(flag == 0)
	{
		float y = 0.9f*y_prev + 0.1f*x;
		y_prev=y;
		ret = y;
	}
	
	if(flag == 1)
	{
		float y2 = 0.9f*y_prev2 + 0.1f*x;
		y_prev2=y2;
		ret = y2;
	}
	return ret;
}
