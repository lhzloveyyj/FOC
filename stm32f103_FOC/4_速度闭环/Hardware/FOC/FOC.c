#include "stm32f10x.h"                  // Device header
#include "foc.h"
#include "math.h"
#include "pwm.h"
#include "fast_sin.h"
#include "usart.h"
#include "delay.h"

#define volatge_high 	12.0f				//电压限制值
#define Udc 			12.0f				//母线电压
#define sqrt3			1.732				//根号3
#define polePairs 	 	11 					// 电机的极对数
#define _2PI 	 		6.2831853 			// 2PI
#define _PI 	 		3.1415927 			// PI
#define _3PI_2			4.71238898038		//PI/3

float  zero = 0.0f;							//零电角度

//幅值限制函数
float limit(float in_vo,float low,float high)
{
	if(in_vo>=high)
		in_vo=high;
	else if(in_vo<=low)
		in_vo=low;
	else
		in_vo=in_vo;
	return in_vo;
}
 
// 电角度 = 机械角度 * 极对数
float _electricalAngle(float jixie_angle)
{
    return (jixie_angle * polePairs - zero);
}

//矫正
void angle_init(float jixie_angle)
{
	setPhaseVoltage(3,0,_3PI_2);
	
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	
	zero = _electricalAngle(jixie_angle);
	
	setPhaseVoltage(0,0,_3PI_2);
	printf("初始化完成\r\n");
	printf("零电位角度:	%lf\r\n",zero);
}

// 把角度值归一化在 [0, 2pi]
float Angle_limit(float angle)
{
    float a = fmod(angle, _2PI); // fmod()函数用于浮点数的取余运算
    return a >= 0.0f ? a : (a + _2PI);
}

float pwm_a=0,pwm_b=0,pwm_c=0;
void setpwm(float Ua,float Ub,float Uc)
{
	//输出限幅
	Ua = limit(Ua,0.0f,volatge_high);
	Ub = limit(Ub,0.0f,volatge_high);
	Uc = limit(Uc,0.0f,volatge_high);
	
	//PWM限幅
	pwm_a = limit(Ua / Udc , 0.0f , 1.0f);
	pwm_b = limit(Ub / Udc , 0.0f , 1.0f);
	pwm_c = limit(Uc / Udc , 0.0f , 1.0f);
	
	//PWM写入
	pwm_a_write(pwm_a * 2600);
	pwm_b_write(pwm_b * 2600);
	pwm_c_write(pwm_c * 2600);
}

// FOC核心函数：输入Uq、Ud和电角度，输出三路PWM
float Ualpha=0.0f,Ubate=0.0f;
float Ua=0.0f,Ub=0.0f,Uc=0.0f;
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	//力矩限制
	Uq = limit(Uq,-Udc/2,Udc/2);
	
	//角度归一化
	angle_el = Angle_limit(angle_el);
	
	//求电角度
	angle_el = _electricalAngle(angle_el);
	
	//park逆变换
	Ualpha = -Uq * fast_sin(angle_el) + Ud * fast_cos(angle_el);
	Ubate  =  Uq * fast_cos(angle_el) + Ud * fast_sin(angle_el);
	
	//clarke逆变换
	Ua = Ualpha + Udc/2;
	Ub = (sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	Uc = (-sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	
	//printf("%lf,%lf,%lf\n",Ua,Ub,Uc);
	setpwm(Ua,Ub,Uc);
}




