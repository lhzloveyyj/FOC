#include "at32f403a_407.h"              // Device header
#include "foc.h"
#include "math.h"
#include "fast_sin.h"
#include "delay.h"
#include "stdio.h"
#include "pid.h"
#include "as5600.h"
#include "at32f403a_407_int.h"

#define volatge_high 	12.0f				//电压限制值
#define Udc 			12.0f				//母线电压
#define sqrt3			1.732f				//根号3
#define sqrt3_2			0.866f				//根号3/2
#define polePairs 	 	11 					// 电机的极对数
#define _2PI 	 		6.28318f 			// 2PI
#define _PI 	 		3.14159f 			// PI
#define _3PI_2			4.712388f			//PI/3
#define _1_SQRT3 		0.577350f			//根号三分之一
#define Gain 			10.0f				//电流缩小倍数

float  zero = 0.0f;							//零电角度
uint16_t AD_Value[3]={0};
float  Iq_tar = 30.0f;	

void open(float Uq,float Ud,float angle_el);

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

// 把角度值归一化在 [0, 2pi]
float Angle_limit(float angle)
{
    float a = fmod(angle, _2PI); // fmod()函数用于浮点数的取余运算
    return a >= 0.0f ? a : (a + _2PI);
}

// 电角度 = 机械角度 * 极对数
float _electricalAngle(float shaft_angle)
{
    return Angle_limit(shaft_angle * polePairs - zero);
}


//矫正
void angle_init(void)
{
	open(0.0f,1.0f,0.0f);
	delay_ms(100);;
	zero = _electricalAngle(angle_get());
	float zero_angle = _electricalAngle(angle_get());
	printf("初始化完成\r\n");
	printf("零电位角度:	%lf		%lf\r\n",zero,zero_angle);
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
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, pwm_a * 4000);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pwm_b * 4000);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, pwm_c * 4000);
	
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_2, 3950);
}

// FOC核心函数：输入Uq、Ud和电角度，输出三路PWM
float Ualpha=0.0f,Ubate=0.0f;
float Ialpha=0.0f,Ibate=0.0f;

float Ua=0.0f,Ub=0.0f,Uc=0.0f;
float Uq=0.0f,Ud=0.0f;
float Iq=0.0f,Id=0.0f;
void setPhaseVoltage(uint16_t Ia, uint16_t Ib, uint16_t Ic,float angle_el)
{
	Ia = Ia / Gain;
	Ib = Ib / Gain;
	Ic = Ic / Gain;

	//力矩限幅
	Uq = limit(Uq,-Udc/2,Udc/2);
	
	//求电角度
	angle_el = _electricalAngle(angle_el);

	//clarke变换
	Ialpha = Ia - Ib * 0.5f -Ic * 0.5f;
	Ibate  = Ib * sqrt3_2 - Ic * sqrt3_2;

	//park变换
	Id = Ialpha * fast_cos(angle_el) + Ibate * fast_sin(angle_el);
	Iq = -Ialpha * fast_sin(angle_el) + Ibate * fast_cos(angle_el);

	//低通滤波
	Iq = LowPass_Filter(Iq,0);
	Id = LowPass_Filter(Id,1);
	
	//PID求Id,Iq
	Ud = Id_pid(Id,0.0f);
	Uq = Iq_pid(Iq,tar);
	
	//park逆变换
	Ualpha = -Uq * fast_sin(angle_el) + Ud * fast_cos(angle_el);
	Ubate  =  Uq * fast_cos(angle_el) + Ud * fast_sin(angle_el);
	
	//clarke逆变换
	Ua = Ualpha + Udc/2.0f;
	Ub = (sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	Uc = (-sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	
	setpwm(Ua,Ub,Uc);
}

void open(float uq,float ud,float angle_el)
{
	//求电角度
	angle_el = _electricalAngle(angle_el);
	
	//park逆变换
	Ualpha = -uq * fast_sin(angle_el) + ud * fast_cos(angle_el);
	Ubate  =  uq * fast_cos(angle_el) + ud * fast_sin(angle_el);
	
	//clarke逆变换
	Ua = Ualpha + Udc/2.0f;
	Ub = (sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	Uc = (-sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	
	setpwm(Ua,Ub,Uc);
}

//上电读取偏置电压
void first_get(uint16_t *First_a,uint16_t *First_b,uint16_t *First_c)
{
	for(int i=0;i<16;i++)
	{
		*First_a += AD_Value[0];
		*First_b += AD_Value[1];
		*First_b += AD_Value[2];
	}
	*First_a = *First_a>>4;
	*First_b = *First_b>>4;
	*First_c = *First_c>>4;
}


