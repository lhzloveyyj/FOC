#include "at32f403a_407.h"              // Device header
#include "foc.h"
#include "math.h"
#include "fast_sin.h"
#include "delay.h"
#include "stdio.h"
#include "as5600.h"

#define volatge_high 	12.0f				//��ѹ����ֵ
#define Udc 			12.0f				//ĸ�ߵ�ѹ
#define sqrt3			1.732f				//����3
#define polePairs 	 	11 					// ����ļ�����
#define _2PI 	 		6.28318f 			// 2PI
#define _PI 	 		3.14159f 			// PI
#define _3PI_2			4.712388f			//PI/3

float  zero = 0.0f;							//���Ƕ�

//��ֵ���ƺ���
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
 
// �ѽǶ�ֵ��һ���� [0, 2pi]
float Angle_limit(float angle)
{
    float a = fmod(angle, _2PI); // fmod()�������ڸ�������ȡ������
    return a >= 0.0f ? a : (a + _2PI);
}

// ��Ƕ� = ��е�Ƕ� * ������
float _electricalAngle(float shaft_angle)
{
    return Angle_limit(shaft_angle * polePairs - zero);
}

//����
void angle_init(void)
{
	setPhaseVoltage(0.0f,1.0f,0.0f);
	delay_ms(100);;
	zero = _electricalAngle(angle_get());
	float zero_angle = _electricalAngle(angle_get());
	printf("��ʼ�����\r\n");
	printf("���λ�Ƕ�:	%lf		%lf\r\n",zero,zero_angle);
}

float pwm_a=0,pwm_b=0,pwm_c=0;
void setpwm(float Ua,float Ub,float Uc)
{
	//����޷�
	Ua = limit(Ua,0.0f,volatge_high);
	Ub = limit(Ub,0.0f,volatge_high);
	Uc = limit(Uc,0.0f,volatge_high);
	
	//PWM�޷�
	pwm_a = limit(Ua / Udc , 0.0f , 1.0f);
	pwm_b = limit(Ub / Udc , 0.0f , 1.0f);
	pwm_c = limit(Uc / Udc , 0.0f , 1.0f);
	
	//PWMд��
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, pwm_a * 6000);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, pwm_b * 6000);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, pwm_c * 6000);
}

// FOC���ĺ���������Uq��Ud�͵�Ƕȣ������·PWM
float Ualpha=0.0f,Ubate=0.0f;
float Ua=0.0f,Ub=0.0f,Uc=0.0f;
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	
	//�����޷�
	Uq = limit(Uq,-Udc/2,Udc/2);
	
	//�Ƕȹ�һ��
	angle_el = Angle_limit(angle_el);
	
	//���Ƕ�
	angle_el = _electricalAngle(angle_el);
	
	//park��任
	Ualpha = -Uq * fast_sin(angle_el) + Ud * fast_cos(angle_el);
	Ubate  =  Uq * fast_cos(angle_el) + Ud * fast_sin(angle_el);
	
	//clarke��任
	Ua = Ualpha + Udc/2.0f;
	Ub = (sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	Uc = (-sqrt3 * Ubate - Ualpha)/2.0f + Udc/2.0f;
	
	setpwm(Ua,Ub,Uc);
}




