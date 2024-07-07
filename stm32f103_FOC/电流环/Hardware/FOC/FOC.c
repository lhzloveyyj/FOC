#include "stm32f10x.h"                  // Device header
#include "foc.h"
#include "math.h"
#include "pwm.h"
#include "fast_sin.h"
#include "usart.h"
#include "PID.h"

#define volatge_high 	12.0f				//��ѹ����ֵ
#define Udc 			12.0f				//ĸ�ߵ�ѹ
#define sqrt3			1.732				//����3
#define polePairs 	 	11 					// ����ļ�����
#define _2PI 	 		6.2831853 			// 2PI
#define _PI 	 		3.1415927 			// PI
#define _1_SQRT3 0.57735026919				//��������֮һ

//��ֵ���ƺ���
float limit(float in_vo,float low,float high)
{
	if(in_vo>=high)
		in_vo=high;
	else if(in_vo<=low)
		in_vo=high;
	else
		in_vo=in_vo;
	return in_vo;
}
 
// ��Ƕ� = ��е�Ƕ� * ������
float _electricalAngle(float shaft_angle)
{
    return (shaft_angle * polePairs);
}

// �ѽǶ�ֵ��һ���� [0, 2pi]
float Angle_limit(float angle)
{
    float a = fmod(angle, _2PI); // fmod()�������ڸ�������ȡ������
    return a >= 0.0f ? a : (a + _2PI);
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
	pwm_a_write(pwm_a * 1440);
	pwm_b_write(pwm_b * 1440);
	pwm_c_write(pwm_c * 1440);
	//printf("%lf,%lf,%lf\n",pwm_a * 2600,pwm_b * 2600,pwm_c * 2600);
}

// FOC���ĺ���������Uq��Ud�͵�Ƕȣ������·PWM
float Ualpha=0.0f,Ubate=0.0f;
float Ialpha=0.0f,Ibate=0.0f;

float Ua=0.0f,Ub=0.0f,Uc=0.0f;
float Uq=0.0f,Ud=0.0f;
float Iq=0.0f,Id=0.0f;
void setPhaseVoltage(float Ia,float Ib,float Ic,float angle_el)
{
	//�Ƕȹ�һ��
	angle_el = Angle_limit(angle_el);
	
	//���Ƕ�
	angle_el = _electricalAngle(angle_el);

	//clarke�任
	Ialpha = Ia;
	Ibate  = Ia*_1_SQRT3 + Ib*2*_1_SQRT3;
	
	//park�任
	Id = Ialpha * fast_cos(angle_el) + Ibate * fast_sin(angle_el);
	Iq = -Ialpha * fast_sin(angle_el) + Ibate * fast_cos(angle_el);

	Iq = LowPass_Filter(Iq,0);
	Id = LowPass_Filter(Id,1);

	//������PI
	Uq = Iq_pid(Iq,0.1);
	Ud = Id_pid(Id,0);

	//�����޷�
	//Uq = limit(Uq,-Udc/2,Udc/2);
	Uq=3;
	Ud=0;
	//park��任
	Ualpha = -Uq * fast_sin(angle_el) + Ud * fast_cos(angle_el);
	Ubate  =  Uq * fast_cos(angle_el) + Ud * fast_sin(angle_el);
	
	//clarke��任
	Ua = Ualpha + Udc/2;
	Ub = (sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	Uc = (-sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	
	setpwm(Ua,Ub,Uc);
}




