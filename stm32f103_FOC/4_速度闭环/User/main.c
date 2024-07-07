#include "stm32f10x.h"                  // Device header
#include "pwm.h"
#include "AS5600.h"
#include "usart.h"
#include "I2C.h"
#include "delay.h"
#include "timer1.h"
#include "LED.h"
#include "FOC.h"
#include "fast_sin.h"
#include "pid.h"

float angle =  0.0;
float all_angle =  0.0;
float speed =  0.0;
float speed_out = 0.0;
float local_out = 0.0;
int t=0;

int main(void)
{
	delay_init();
	uart_init(115200);
	PWM_Init();
	IIC_Init();
	timer1_Init();
	LED_Init();
	
	angle_init(angle);
	
	while (1)
	{
		//printf("%lf\n", angle);
		//printf("%lf		%lf\n", all_angle,local_out);
		//printf("%lf\n",speed_out);
		printf("%lf,%lf,%lf\n",Ua,Ub,Uc);
	}
}

/*  10ms÷–∂œ */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		angle = angle_get();
		all_angle = angle_get_all(angle);
		speed=get_speed(all_angle);
		speed = LowPass_Filter(speed);
		speed_out = speed_pid(speed,1);
		setPhaseVoltage(speed_out,0,angle);
		t++;
		if(t==100)
		{
			LED1_Turn();
			t=0;
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
