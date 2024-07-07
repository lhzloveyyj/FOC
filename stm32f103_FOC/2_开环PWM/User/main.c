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

float angle =  0.0;
int t=0;

int main(void)
{
	uart_init(115200);
	PWM_Init();
	delay_init();
	IIC_Init();
	timer1_Init();
	LED_Init();
	
	int t=0;
	
	while (1)
	{
		//printf("%lf\n", angle);
		//printf("%lf\n", fast_sin(3.1415f/3));
		printf("%lf,%lf,%lf\r\n",Ua,Ub,Uc);
	}
}

/*  10ms÷–∂œ */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		angle = angle_get();
		setPhaseVoltage(3,0,angle);
		
		t++;
		if(t==100)
		{
			LED1_Turn();
			t=0;
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
