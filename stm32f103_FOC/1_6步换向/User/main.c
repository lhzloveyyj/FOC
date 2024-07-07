#include "stm32f10x.h"                  // Device header
#include "pwm.h"
#include "AS5600.h"
#include "usart.h"
#include "I2C.h"
#include "delay.h"
#include "timer1.h"
#include "LED.h"

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
	
	while (1)
	{
		pwm_a_write(1000);
		pwm_b_write(1000);
		pwm_c_write(1000);
//		delay_ms(100);
//		
//		pwm_a_write(2800);
//		pwm_b_write(0);
//		pwm_c_write(0);
//		delay_ms(100);
//		
//		pwm_a_write(2800);
//		pwm_b_write(2800);
//		pwm_c_write(0);
//		delay_ms(100);
//		
//		pwm_a_write(0);
//		pwm_b_write(2800);
//		pwm_c_write(0);
//		delay_ms(100);
//		
//		pwm_a_write(0);
//		pwm_b_write(2800);
//		pwm_c_write(2800);
//		delay_ms(100);
//		
//		pwm_a_write(0);
//		pwm_b_write(0);
//		pwm_c_write(2800);
//		delay_ms(100);
		
	}
}

/*  10ms÷–∂œ */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		angle = angle_get();
		t++;
		if(t==100)
		{
			LED1_Turn();
			t=0;
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
