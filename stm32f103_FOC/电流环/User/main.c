#include "stm32f10x.h"                  // Device header
#include "pwm.h"
#include "AS5600.h"
#include "usart.h"
#include "I2C.h"
#include "delay.h"
#include "timer1.h"
#include "LED.h"
#include "FOC.h"
#include "AD.h"
#include "fast_sin.h"

float angle =  0.0;
int t=0,x=0;
float i_1=0,i_2=0,i_3=0;
uint16_t f_a=0,f_b=0;

int main(void)
{
	uart_init(115200);
	PWM_Init();
	delay_init();
	IIC_Init();
	timer1_Init();
	LED_Init();
	AD_Init();
	
	delay_ms(1000);
	beep_on(200);
	delay_ms(200);
	beep_on(200);
	delay_ms(200);

	//????????
	first_get(&f_a,&f_b);
	while (1)
	{
		i_1 = AD_Value[0] - f_a;
		i_2 = AD_Value[1] - f_b;
		i_3 = 0-i_1-i_2;

		//printf("%d,%d\r\n",AD_Value[0],AD_Value[1]);
		printf("%lf,%lf,%lf\r\n",i_1,i_2,i_3);
		//printf("%lf,%lf\r\n",Ialpha,Ibate);
		
		//printf("%lf\r\n",Id);
		//printf("%lf\r\n",Iq);
		//printf("%lf\r\n",Uq);
		//printf("%lf,%lf,%lf\r\n",Ua,Ub,Uc);
		t++;
		if(t==100)
		{
			LED1_Turn();
			t=0;
		}
	}
}

/*  10ms÷–∂œ */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		angle = angle_get();
		//i_1 = (float)(AD_Value[0] - f_a)/10;
		//i_2 = (float)(AD_Value[1] - f_b)/10;
		//i_3 = 0-i_1-i_2;
		setPhaseVoltage(i_1,i_2,i_3,angle);
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
