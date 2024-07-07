#include "stm32f10x.h"                  // Device header
#include "LED.h"  
#include "delay.h" 

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void LED1_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

void LED1_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

void LED1_Turn(void)
{
	GPIOB->ODR ^= GPIO_Pin_0;
}

void beep_on(int t)
{
	for(int i=0;i<t;i++)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_3); 
		delay_us(500);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3); 
		delay_us(50);
	}
	GPIO_ResetBits(GPIOA,GPIO_Pin_3); 
}

