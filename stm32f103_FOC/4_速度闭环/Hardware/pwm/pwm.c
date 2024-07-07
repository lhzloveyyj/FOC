#include "stm32f10x.h"                  // Device header
#include "pwm.h" 

/*    占空比=CCR/ARR     */
/*    PWM频率输出= 72M /(PSC+1）/ARR+1)/2 = 72M /(0+1）/2880)/2 = 12.5k HZ  */ 
/*    PWM周期 = 1/12.5k =  80 us */ 


void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;					//数字滤波器采样频率
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;	//中央对齐模式
	TIM_TimeBaseInitStructure.TIM_Period = PWM_ARR;								//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_PSC;							//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = REP_counter;				//预分频系数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;							//脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;					//高电平有效
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_CCR;									//CCR
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

void pwm_a_write(uint16_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);
}

void pwm_b_write(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);
}

void pwm_c_write(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);
}

void Start_Motor(void)
{
  /*PWM寄存器占空比清零*/
   TIM2->CCR1=0x00;
   TIM2->CCR2=0x00;
   TIM2->CCR3=0x00;
   //使能PWM输出通道OC1/OC1N/OC2/OC2N/OC3/OC3N
   TIM1->CCER|=0x5555;	
}

void Stop_Motor(void)
{
  /*PWM寄存器占空比清零*/
   TIM2->CCR1=PWM_ARR;
   TIM2->CCR2=PWM_ARR;
   TIM2->CCR3=PWM_ARR;
   //不使能PWM输出通道OC1/OC1N/OC2/OC2N/OC3/OC3N
   TIM2->CCER&=0xAAAA;	
}
