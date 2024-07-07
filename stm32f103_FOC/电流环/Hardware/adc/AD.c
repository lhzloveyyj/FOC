#include "stm32f10x.h"  // Device header
#include "AD.h" 

uint16_t AD_Value[2];
void AD_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 配置 ADC 通道及采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_1Cycles5);
		
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;								// 独立模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;							// 数据右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;			// 外部触发事件，定时器2
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;								// 禁用连续转换模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;									// 启用扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 2;											// 设置通道数量为 2
	ADC_Init(ADC1, &ADC_InitStructure);
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);									// 重置校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);			// 等待重置完成
	ADC_StartCalibration(ADC1);									// 启动校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);				// 等待校准完成
	
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);						// 启用外部触发

	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);						//软件触发使能
}

//上电读取偏置电压
void first_get(uint16_t *First_a,uint16_t *First_b)
{
	for(int i=0;i<16;i++)
	{
		*First_a += AD_Value[0];
		*First_b += AD_Value[1];
	}
	*First_a = *First_a>>4;
	*First_b = *First_b>>4;
}



