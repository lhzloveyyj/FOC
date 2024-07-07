#include "stm32f10x.h"                  // Device header
#include "I2C.h"
#include "delay.h"
#include "sys.h"

 /**
 * @brief   		I2C��ʼ��
 */
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;  				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 					
}

//I2C�����ʱ
void delay_s(u32 i)
{
	while(i--);
}

 /**
 * @brief   		����IIC��ʼ�ź�
 */
void IIC_Start(void)
{
	SDA_OUT();     												
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_s(4);
 	IIC_SDA=0;														
	delay_s(4);
	IIC_SCL=0;																											
}	  

/**
 * @brief   		����IICֹͣ�ź�
 */
void IIC_Stop(void)
{
	SDA_OUT();																												
	IIC_SCL=0;
	IIC_SDA=0;																												
	delay_s(4);
	IIC_SCL=1; 
	IIC_SDA=1;																												
	delay_s(4);
}

/**
 * @brief    		�ȴ�Ӧ���źŵ���
 * @return			����ֵ��1������Ӧ��ʧ��     0������Ӧ��ɹ�
 */
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      																				
	IIC_SDA=1;
	delay_s(1);
	IIC_SCL=1;
	delay_s(1);	
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;																					
	return 0;  
} 

/**
 * @brief    		����ACKӦ��
 */
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_s(2);
	IIC_SCL=1;
	delay_s(2);
	IIC_SCL=0;
}

/**
 * @brief    		������ACKӦ��
 */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_s(2);
	IIC_SCL=1;
	delay_s(2);
	IIC_SCL=0;
}					 				     

/**
 * @brief    		IIC����һ���ֽ�
 * @param[in]   txd : ��Ҫ���͵�����
 * @date��      ���شӻ�����Ӧ��  1����Ӧ��  0����Ӧ��			
 */
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;															
    for(t=0;t<8;t++)
    {              
																								
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_s(2);
		IIC_SCL=1;
		delay_s(2);
		IIC_SCL=0;	
		delay_s(2);
    }	 
} 	    

/**
 * @brief    		IIC��һ���ֽ�
 * @param[in]   ack    ack=1ʱ������ACK��ack=0������nACK   
 */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();																																														//SDA����Ϊ����
  for(i=0;i<8;i++ )
	{
    IIC_SCL=0; 
		delay_s(2);
		IIC_SCL=1;
    receive<<=1;
    if(READ_SDA)receive++;   
		delay_s(1);
  }					 
    if (!ack)
        IIC_NAck();																
    else
        IIC_Ack(); 																		
    return receive;
}

