#include "I2C.h"

//I2C软件延时
void delay_s(u32 i)
{
	while(i--);
}

 /**
 * @brief   		产生IIC起始信号
 */
void IIC_Start(void)
{
	SDA_OUT();     												
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_s(20);
 	IIC_SDA=0;														
	delay_s(20);
	IIC_SCL=0;																											
}	  

/**
 * @brief   		产生IIC停止信号
 */
void IIC_Stop(void)
{
	SDA_OUT();																												
	IIC_SCL=0;
	IIC_SDA=0;																												
	delay_s(20);
	IIC_SCL=1; 
	IIC_SDA=1;																												
	delay_s(20);
}

/**
 * @brief    		等待应答信号到来
 * @return			返回值：1，接收应答失败     0，接收应答成功
 */
u8 IIC_Wait_Ack(void)
{
	u16 ucErrTime=0;
	SDA_IN();      																				
	IIC_SDA=1;
	delay_s(20);
	IIC_SCL=1;
	delay_s(30);	
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>500)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;																					
	return 0;  
} 

/**
 * @brief    		产生ACK应答
 */
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_s(20);
	IIC_SCL=1;
	delay_s(20);
	IIC_SCL=0;
}

/**
 * @brief    		不产生ACK应答
 */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_s(20);
	IIC_SCL=1;
	delay_s(20);
	IIC_SCL=0;
}					 				     

/**
 * @brief    		IIC发送一个字节
 * @param[in]   txd : 需要发送的数据
 * @date：      返回从机有无应答  1，有应答  0，无应答			
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
		delay_s(20);
		IIC_SCL=1;
		delay_s(20);
		IIC_SCL=0;	
		delay_s(20);
    }	 
} 	    

/**
 * @brief    		IIC读一个字节
 * @param[in]   ack    ack=1时，发送ACK，ack=0，发送nACK   
 */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();																																														//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    IIC_SCL=0; 
		delay_s(20);
		IIC_SCL=1;
    receive<<=1;
    if(READ_SDA)receive++;   
		delay_s(20);
  }					 
    if (!ack)
        IIC_NAck();																
    else
        IIC_Ack(); 																		
    return receive;
}

