#include "AS5600.h"  
#include "I2C.h"  

//原始数据获取0~4096
u8 AS5600_IIC_Read_OneByte(u8 deviceaddr,u8 readaddr)
{
  u8 temp;
  IIC_Start();
  IIC_Send_Byte(deviceaddr&0xfe);
  IIC_Wait_Ack();
  IIC_Send_Byte(readaddr);
  IIC_Wait_Ack();

  IIC_Start();
  IIC_Send_Byte(deviceaddr|0x01);
  IIC_Wait_Ack();
  temp=IIC_Read_Byte(0);
  IIC_Stop();
  return temp;
}

//角度转换为0~2PI
int value = 0; 
float Angle = 0.0;
float _2PI=6.283185f;
float angle_get(void)
{
	value =  AS5600_IIC_Read_OneByte((0x36<<1),0x0e); 
	value <<= 8;
	value |= AS5600_IIC_Read_OneByte((0x36<<1),0x0f); 
	Angle = value / 4096.0f * _2PI;
	if(Angle >= _2PI)
		Angle = _2PI;
	return Angle;
}


