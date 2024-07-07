#include "AS5600.h"  
#include "I2C.h"  
#include "math.h"  

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

//角度累计计算
float val=0.0f,last_val=0.0f,err=0.0f,circle = 0.0f;
float angle_get_all(float angle)
{
	val = angle;
	err = val - last_val;
	if( fabs(err) > (0.8f*6.2831853f) )
		circle += (err > 0) ? -1 : 1;
	last_val = val;
	return (float)circle * 6.28318530718f + last_val;
}

//速度计算
float Speed = 0.0f;
float now=0.0f,pre=0.0f;
float get_speed(float angle)
{
	now = angle;
	Speed = (now - pre) / 1.0f;
	pre = now;
	return Speed*100;
}

//低通滤波器
float y_prev;
float LowPass_Filter(float x)
{
	float y = 0.9f*y_prev + 0.1f*x;
	
	y_prev=y;
	
	return y;
}


