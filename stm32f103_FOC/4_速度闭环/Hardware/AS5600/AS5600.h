#ifndef __AS5600_H
#define __AS5600_H

u8 AS5600_IIC_Read_OneByte(u8 deviceaddr,u8 readaddr);
float angle_get(void);
float angle_get_all(float angle);
float get_speed(float angle);
float LowPass_Filter(float x);

#endif
