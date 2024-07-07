#ifndef __PID_H
#define __PID_H

float Iq_pid(float speed,float tar);
void  speed_clear(void);

float Id_pid(float speed,float tar);
void  speed_2_clear(void);

float LowPass_Filter(float x,int flag);

#endif
