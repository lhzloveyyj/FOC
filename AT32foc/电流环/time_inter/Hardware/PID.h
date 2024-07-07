#ifndef __PID_H
#define __PID_H

void  angle_clear(void);
float LowPass_Filter(float x,int flag);
float Id_pid(float speed,float tar);
float Iq_pid(float speed,float tar);


#endif
