#ifndef __PID_H
#define __PID_H

float angle_pid(float now_angle,float tar_angle);
void  angle_clear(void);

float speed_pid(float speed,float tar);
void  speedd_clear(void);

#endif
