#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void pwm_a_write(uint16_t Compare);
void pwm_b_write(uint16_t Compare);
void pwm_c_write(uint16_t Compare);
void Start_Motor(void);
void Stop_Motor(void);
void Svpwm_Out(void);

#define PWM_ARR 1440-1
#define PWM_PSC 0
#define REP_counter 0
#define PWM_CCR 0
#define PWM_FREQ ((u16)12500)

#endif
