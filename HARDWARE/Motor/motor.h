#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

#define Default_Max_Speed 15  //Ä¬ÈÏÊÇ15km/h
void Motor_Init(void);
void Motor_SetSpeedR(int speed);
void Motor_SetSpeedL(int speed);
void Motor_SetSpeed(int speed);
void Motor_Brake(float ratio);
void Motor_Go(int need_speed);
void Motor_Stop(u8 deep);
extern int lastspeed;
#endif
