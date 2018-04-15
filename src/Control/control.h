#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define MAX_SPEED 4000
#define PI 3.14159265
#define MIDDLE 7

extern int Motor_Close;

void Get_Angle(u8 way);
int balance(float Angle, float Gyro);
int velocity(int encoder_left, int encoder_right);
void setResultPwm(int moto1, int moto2);
void SafeCheck(void);
#endif
