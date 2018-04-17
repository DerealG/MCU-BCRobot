#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define MPU6050_INT PAin(12)
#define MAX_SPEED 4000
#define PI 3.14159265

extern int Motor_Close;

void Get_Angle(u8 way);
int balance(float Angle, float Gyro);
int velocity(int encoder_left, int encoder_right);
int turn(int encoder_left, int encoder_right, float gyro);
void setResultPwm(int moto1, int moto2);
void SafeCheck(void);

void DataSend(void);

#endif
