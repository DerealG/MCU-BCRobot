#ifndef __FILTER_H
#define __FILTER_H

extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void First_Filter(float angle_m, float gyro_m);

#endif
