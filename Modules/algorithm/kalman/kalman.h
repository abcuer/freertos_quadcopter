#ifndef _KALMAN_H
#define _KALMAN_H

#include "math.h"
#include <stdint.h>

struct _1_ekf_filter
{
	float LastP;
	float	Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
};

//void ekf_1(struct EKF *ekf,void *input);  //һά������
void kalman_1(struct _1_ekf_filter *ekf,float input);  //һά������
float kalman_2_Update(float InputAngle,float InputGyro,float dt);

#endif


