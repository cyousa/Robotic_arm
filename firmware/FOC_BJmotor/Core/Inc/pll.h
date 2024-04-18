#ifndef  __PLL_H
#define  __PLL_H
#include "main.h"
#include "arm_math.h"
#include "stdbool.h"
#define ABS(X) (X)>0?(X):(X)

#define current_meas_period 0.00006f

struct PosVel{
	float pos_in_one;	//����ֵ����
	float vel;				//�ٶ�
	float radio;			//��Ƕ�
	float phase_vel;	//����ٶ�
	float pos;				//����λ��
};

extern struct PosVel pos_vel;


void MeasurePosVel(uint16_t cnt,struct PosVel *pv);

inline int mod(const int dividend, const int divisor);
inline float wrap_pm(float x, float y) ;
inline float fmodf_pos(float x, float y) ;
inline float wrap_pm_pi(float x);
#endif