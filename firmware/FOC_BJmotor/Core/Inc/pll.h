#ifndef  __PLL_H
#define  __PLL_H
#include "main.h"
#include "arm_math.h"
#include "stdbool.h"
#define ABS(X) (X)>0?(X):(X)

#define current_meas_period 0.00006f

struct PosVel{
	float pos_in_one;	//绝对值参数
	float vel;				//速度
	float radio;			//电角度
	float phase_vel;	//电角速度
	float pos;				//增量位置
};

extern struct PosVel pos_vel;


void MeasurePosVel(uint16_t cnt,struct PosVel *pv);

inline int mod(const int dividend, const int divisor);
inline float wrap_pm(float x, float y) ;
inline float fmodf_pos(float x, float y) ;
inline float wrap_pm_pi(float x);
#endif