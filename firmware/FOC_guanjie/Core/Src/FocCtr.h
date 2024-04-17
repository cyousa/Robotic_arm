#ifndef __FOCCTR_H
#define __FOCCTR_H
#include "main.h"
#include "pll.h"

#define AS_CS_H HAL_GPIO_WritePin(GPIOB, AS_CS_Pin, 1);
#define AS_CS_L HAL_GPIO_WritePin(GPIOB, AS_CS_Pin, 0);
#define squrt3 1.73205f
#define PWM_Max 4200
#define K1 0.0003834952f
#define Vbus   16


extern float ta,tb,tc;
extern struct PosVel pos_vel; 
extern float Uq,Ud; 
extern PID Iq_current,Id_current;
extern PID Speed_ctl;
extern float Iq_Target;

float Iq_current_loop();
float Id_current_loop();
float  speed_loop();
void sector_judg();
void Park_change_Contrary();
void Klark_change();
#endif
