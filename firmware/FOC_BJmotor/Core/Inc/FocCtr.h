#ifndef __FOCCTR_H
#define __FOCCTR_H
#include "main.h"
#include "pll.h"

#define  Vbus  16.0f;


extern uint8_t isready;
extern float Target_angel;
extern float ud,uq;
void SVPWM();
void sector_judg();
void Park_contrary_change();
void Position_loop();
void speed_loop();
#endif