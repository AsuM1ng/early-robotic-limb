#ifndef __TIMER7_ACTION_TIMING_H
#define __TIMER7_ACTION_TIMING_H

#include "sys.h"
#include "main.h"

void action_time_set(uint32_t time_100ms);
void TIM3_NVIC_Configuration(void);
//void Action_setTimer(TIMEVAL value);
//TIMEVAL Action_getElapsedTime(void);

extern u8 action_finish_flag;

#endif 

