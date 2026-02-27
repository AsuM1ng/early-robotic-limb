#ifndef __TIMER7_ACTION_TIMING_H
#define __TIMER7_ACTION_TIMING_H

#include "sys.h"
#include "main.h"

void task_time_set(uint32_t times_100ms);
void task_stop_timing(void);
void TIM3_NVIC_Configuration(void);
//void Action_setTimer(TIMEVAL value);
//TIMEVAL Action_getElapsedTime(void);

extern u8 timing_end_flag;
extern u8 conter_times;
extern u8 target_conter_num;

#endif 

