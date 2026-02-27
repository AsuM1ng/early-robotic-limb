#ifndef __DELAY_SYSTICK_H
#define __DELAY_SYSTICK_H

#include "stm32f4xx.h"

void Delay_Init(void);
void Delay_us(uint32_t n_us);
void Delay_ms(uint32_t n_ms);
	
#endif

