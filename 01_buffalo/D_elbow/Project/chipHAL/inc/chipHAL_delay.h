#ifndef __DELAY_H
#define __DELAY_H 			   
//#include <sys.h>
#include "stm32f4xx.h" 
#include <stdint.h>


void chipHAL_Delay_init(uint8_t SYSCLK);

void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);

#endif





























