
#ifndef __CHAL_IWDG_H__
#define __CHAL_IWDG_H__

#include "stdint.h"
#include "stm32f4xx_iwdg.h"

void IWDG_Init(uint8_t prer,uint16_t rlr);//IWDG³õÊ¼»¯
void IWDG_Feed(void);  //Î¹¹·º¯Êý

#endif
