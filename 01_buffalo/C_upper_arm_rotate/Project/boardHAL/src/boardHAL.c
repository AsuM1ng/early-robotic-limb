/*****************************************************************
 * COPYRIGHT NOTICE
 * Copyright (C) 2018, Buffalo Robot Tech.co, Ltd
 * All rights reserved.
 *
 * @file 	BHAL.c
 * @brief   This file contains all the functions prototypes for the GPIO firmware
 *          library. Some of stm32 lib are used there.
 *
 * @author  chenhuachuan
 * @email   Chenhc@buffalo-robot.com
 * @date    07.25, 2018
 * @version Software v1.0.0.0
******************************************************************/

#include "chipHAL_buzzer.h"
#include "chipHAL_USART.h"
#include "chipHAL_GPIO.h"
#include "chipHAL_TIM.h"
#include "chipHAL_IWDG.h"

#include <stdint.h>
#include "stm32f4xx.h"
#include "boardHAL.h"
#include "BreakControl.h"


__asm void WFI_SET(void)
{
	WFI;		  
}

//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}

//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}

//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}


void boardHAL_init(void)
{
    __disable_irq();        //< Disable the all IRQs before initializing.
	
    /*延时函数初始化*/
    chipHAL_Delay_init(168);

    GPIO_LedInit();
    
    GPIO_CanIDInit();

#if ('B' == JOINTBOARD_TYPE)
    GPIO_BreakInit();
#endif

    /*绝对位置传感器初始化*/
    AbsAngleSensor_Init();
    delay_ms(500);
    
    /*初始化控制节拍定时器*/
    /*10 * (1/(84000000/84)) = 10uS*/
    TIM_GeneralTimerConfig(1000, 84);
    
//	IWDG_Init(IWDG_Prescaler_64, 500); //prescaler_64, reload value 500, overflow time 1s	

    __enable_irq();              //< After all peripherals have been initialized, IRQs are enabled.	
}


