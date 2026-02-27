#ifndef BREAKCONTROL_H_INCLUDED
#define BREAKCONTROL_H_INCLUDED

#include <stdint.h>
#include "stm32f4xx_gpio.h"	
#include "Board_type_config.h"



/**
* Break GPIO interface
*/
#if ('B' == JOINTBOARD_TYPE)

typedef enum{
    Break       = 0x00,
    Release     = 0x01
}BreakControl_en;

typedef enum{
    Breaking    = 0x00,
    Releaseing  = 0xFF
}BreakStatus;


#define BREAK_CONTROL_GPIO_CLOCK     RCC_AHB1Periph_GPIOA
#define BREAK_CONTROL_GPIO_PORT      GPIOA
#define BREAK_CONTROL_GPIO_PIN       GPIO_Pin_1

#define BREAK_FEEDBACK_GPIO_CLOCK    RCC_AHB1Periph_GPIOA
#define BREAK_FEEDBACK_GPIO_PORT     GPIOA
#define BREAK_FEEDBACK_GPIO_PIN      GPIO_Pin_2

/*刹车GPIO初始化*/
void GPIO_BreakInit(void);

/*获取刹车状态*/
BreakStatus GetBreakStatus(void);

/*刹车控制函数*/
void BreakControlProcess(BreakControl_en breakCtrl);

#endif


#endif // BREAKCONTROL_H_INCLUDED

