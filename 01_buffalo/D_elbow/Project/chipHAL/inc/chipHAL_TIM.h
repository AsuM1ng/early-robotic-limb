#ifndef _TIM_H_
#define _TIM_H_

#include "stm32f4xx.h"

/**
* System clock
*/
//#define CLK_SYS				168000000uL		///< System clock CLK_SYS=168M
//#define CLK_TIMAPB2			(CLK_SYS/1)		///< Clock of APB2 timer CLK_TIMAPB2=168M
//#define CLK_TIMAPB1			(CLK_SYS/2)		//< Clock of APB1 timer CLK_TIMAPB1=84M

/**
* PWM Timer.
*/
//#define PWM_TIM			TIM8									///< PWM Timer.
//#define PWM_TIM_PRSC		0										///< PWM TIM prescaler = 0.
//#define PWM_TIM_CNTCLK	( CLK_TIMAPB2/(PWM_TIM_PRSC+1) )		///< PWM_TIM_CK_CNT = 168M.
//#define PWM_TIM_CNTPERIOD	( PWM_TIM_CNTCLK/(2*PWM_FREQUENCY) )	///< PWM_TIM's ARR = PWM_TIM_CNTPERIOD.
//#define REP_RATE			1										///< Repetition counter = 1.

#define TIM_CANOPEN_SLAVE     TIM2
#define TIM_CANOPEN_MASTER    TIM4
#define TIM_PVT			      TIM3
#define TIM_GENERAL_TIMER	  TIM7
#define TIM_ABS_ENCODER       TIM6


#define TIM_PvtIRQHandler            TIM3_IRQHandler
#define TIM_CANopenSlaveIRQHandler   TIM2_IRQHandler
#define TIM_CANopenMasterIRQHandler  TIM4_IRQHandler
#define TIM_GeneralTimerIRQHandler   TIM7_IRQHandler

typedef struct{
    uint32_t u32MillisecondCount;   /*毫秒计数器*/
    uint32_t u32FiveMiliSecondCount;    /*5mS计数器*/
    uint32_t u32SecondCount;        /*秒计数器*/
    uint32_t u32BaseCycle;          /*基本计数周期，单位uS*/
    
    uint32_t u32TenMilisecond;      /*10mS计时门限*/
    uint32_t u32OneSecond;          /*1S计时门限*/
}SoftTimer_t;
extern SoftTimer_t         SoftTimer;

void TIM_pvtConfig(void);
void TIM_CANopenSlaveConfig(void);
void TIM_CANopenMasterConfig(void);
void TIM_GeneralTimerConfig(uint16_t period, uint16_t prescaler);
uint8_t SoftTimerInit(SoftTimer_t *SoftTime);
//void TIM_AbsEncoderConfig(void);

#endif

