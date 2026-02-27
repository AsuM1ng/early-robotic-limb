#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "TIM.h"
#include "LED.h"

void TIM3_Init(u16 arr,  u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure; 
	NVIC_InitTypeDef  NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitStructure.TIM_Period = arr; //??????
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;	//?????
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //??????
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);// ??????? TIM3 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //?????? 3 ????
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //??? 3 ??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //????? 1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //???? 3 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
	NVIC_Init(&NVIC_InitStructure);// ????NVIC
	
	TIM_Cmd(TIM3,ENABLE); //?????? 3
}

u8 count = 0;

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET) //TIM3????
	{
		LED1 = !LED1;
		count++;
		if(count==10){
			LED0 = !LED0;
			count = 0;
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	//???????
}
