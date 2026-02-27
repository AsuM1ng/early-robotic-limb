#include "timer7_action_timing.h"

u8 action_finish_flag = 0;

TIMEVAL last_counter_val_7 = 0;
TIMEVAL elapsed_time_7 = 0;

// Initializes the timer, turn on the interrupt and put the interrupt time to zero
void TIM3_NVIC_Configuration(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void action_time_set(uint32_t time_100ms) 
{
	uint32_t prescaler = 4200 - 1; // 
	uint32_t autoreload = time_100ms*1000 - 1; //
    
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructure.TIM_Period = autoreload;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
//	TIM_ClearITPendingBit(TIM3, TIM_SR_UIF);
//	
//	TIM_SetCounter(TIM3, 1);
//	
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//    
//	TIM_Cmd(TIM3, ENABLE);
}

//void action_time_set(u8 num)
//{
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
//	TIM_Cmd(TIM7, DISABLE);
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_TimeBaseStructure.TIM_Period = (1000 * num) - 1;		//100对应0.1s，num是次数，10次为1s，5次0.5秒，最高可计算858993秒
//	TIM_TimeBaseStructure.TIM_Prescaler = 16800-1;				//42MHz/42000 = 10k, 计时一次0.1ms
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
//	
//	TIM_ClearITPendingBit(TIM7, TIM_SR_UIF);
//	
//	TIM_Cmd(TIM7, ENABLE);

//	/* Preset counter for a safe start */
//	TIM_SetCounter(TIM7, 1);

//	/* TIM Interrupts enable */
//	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
//}


void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM_Cmd(TIM3, DISABLE);
		TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
		action_finish_flag = 1;
		TimeDispatch();
    }
	
}


