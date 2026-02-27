#include "timer7_action_timing.h"

u8 target_conter_num = 0;
u8 conter_times = 0;
u8 timing_end_flag = 0;

TIM_TimeBaseInitTypeDef action_timer;

TIMEVAL last_counter_val_7 = 0;
TIMEVAL elapsed_time_7 = 0;

// Initializes the timer, turn on the interrupt and put the interrupt time to zero
void TIM3_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Compute the prescaler value */
	uint16_t PrescalerValue =8400-1; //42MHz,4200/42MHz = 0.1ms

	/* Time base configuration */
	action_timer.TIM_Period = 1000 - 1;
	action_timer.TIM_Prescaler = PrescalerValue;
	action_timer.TIM_ClockDivision = 0;
	action_timer.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &action_timer);
	
	TIM_ClearITPendingBit(TIM3, TIM_SR_UIF);
 
	/* TIM2 enable counter */  //Æô¶¯TIM2
	TIM_Cmd(TIM3, ENABLE);

	/* Preset counter for a safe start */
	TIM_SetCounter(TIM3, 1);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

//void task_time_set(uint32_t time_100ms)
//{
//	conter_num = time_100ms;
//	TIM_Cmd(TIM3, ENABLE);
//	TIM_SetCounter(TIM3, 1);
//}

void task_time_set(uint32_t times_100ms)
{
	target_conter_num = times_100ms;
	/* TIM2 enable counter */  //Æô¶¯TIM2
	TIM_Cmd(TIM3, ENABLE);
	/* Preset counter for a safe start */
	TIM_SetCounter(TIM3, 1);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void task_stop_timing(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
}

void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		conter_times++;
		if(conter_times >= target_conter_num - 1)
		{
			sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
			conter_times = 0;
			target_conter_num = 0;
			timing_end_flag = 1;
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
//			if(MS.Control == 0x010F)
//				TS.waiting = 1;
		}
  }
}


