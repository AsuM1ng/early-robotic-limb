#include "canfestival_timer.h"

// 下面函数是用于配制canfestival的唯一硬件定时器TIM14,
// 所以这个函数要在主函数中进行调用,然后传入的形参也是TIM14
void TIMConfig(TIM_TypeDef* TIMx, uint16_t TIM_Period, uint16_t TIM_Prescaler)
{
	TIM_TimeBaseInitTypeDef	 TIM_BaseInitStructure;

	//TIM_RepetitionCounter(TIM1_RCR)=0，每次向上溢出都产生更新事件
	TIM_DeInit(TIMx);//初始化 TIM1寄存器
	/*分频和周期计算公式：
	Prescaler = (TIMxCLK / TIMx counter clock) - 1;
	Period = (TIMx counter clock / TIM3 output clock) - 1
	TIMx counter clock 为你所需要的 TXM 的定时器时钟*/
	TIM_BaseInitStructure.TIM_Period = TIM_Period-1;//查数据手册可知，TIM1与 TIM8为16位自动装载
	/*在 system_stm32f4xx.c 中设置的 APB2 Prescaler = 2 ,可知
	*APB1时钟为168M/2*2,因为如果 APB1分频不为1，则定时时钟 x2*/
	TIM_BaseInitStructure.TIM_Prescaler = TIM_Prescaler-1;//分频为10K
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode =  TIM_CounterMode_Up;//向上计数
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIMx, &TIM_BaseInitStructure);

	//清中断，以免一启用中断后立即产生中断
	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	//使能TIM1中断源
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	//TIM1总开关：开启
	TIM_Cmd(TIMx, ENABLE);//使能 TIM1定时器
}



void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;    	    //更新事件
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;    //抢占优先级0  原始0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          //响应优先级1   原始1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             //允许中断
    NVIC_Init(&NVIC_InitStructure);                             //写入设置

}


// canfestival 定时器 timer14定时器
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM14,TIM_IT_Update)!=RESET)
    {
        TIM_ClearITPendingBit(TIM14, TIM_IT_Update); //清中断
        TimeDispatch(); // canfestival的库
    }
}



void setTimer(TIMEVAL value)
{
	TIM14->ARR = TIM14->CNT + value;
}


TIMEVAL getElapsedTime(void)
{
	return TIM14->CNT;
}