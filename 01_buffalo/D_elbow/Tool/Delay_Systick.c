/* 采用systick作为精确延时的定时器，这里和硬件有耦合. */
/* 这里的基于systick的延时函数功能均未用到Systick中断！ */

#include "Delay_Systick.h"

#define STM32F4_HCLK	168		//STM32F4的HCLK = 168MHz

static uint16_t fac_us = 0;		//systick_clk=21MHz -->1MHz的分频系数
static uint16_t fac_ms = 0;		//systick_clk=21MHz -->1000Hz的分频系数

void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);		//设置systick的时钟源: HCLK/8 = 168M/8=21MHz
	fac_us = STM32F4_HCLK / 8;									//systick_clk=21MHz -->1MHz的分频系数为21
	fac_ms = fac_us * 1000;										//systick_clk=21MHz -->1KHz的分频系数为21000
}

/**********************************************************
* @FuncName: Delay_us
* @Param: n_us -- 延时多少n个us
* @Return: None
* @Brief: 考虑到systick是24位定时器，LOAD和VAL的最大值为2^24-1
*			=16777215;fac_us = 21，因此最大延时n_us(max)=16777216
*			/21 = 798,915us.
* @addtion: n_us(max) = 798,915us, n_us>=1.
**********************************************************/
void Delay_us(uint32_t n_us)
{
	uint32_t tmp = 0;
	SysTick->LOAD = n_us * fac_us - 1;				//计时时间加载
	SysTick->VAL = 0;								//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;		//开始倒数计时
	
	do
	{
		tmp = SysTick->CTRL;						//读Systcik控制状态寄存器
	}while( (tmp&0x01) && !(tmp&(1<<16)) );			//等待时间到达。(如果在systick使能的情况下counterflag未置1，则时间还未到达，继续等待)
													//如果在systick使能的情况下counterflag=1，则时间到达，跳出循环(等待停止).
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		//关闭定时器
	SysTick->VAL = 0;								//清空定时器
}

/**********************************************************
* @FuncName: Delay_xms
* @Param: n_ms -- 延时多少n个ms
* @Return: None
* @Brief: 考虑到systick是24位定时器，LOAD和VAL的最大值为2^24-1
*			=16777215;fac_ms = 21000，因此最大延时n_ms(max)
*			=16777216/21000 = 798ms.
* @addtion: n_ms(max) = 798 ms, n_ms>=1
**********************************************************/
static void Delay_xms(uint32_t n_ms)
{
	uint32_t tmp = 0;
	SysTick->LOAD = n_ms * fac_ms - 1;				//计时时间加载
	SysTick->VAL = 0;								//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;		//开始倒数计时
	
	do
	{
		tmp = SysTick->CTRL;						//读Systcik控制状态寄存器
	}while( (tmp&0x01) && !(tmp&(1<<16)) );			//等待时间到达。(如果在systick使能的情况下counterflag未置1，则时间还未到达，继续等待)
													//如果在systick使能的情况下counterflag=1，则时间到达，跳出循环(等待停止).
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;		//关闭定时器
	SysTick->VAL = 0;								//清空定时器
}

/**********************************************************
* @FuncName: Delay_ms
* @Param: n_ms -- 延时多少n个ms
* @Return: None
* @Brief: 考虑到定时长短，这里将定时器定时长度划分为两部分：
*			整数部分 + 小数部分.(以600ms为一个进制)
* @addtion: 
**********************************************************/
void Delay_ms(uint32_t n_ms)
{
	uint16_t repeat = n_ms / 600;					//m个600ms
	uint16_t remain = n_ms % 600;					//外加剩余 (n_ms- m*600).
	
	while(repeat--)									//延时m个600ms
	{
		Delay_xms(600);
	}
	if(remain)										//由于Delay_xms(nms)函数的传入参数nms必须大于0，因此这里要做判断.
	{
		Delay_xms(remain);	
	}
}
