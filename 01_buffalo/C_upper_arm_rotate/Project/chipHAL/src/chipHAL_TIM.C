
#include "chipHAL_TIM.h"



#define TIM_PVT_CLK 	    RCC_APB1Periph_TIM3
#define TIM_PVT_IRQn 	    TIM3_IRQn

#define TIM_CANOPEN_SLAVE_CLK       RCC_APB1Periph_TIM2
#define TIM_CANOPEN_SLAVE_IRQn 	    TIM2_IRQn

#define TIM_CANOPEN_MASTER_CLK      RCC_APB1Periph_TIM4
#define TIM_CANOPEN_MASTER_IRQn     TIM4_IRQn

// define a general timer
#define TIM_GENERAL_TIMER_CLK       RCC_APB1Periph_TIM7
#define TIM_GENERAL_TIMER_IRQn 	    TIM7_IRQn

// ABS encoder clock timer
#define TIM_ABS_ENCODER_CLK       RCC_APB1Periph_TIM6
#define TIM_ABS_ENCODER_IRQn 	  TIM6_DAC_IRQn


SoftTimer_t         SoftTimer;


// config TIM3 for PVT generator
void TIM_pvtConfig(void)
{
    NVIC_InitTypeDef         NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(TIM_PVT_CLK, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 8400-1; /* CNT递增的单位为100μs */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//period = 100-1 = 99, so the interupt interval is 100μs * 100 =10ms	
    TIM_TimeBaseStructure.TIM_Period = 99;  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM_PVT, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM_PVT, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM_PVT, ENABLE);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_InitStructure.NVIC_IRQChannel = TIM_PVT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// config TIM2 for CANopenSlave, TIM2_IRQn to deal with the events of CANopenSlave
void TIM_CANopenSlaveConfig(void)
{
	NVIC_InitTypeDef         NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(TIM_CANOPEN_SLAVE_CLK, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler =84-1; /* CNT递增的单位为1微秒 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//period = 0-1 = 0xFFFFFFFF, so the interupt interval is 1μs	
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM_CANOPEN_SLAVE, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM_CANOPEN_SLAVE, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM_CANOPEN_SLAVE, ENABLE);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_InitStructure.NVIC_IRQChannel = TIM_CANOPEN_SLAVE_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// config TIM4 for CANopenMaster, TIM4_IRQn to deal with the events of CANopenMaster
void TIM_CANopenMasterConfig(void)
{
	NVIC_InitTypeDef         NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(TIM_CANOPEN_MASTER_CLK, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler =84-1; /* CNT递增的单位为1微秒 */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM_CANOPEN_MASTER, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM_CANOPEN_MASTER, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM_CANOPEN_MASTER, ENABLE);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_InitStructure.NVIC_IRQChannel = TIM_CANOPEN_MASTER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// config TIM7 for General timer
void TIM_GeneralTimerConfig(uint16_t period, uint16_t prescaler)
{
	NVIC_InitTypeDef         NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(TIM_GENERAL_TIMER_CLK, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = (prescaler - 1);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseStructure.TIM_Period = (period - 1);
	
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM_GENERAL_TIMER, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM_GENERAL_TIMER, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM_GENERAL_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM_GENERAL_TIMER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


//// config TIM6 for AbsEncoder, TIM_ABS_ENCODER_IRQn to deal with the events of AbsEncoder
//void TIM_AbsEncoderConfig(void)
//{
//	NVIC_InitTypeDef         NVIC_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//	RCC_APB1PeriphClockCmd(TIM_ABS_ENCODER_CLK, ENABLE);

//    TIM_TimeBaseStructure.TIM_Prescaler =84-1; /* CNT递增的单位为1微秒 */
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	
//	//period = 2-1 = 0x1, so the interupt interval is 2μs	
//    TIM_TimeBaseStructure.TIM_Period =0x0C;              
////	TIM_TimeBaseStructure.TIM_Period =0xFF;
//	
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM_ABS_ENCODER, &TIM_TimeBaseStructure);

//	TIM_ITConfig(TIM_ABS_ENCODER, TIM_IT_Update, ENABLE);

//    TIM_Cmd(TIM_ABS_ENCODER, ENABLE);
//	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

//    NVIC_InitStructure.NVIC_IRQChannel = TIM_ABS_ENCODER_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

uint8_t SoftTimerInit(SoftTimer_t *SoftTime)
{
    SoftTime->u32BaseCycle = 1;    /*基础时钟周期为1mS*/
    SoftTime->u32TenMilisecond = 10/SoftTime->u32BaseCycle;
    SoftTime->u32OneSecond = 1000/SoftTime->u32BaseCycle;
    
    return 0x00;
}



