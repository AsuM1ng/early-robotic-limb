#include "main.h"

void sys_init(void)
{
	SystemClock_Config();
	TIM3_NVIC_Configuration();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);    	//初始化延时函数
	

	can_init();
	TIM2_Init();
	USART1_Init(115200);
	
	unsigned char nodeID = 0x01;                   //节点ID
  setNodeId(&Master_Data, nodeID);
  setState(&Master_Data, Initialisation);				//节点初始化
  setState(&Master_Data, Operational);		
	
	setNodeId(&Master2_Data, nodeID);
  setState(&Master2_Data, Initialisation);				//节点初始化
  setState(&Master2_Data, Operational);
	
//	CanTxMsg Operation_active = {0};
//	Operation_active.StdId = 0x00;
//	Operation_active.IDE = CAN_ID_STD;
//	Operation_active.RTR = 0;
//	Operation_active.DLC = 2;
//	Operation_active.Data[0] = 0x01;
//	Operation_active.Data[1] = 0x00;
//	CAN_Transmit(CAN1, &Operation_active);
	Master_Data.canHandle = CAN1;
	Master2_Data.canHandle = CAN2;
}

void hardware_init(void)
{
	key_init();
	led_init();
}

void action_flag_init(void)
{
	actionf.ready = 0;
	actionf.action = 0;
	actionf.halt = 0;
	actionf.zero = 0;
	
	missionp.action_state = 0;
	missionp.action_times = 0;
	missionp.hlat_state = 1;
	missionp.ready_state = 0;
}

void SystemClock_Config(void) {
    RCC_DeInit();
    
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_WaitForHSEStartUp() != SUCCESS);

    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);

    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    while (RCC_GetSYSCLKSource() != 0x08);

    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
