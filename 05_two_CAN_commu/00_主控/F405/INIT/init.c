#include "main.h"


void sys_init(void)
{
	SystemClock_Config();
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
	
	Master_Data.canHandle = CAN1;
	Master2_Data.canHandle = CAN2;
	
	pressf.key0_flag = 0;
	pressf.key1_flag = 0;
	pressf.wkup_flag = 0;

	uint8_t key = 0;
	
	key_init();
	led_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);    	//初始化延时函数
	

	can_init();
	TIM2_Init();
//	TIM3_Init();
	USART1_Init(115200);
}

void SystemClock_Config(void){
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

