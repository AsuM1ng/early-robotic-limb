#include "main.h"	

ActionFlag actionf;
MissionProcess missionp;

int main(void)
{ 
/******  系统初始化  ******/
	action_flag_init();
	sys_init();
	hardware_init();
	
	Master_Data.canHandle = CAN1;
	Master2_Data.canHandle = CAN2;
	
	RCC_ClocksTypeDef RCC_CLK;
	RCC_GetClocksFreq(&RCC_CLK);

//	PDout(2) = 0;
//	action_time_set(100);

	int a = 1;

//	action_time_set(150);
//	delay_ms(1000);
//	PDout(2) = 1;
	
	while(1)
	{
		Action_Process();
	}
}

