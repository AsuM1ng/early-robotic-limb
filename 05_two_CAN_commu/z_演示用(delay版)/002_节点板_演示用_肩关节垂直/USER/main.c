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
				//KEY0按下执行
		if(!GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_GPIO_PIN))
		{
			actionf.ready = 1;
			PDout(2) = !PDout(2);
			delay_ms(300);
		}
		
		//KEY1按下执行
		if(!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
		{
			actionf.action = 1;
			PAout(8) = !PAout(8);
			delay_ms(300);
		}

		//WK_UP按下执行
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
			actionf.zero = 1;
			PDout(2) = !PDout(2);
			PAout(8) = !PAout(8);
			delay_ms(300);
		}
		
		Action_Process();
	}
}

