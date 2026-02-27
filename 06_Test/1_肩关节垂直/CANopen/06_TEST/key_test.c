#include "main.h"

void key_test(void)
{
	if(!GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_GPIO_PIN))
		{
			delay_ms(300);
			PDout(2) = !PDout(2);
			masterSendNMTstateChange(&Master_Data, 0x00, 0x01);
			delay_ms(10);
			Action_Ready();
		}
		
		if(!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
		{
			delay_ms(300);
			TC.halt = 1;			
		}
		
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
//			masterSendNMTstateChange(&Master_Data, 0x00, 0x01);
			delay_ms(300);
			PDout(2) = !PDout(2);
			timing_end_flag = 0;
			Move2Zero();
			
		}
}
