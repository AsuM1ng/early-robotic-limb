#include "main.h"

void key_driver_task(void)
{
	//KEY0按下执行
	if(!GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_GPIO_PIN))
	{
		Master_Data.canHandle = CAN1;
		sendSDO(&Master_Data,SDO_CLIENT,0,Ready);
		PDout(2) = !PDout(2);
		delay_ms(300);
	}
		
	//KEY1按下执行
	if(!GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN))
	{
		Master_Data.canHandle = CAN1;
		sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
		PAout(8) = !PAout(8);
		delay_ms(300);
	}

	//WK_UP按下执行
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
	{
		Master_Data.canHandle = CAN1;
		sendSDO(&Master_Data,SDO_CLIENT,0,Zero);
		PDout(2) = !PDout(2);
		PAout(8) = !PAout(8);
		delay_ms(300);
	}
		
	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
	{
		Master_Data.canHandle = CAN1;
		sendSDO(&Master_Data,SDO_CLIENT,0,Drink);
		PDout(2) = !PDout(2);
		PAout(8) = !PAout(8);
		delay_ms(300);
	}
	
	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
	{
		Master_Data.canHandle = CAN1;
		sendSDO(&Master_Data,SDO_CLIENT,0,Stretching_exercise);
		PDout(2) = !PDout(2);
		PAout(8) = !PAout(8);
		delay_ms(300);
	}
	
}



