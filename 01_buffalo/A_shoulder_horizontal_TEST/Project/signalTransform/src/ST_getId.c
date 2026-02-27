#include "stm32f4xx.h"
#include "ST_GetId.h"
#include "chipHAL_GPIO.h"


UNS16 getId(void)
{
	UNS16 node_id = 0;
	node_id = GPIO_ReadInputDataBit(CANID0_GPIO_PORT, CANID0_GPIO_PIN) + 2 * GPIO_ReadInputDataBit(CANID1_GPIO_PORT,CANID1_GPIO_PIN)
		+ 4 * GPIO_ReadInputDataBit(CANID2_GPIO_PORT, CANID2_GPIO_PIN) + 8 * GPIO_ReadInputDataBit(CANID3_GPIO_PORT, CANID3_GPIO_PIN);

	node_id = node_id^0x0F;  //相当于低4位取反，高4位不变。	
	
	return node_id;
}


