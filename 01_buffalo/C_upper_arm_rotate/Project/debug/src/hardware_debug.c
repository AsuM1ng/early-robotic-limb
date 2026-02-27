
/*
********************************************************************************
*@brife：此文件的功能用于硬件debug
		1.用硬件管脚的开关来测试程序的运行时间
		2.采用DAC将数字量转换成模拟量输出到特定的管件
		
*******************************************************************************/
#include "hardware_debug.h"

void IO_test_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(IO_TEST_GPIO_CLK , ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IO_TEST_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(IO_TEST_GPIO_PORT, &GPIO_InitStructure);
}

void IO_switch(uint8_t status)
{
	if(status!=1)
		GPIO_ResetBits(IO_TEST_GPIO_PORT,IO_TEST_GPIO_PIN);
	else
		GPIO_SetBits(IO_TEST_GPIO_PORT,IO_TEST_GPIO_PIN);		
}




void debug_init(void)
{
	IO_test_init();
}

