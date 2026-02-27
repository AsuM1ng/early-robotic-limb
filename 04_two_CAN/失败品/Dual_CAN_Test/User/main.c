/***********************************************************************
文件名称：main.C
功    能：
编写时间：
编 写 人：
注    意：
***********************************************************************/
#include "main.h"	


int main(void)
{
	/*
		ST固件库中的启动文件已经执行了 SystemInit() 函数，该函数在 system_stm32f4xx.c 文件，主要功能是
		配置CPU系统的时钟，内部Flash访问时序，配置FSMC用于外部SRAM等。
	*/
	NVIC_Configuration(); 
	CAN1_Configuration();
	CAN2_Configuration();
	while(1)
	{
		if(can1_rec_flag == 1)	 		//如果CAN1接收到了一帧数据
		{
			can1_rec_flag = 0;
			CAN1_WriteData(0x18412345);	//以ID为 0x18412345向CAN上发送数据
		}
		if(can2_rec_flag == 1)	 		//如果CAN2接收到了一帧数据
		{
			can2_rec_flag = 0;
			CAN2_WriteData(0x18412345);	//以ID为 0x18412345向CAN上发送数据
		} 		
	}
}
