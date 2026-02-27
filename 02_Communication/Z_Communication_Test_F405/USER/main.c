#include "sys.h"	
#include "main.h"		

void motor_1000rmp_10s();

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);    	//初始化延时函数
	

	CAN1_Init(&Master_Data,1000000);
	TIM2_Init();
//	TIM3_Init();
	USART1_Init(115200);
	
	unsigned char nodeID = 0x01;                   //节点ID
  setNodeId(&Master_Data, nodeID);
  setState(&Master_Data, Initialisation);				//节点初始化
  setState(&Master_Data, Operational);		

		//激活PVM
		unsigned char activ_PVM[5]={0x2F,0x60,0x60,0x00,0x03};
		//控制器去使能
		unsigned char EPOS_disable[6] = {0x2B,0x40,0x60,0x00,0x06,0x00};
		//控制器使能
		unsigned char EPOS_enable[6] = {0x2B,0x40,0x60,0x00,0x0F,0x00};
		//设置目标转速为1000rpm
		unsigned char target_speed_1000rpm[8] = {0x23,0xFF,0x60,0x00,0xE8,0x03,0x00,0x00};
		//读取实际转速
		unsigned char read_actual_speed[4] = {0x40,0xD3,0x30,0x01};
	
	while(1)
	{
		delay_ms(1000);
		sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
		delay_ms(10);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
		delay_ms(10);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		delay_ms(10);
		sendSDO(&Master_Data,SDO_CLIENT,0,target_speed_1000rpm);
		delay_ms(10);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		delay_ms(2000);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	}
}



