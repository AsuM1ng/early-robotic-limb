#include "sys.h"	
#include "main.h"		


int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);    	//初始化延时函数
	

	CAN1_Init(&Master_Data,1000000);
	TIM3_Init();
	USART1_Init(115200);
	
	unsigned char nodeID = 0x01;                   //节点ID
  setNodeId(&Master_Data, nodeID);
  setState(&Master_Data, Initialisation);				//节点初始化
  setState(&Master_Data, Operational);		


	
	while(1)
	{
		delay_ms(500);
		
	unsigned char get_test_datasend[8]={0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x00};
	sendSDO(&Master_Data,SDO_CLIENT,0,get_test_datasend);
	}
}
