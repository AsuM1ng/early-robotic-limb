#include "main.h"	

typedef struct
{
	uint8_t right_key_flag;
	uint8_t down_key_flag;
	uint8_t left_key_flag;
	uint8_t up_key_flag;
} PressFlag;


PressFlag pressf;

int main(void)
{ 
	pressf.right_key_flag = 0;
	pressf.left_key_flag = 0;
	pressf.down_key_flag = 0;
	pressf.up_key_flag = 0;
	
	uint8_t key;
	uint8_t a = 0;
	
	key_init();
	led_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);    	//初始化延时函数
	

	CAN1_Init(&Master_Data,1000000);
//	CAN2_Init(&Master2_Data,1000000);
	TIM2_Init();
	USART1_Init(115200);
	
	unsigned char nodeID = 0x01;                   //节点ID
	
	setNodeId(&Master_Data, nodeID);
  setState(&Master_Data, Initialisation);				//节点初始化
  setState(&Master_Data, Operational);

//  unsigned char activ_PVM[5]={0x2F,0x60,0x60,0x00,0x03};
//	//控制器去使能
//  unsigned char EPOS_disable[6] = {0x2B,0x40,0x60,0x00,0x06,0x00};
//  //控制器使能
//  unsigned char EPOS_enable[6] = {0x2B,0x40,0x60,0x00,0x0F,0x00};
//	//设置关节目标速度120
	unsigned char shoulder_target_120rpm[8] = {0x23,0xFF,0x60,0x00,0x78,0x00,0x00,0x00};
//	//PVMHalt
//  unsigned char PVM_Halt[6] = {0x2B,0x40,0x60,0x00,0x0F,0x01};
	
	Message temp;
	temp.cob_id = 0x601;
	temp.rtr = 0;
	temp.len = 8;
	for(int i = 0; i < 8; i++)
	{
		temp.data[i] = shoulder_target_120rpm[i];
	}
	
	while(1)
	{
		key = key_scan(0);
		if(key)
		{
			switch(key)
			{
				case KEY0_PRES: pressf.right_key_flag = 1; break;
				case KEY1_PRES:	pressf.down_key_flag = 1;  break;
				case KEY2_PRES:	pressf.left_key_flag = 1;  break;
				case WKUP_PRES:	pressf.up_key_flag = 1;		 break;
			}
		}

		if(pressf.right_key_flag)
		{
			a = canSend(CAN1, &temp);
			if(!a)
				PFout(9) = 0;
			pressf.right_key_flag = 0;
		}
		
		if(pressf.down_key_flag)
		{
			PFout(10) = 0;
			pressf.down_key_flag = 0;
		}
		
		if(pressf.left_key_flag)
		{
			PFout(9) = 1;
			pressf.left_key_flag = 0;
		}
		
		if(pressf.up_key_flag)
		{
			PFout(10) = 1;
			pressf.up_key_flag = 0;
		}
	}
}
