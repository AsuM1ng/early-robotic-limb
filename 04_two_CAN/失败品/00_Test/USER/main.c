#include "sys.h"	
#include "main.h"		


////激活PPM
//unsigned char activ_PPM[5]={0x2F,0x60,0x60,0x00,0x01};
////激活PVM
//unsigned char activ_PVM[5]={0x2F,0x60,0x60,0x00,0x03};
////控制器去使能
//unsigned char EPOS_disable[6] = {0x2B,0x40,0x60,0x00,0x06,0x00};
////控制器使能
//unsigned char EPOS_enable[6] = {0x2B,0x40,0x60,0x00,0x0F,0x00};
////设置目标转速为-10rpm
//unsigned char target_speed_1000rpm[8] = {0x23,0xFF,0x60,0x00,0xF6,0xFF,0xFF,0xFF};
////读取实际转速
//unsigned char read_actual_speed[4] = {0x40,0xD3,0x30,0x01};


////设置动作1_肩关节_起势_正转_速度20rpm
//unsigned char shoulder_start_forward_20rpm[8] = {0x23,0xFF,0x60,0x00,0x14,0x00,0x00,0x00};
////设置动作1_肩关节_往复_正转_速度90rpm
//unsigned char shoulder_circulate_forward_90rpm[8] = {0x23,0xFF,0x60,0x00,0x5A,0x00,0x00,0x00};
////设置动作1_肩关节_起势_反转_速度90rpm
//unsigned char shoulder_circulate_reverse_90rpm[8] = {0x23,0xFF,0x60,0x00,0xA6,0xFF,0xFF,0xFF};
////设置动作1_肘关节_速度（方向待明确）_83rpm
//unsigned char elbow_circulate_reverse_83rpm[8] = {0x23,0xFF,0x60,0x00,0x53,0x00,0x00,0x00};
//		

////设置关节转出速度120
//unsigned char shoulder_horizen_120rpm[8] = {0x23,0x81,0x60,0x00,0x78,0x00,0x00,0x00};
////设置关节目标速度120
//unsigned char shoulder_target_120rpm[8] = {0x23,0xFF,0x60,0x00,0x78,0x00,0x00,0x00};
////肩关节水平自由位置
//unsigned char shoulder_horizen_position[8] = {0x23,0x7A,0x60,0x00,0x00,0x00,0x00,0x00};
////绝对位置模式设置
//unsigned char absolut_position[6] = {0x2B,0x40,0x60,0x00,0x1F,0x00};


////PPMHalt
//unsigned char PPM_Halt[6] = {0x2B,0x40,0x60,0x00,0x0F,0x01};
////PVMHalt
//unsigned char PVM_Halt[6] = {0x2B,0x40,0x60,0x00,0x0F,0x01};


		

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
	
	
	

  unsigned char activ_PVM[5]={0x2F,0x60,0x60,0x00,0x03};
	//控制器去使能
  unsigned char EPOS_disable[6] = {0x2B,0x40,0x60,0x00,0x06,0x00};
  //控制器使能
  unsigned char EPOS_enable[6] = {0x2B,0x40,0x60,0x00,0x0F,0x00};
	//设置关节目标速度120
	unsigned char shoulder_target_120rpm[8] = {0x23,0xFF,0x60,0x00,0x78,0x00,0x00,0x00};
	//PVMHalt
  unsigned char PVM_Halt[6] = {0x2B,0x40,0x60,0x00,0x0F,0x01};
	
	
	
	Message temp;
	temp.cob_id = 0x680;
	temp.rtr = 0;
	temp.len = 8;
	for(int i = 0; i < 8; i++)
	{
		temp.data[i] = shoulder_target_120rpm[i];
	}
	

	
	while(1)
	{
		delay_ms(100);
		canSend(
		
//		delay_ms(1500);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
//		sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
////		delay_ms(10);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
////		delay_ms(10);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
////		delay_ms(10);
//		sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_target_120rpm);
////		delay_ms(10);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
//		delay_ms(15000);
//		sendSDO(&Master_Data,SDO_CLIENT,0,PVM_Halt);
//		delay_ms(60000);
//		delay_ms(60000);
	}
}
