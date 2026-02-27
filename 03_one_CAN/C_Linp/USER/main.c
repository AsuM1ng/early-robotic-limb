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
	int i = 0;
	
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
//	//设置关节目标速度120
//	unsigned char shoulder_target_120rpm[8] = {0x23,0xFF,0x60,0x00,0x78,0x00,0x00,0x00};
//设置动作1_肘关节_速度（方向待明确）_83rpm
unsigned char elbow_circulate_reverse_83rpm[8] = {0x23,0xFF,0x60,0x00,0xAD,0xFF,0xFF,0xFF};
	//PVMHalt
  unsigned char PVM_Halt[6] = {0x2B,0x40,0x60,0x00,0x0F,0x01};
	
////设置动作1_肩关节_起势_正转_速度20rpm
//unsigned char shoulder_start_forward_20rpm[8] = {0x23,0xFF,0x60,0x00,0x14,0x00,0x00,0x00};
////设置动作1_肩关节_往复_正转_速度90rpm
//unsigned char shoulder_circulate_forward_90rpm[8] = {0x23,0xFF,0x60,0x00,0x5A,0x00,0x00,0x00};
////设置动作1_肩关节_起势_反转_速度90rpm
//unsigned char shoulder_circulate_reverse_90rpm[8] = {0x23,0xFF,0x60,0x00,0xA6,0xFF,0xFF,0xFF};
	

	delay_ms(15000);
	while(1)
	{
		
		delay_ms(1500);
		sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		sendSDO(&Master_Data,SDO_CLIENT,0,elbow_circulate_reverse_83rpm);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		delay_ms(24000);
		sendSDO(&Master_Data,SDO_CLIENT,0,PVM_Halt);
		delay_ms(60000);
		
	}
}



		
//		delay_ms(500);
//		sendSDO(&Master_Data,SDO_CLIENT,0,activ_PPM);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_horizen_120rpm);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_horizen_position);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,absolut_position);
//		delay_ms(1);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
//		delay_ms(500);
//		sendSDO(&Master_Data,SDO_CLIENT,0,PPM_Halt);
//		while(1)
//		{
//			
//		}
		
	
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
//		delay_ms(2000);
//		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
		
//右手_肩关节_反转_速度
		
		
//设置动作2_肩关节_起势_正转_速度40rpm
		
//设置动作2_肩关节_起势_正转_速度92rpm
		
//设置动作2_肩关节_起势_反转_速度92rpm
		
//设置动作2_肘关节_速度（方向待明确）_96rpm
		
	
		//动作1
		//肩关节，起势，2°开始，一秒转1°，转到18°。然后停2.5秒开始往复运动。18-36-18-36，可来回5组。4秒单面，停1秒再一个单面。肩关节。
		//肘关节，手臂伸直开始转，转102°，24秒，然后转停。
		//加速度很慢
		
		//肩关节减速器减速比120
		//肩关节起势速度：1/6rpm，往复速度：0.75rpm
		//肩关节电机。起势速度：20rpm，往复速度：90rpm
		//肘关节速度：0.689rmp
		//肘关节电机速度：83rpm
		
		
		//动作2
		//肩关节，起势，-2°降到-38°，18秒，停1s。-38°升到+35°，16s。+35到-38°，停1s时间相同
		//肘关节，手臂伸直开始转，转81°，17秒。停止
		//加速度很慢
		
		//肩关节减速器减速比120
		//肩关节起势速度，1/3rpm，往复速度：0.7604rpm
		//肩关节电机速度：起势：40rpm，往复速度：92rpm
		//肘关节速度：0.794rpm
		//肘关节电机速度：96rpm
		
		//肩关节水平转出速度：120rpm