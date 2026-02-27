#include "main.h"

/*****  电机参数监控结构体声明  *****/
MotorData MD;
MotorData MS;


void Action_Process(void)
{
/************************************/
/******  用户暂停当前所有动作  ******/
/************************************/
	if(actionf.halt)
	{
		//清空所有其他标志位
		action_finish_flag = 0;
		actionf.ready = 0;
		actionf.action = 0;
		actionf.halt = 0;
		missionp.ready_state = 0;
		missionp.action_state = 0;
		//发送halt指令
		sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
		missionp.hlat_state = 1;
	}
	
/********************************/
/******  起始动作触发执行  ******/
/********************************/
	//开始ready动作，后台计时等待可能发生的暂停指令
	if(actionf.ready)
	{
		Action_Ready();
		actionf.ready = 0;
	}
		
/*******************************/
/******  中间动作触发执行 ******/
/*******************************/
	//开始action动作
	if(actionf.action)
	{
		Action_Drink();
		actionf.action = 0;
	}
	
/***********************************/
/******  回到0点动作触发执行  ******/
/***********************************/
	if(actionf.zero)
	{
		Move2Zero();
		actionf.zero = 0;
	}
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


//位置模式，移动到准备位置，整个过程持续15.6s
void Action_Ready(void)
{
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_target_120rpm);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	delay_ms(15000);
	sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
}

//void Action_Ready_Halt(void)
//{
//	
//}	


void Action_Drink(void)
{
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PVM);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_start_forward_20rpm);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	delay_ms(1600);
	sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
	delay_ms(2500);
	for(uint8_t i = 0; i < 5; i++)
	{
		sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_circulate_forward_90rpm);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		delay_ms(4000);
		sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
		delay_ms(1000);
		sendSDO(&Master_Data,SDO_CLIENT,0,shoulder_circulate_reverse_90rpm);
		sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
		delay_ms(4000);
		sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
		delay_ms(1000);
	}
	sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
}

void Action_Posion(void)
{
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PPM);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Velocity);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Acceleration);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Deceleration);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,Posotion_Target);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,SSI_immediately);
//	action_time_set(1.5);
//	delay_ms(13200);
	delay_ms(4000);
	sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
}

void Move2Zero(void)
{
	//读取电机当前位置
	sendSDO(&Master_Data,SDO_CLIENT,0,Actual_Position);
	//根据电机当前位置设置定时器时间
	u16 time = back0time(MD.Motor_Position);
	//位置模式！启动！
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PPM);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Velocity);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Acceleration);
	sendSDO(&Master_Data,SDO_CLIENT,0,Profile_Deceleration);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,Move_to_Zero);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	sendSDO(&Master_Data,SDO_CLIENT,0,SSI_immediately);
//	action_time_set(80);
	delay_ms(17000);
}

//void Action_Halt(void)
//{
//	sendSDO(&Master_Data,SDO_CLIENT,0,PVM_Halt);
//}


