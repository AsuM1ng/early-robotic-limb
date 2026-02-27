//进程控制函数、任务计时函数、任务指令下达函数、电机目标位置设置函数。
//细节注意：SDO通信反馈需要时间，判定需要明确代码执行的时间并添加适当的延迟。

#include "main.h"
#include "math.h"

/*****  电机参数监控结构体声明  *****/
MotorData MD;
MotorState MS;

/*****  标志位调度结构体  *****/
TaskCommand TC;
TaskStatus TS;

/*****  任务全局变量  *****/
u16 task_time;								//单次任务定时时长
u16 task_executed_times;			//重复任务已执行次数



//只处理上位机来的标志位和任务调度
void Action_Process(void)
{	
	//开始ready动作，后台计时等待可能发生的暂停指令
	if(TC.ready && TS.waiting)
	{
		Action_Ready();
		TC.ready = 0;
		TC.halt_2 = 0;
	}		
	
	if(TC.zero && TS.waiting)
	{
		Move2Zero();
		TC.zero = 0;
		TC.halt_2 = 0;
	}
	if(TC.drink && TS.waiting)
	{
		Action_Drink();
		TC.drink = 0;
		TC.halt_2 = 0;
	}
	
	if(TC.Stretching_exercise && TS.waiting)
	{
		Action_Stretching_exercise();
		TC.Stretching_exercise = 0;
		TC.halt_2 = 0;
	}
	
}

/********************************/
/******  任务指令下达函数  ******/
/********************************/

//从任意位置移动到Read_Position。
void Action_Ready(void)
{
	target_parameter_set(120, -90, 60, 50, 50);		//相对位移90°，6°/s，电机加速度减速度均为50.
	task_start();
	isMotorTaskFinished();
}	

//从任意位置移动到Zero_Position
void Move2Zero(void)
{
	target_parameter_set(120, 0, 60, 50, 50);		//相对位移90°，6°/s，电机加速度减速度均为50.
	task_start();
	isMotorTaskFinished();
}


void Action_Drink(void)
{
	
}

void Action_Stretching_exercise(void)
{
	u8 num = 0;
	u8 flag = 1;
	
	while(num < 5)
	{
		if(TC.halt_2)
		{
			task_stop_timing();
			timing_end_flag = 0;
			TC.halt_2 = 0;
			break;
		}
		
		if(timing_end_flag)
		{
			task_stop_timing();
			flag = 1;
			timing_end_flag = 0;
			num ++;
		}
		
		if(flag && (num == 0))
		{
			flag = 0;
			target_parameter_set(120, -90, 120, 60, 60);
			task_start();
			isMotorTaskFinished();
			task_time_set(20);
		}
				
		if(flag && (num == 1))
		{
			flag = 0;
			target_parameter_set(120, 0, 120, 60, 60);
			task_start();
			isMotorTaskFinished();
			task_time_set(20);
		}
		
		if(flag && (num == 2))
		{
			flag = 0;
			target_parameter_set(120, -90, 120, 60, 60);
			task_start();
			isMotorTaskFinished();
			task_time_set(20);
		}
		
		if(flag && (num == 3))
		{
			flag = 0;
			target_parameter_set(120, 0, 120, 60, 60);
			task_start();
			isMotorTaskFinished();
			task_time_set(20);
		}	
		
		if(flag && (num == 4))
		{
			flag = 0;
			target_parameter_set(120, -90, 120, 60, 60);
			task_start();
			isMotorTaskFinished();
			task_time_set(20);
		}	
	}
}



void check_EPOS_status(int Relative_angle)
{
	
}


//考虑到可能存在错位的情况，不能用上一次设置的值来算，虽然有Halt这个东西，但我感觉还是得通信一下。
//这个部分读取参数是做了一层负反馈，保证参数设置那一步的通信是正确的。如果不正确则通信报错
u16 task_time_calculator(u8 reduction_ratio, u32 absolute_angle)				//单次相对位移的角度
{
	u32 delata_inc = 0;									//机械臂关节单次运动的inc差值
	int inc_absolute_value = 0;					//关节角度转化为电机实际需要运行的inc值
	u32 rotating_inc_speed = 0;					//电机目标轮廓速度变成inc速度
	u32 rotating_inc_acceleration = 0;	//电机转动轮廓加速度变为inc加速度
	u32 rotating_inc_deceleration = 0;	//电机转动轮廓减速度变为inc减速度
	u16 temp_time = 0;

	//绝对位置设置为inc值（int类）
	switch(reduction_ratio)
	{
		case 120: inc_absolute_value = angle_to_inc_120(absolute_angle); break;
		case 50: inc_absolute_value = angle_to_inc_50(absolute_angle); break;
		default: break;
	}
	
	//求差值，算总路程
	delata_inc = (u32)fabs(inc_absolute_value - (int)MD.Motor_Target_Position);
	//inc速度
	rotating_inc_speed = (u32)trunc(fabs(MD.Profile_Velocity_Target_Set * 4096 / 60));
	//inc加速度
	rotating_inc_acceleration = (u32)trunc(fabs(MD.Profile_Acceleration_Set * 4096 / 60));
	//inc减速度
	rotating_inc_deceleration = (u32)trunc(fabs(MD.Profile_Deceleration_Set * 4096 / 60));
		
	//时间取整，直接扔掉小数，在此基础上+1时间周期保证任务完成
	temp_time = (u16)trunc(10 * ((delata_inc / rotating_inc_speed) + (rotating_inc_speed / 2 / rotating_inc_acceleration) + (rotating_inc_speed / 2 / rotating_inc_deceleration)));					//计算时间的函数是100ms为基础单位，直接先乘以10了作为输入时间。
//	}
//	else  //此处向主控发送信号，待补充
		
	return temp_time;
}


//关节单次行动参数设置。输入为电机关节相对目标位置°（填写度数），电机关节角速度单位为0.1°ps（度/秒）（也就是说，要实现1°ps，输入为10），电机本身的加速度和减速度（不考虑减速比的）
//reduction_ratio这个参数只能选120和50。
//不涉及到标志位的处理
void target_parameter_set(u8 reduction_ratio, u32 joint_absolute_angle, u32 motor_velocity, u32 motor_acceleration, u32 motor_deceleration)			
{
	masterSendNMTstateChange(&Master_Data, 0x00, 0x01);
	delay_ms(10);
	sendSDO(&Master_Data,SDO_CLIENT,0,Error_Clear);				//清除错误状态
	task_time = task_time_calculator(reduction_ratio, joint_absolute_angle) + 2; //+2用于保证到位
	u32 target_position = 0;
	switch(reduction_ratio)
	{
		case 120: 
			target_position = (u32)angle_to_inc_120(joint_absolute_angle);
			MD.Profile_Velocity_Target_Set = motor_velocity * 2;				//关节速度°ps换算成电机的实际转速rpm。
			break;
		case 50: 
			target_position = (u32)angle_to_inc_50(joint_absolute_angle);

//			MD.Profile_Velocity_Target_Set = motor_velocity * 2;				//关节速度°ps换算成电机的实际转速rpm。
			break;
		default: break;
	}
	
//暂时不加入安全，这一块直接注释掉了
//	//将目标加速度和速度写入对比用的set值
//	MD.Profile_Acceleration_Set = motor_acceleration;
//	MD.Profile_Deceleration_Set = motor_deceleration;
	
	//将目标位置写入控制数组	
	MD.Motor_Target_Position_Set = target_position;
	num_to_motor_data(target_position);													//将32位目标的绝对位置变成4个8位数存到MD.MotorData[4]中
	for(u8 i = 4; i < 8; i++)
		Target_Position[i] = MD.MotorData[i-4];										//直接调用结构体参数。
	
	//将目标速度写入控制数组	
	num_to_motor_data(MD.Profile_Velocity_Target_Set);					//将32位目标的绝对位置变成4个8位数存到MD.MotorData[4]中
	for(u8 i = 4; i < 8; i++)
		Target_Velocity[i] = MD.MotorData[i-4];										//直接调用结构体参数。
	
	//将目标加速度写入控制数组	
	num_to_motor_data(motor_acceleration);					//将32位目标的绝对位置变成4个8位数存到MD.MotorData[4]中
	for(u8 i = 4; i < 8; i++)
		Target_Acceleretion[i] = MD.MotorData[i-4];										//直接调用结构体参数。
	
	//将目标加速度写入控制数组	
	num_to_motor_data(motor_acceleration);					//将32位目标的绝对位置变成4个8位数存到MD.MotorData[4]中
	for(u8 i = 4; i < 8; i++)
		Target_Deceleretion[i] = MD.MotorData[i-4];										//直接调用结构体参数。
	
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
	sendSDO(&Master_Data,SDO_CLIENT,0,activ_PPM);
	sendSDO(&Master_Data,SDO_CLIENT,0,Target_Velocity);
	sendSDO(&Master_Data,SDO_CLIENT,0,Target_Acceleretion);
	sendSDO(&Master_Data,SDO_CLIENT,0,Target_Deceleretion);
	sendSDO(&Master_Data,SDO_CLIENT,0,Target_Position);
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_disable);
}

//此后写EPOS动作使能函数
//再之后做动作时间计算，用set值来算
//此时EPOS反馈应该已经完成了，需要验证一下，然后进行动作前判定
//最后执行动作

void task_start(void)
{
	TS.waiting = 0;
	sendSDO(&Master_Data,SDO_CLIENT,0,EPOS_enable);
	delay_ms(90);
	sendSDO(&Master_Data,SDO_CLIENT,0,SSI_immediately);
	TS.operating = 1;						//进入操作状态
	TS.waiting = 0;							//不再是等待状态
}

//待测试
void isMotorTaskFinished(void)
{
	MS.Statusword = 0;
	u16 flag = 0;
	while((!flag))
	{
		if(TC.halt)
		{
			TC.halt_2 = 1;
			TC.halt = 0;
			TC.ready = 0;
			TC.zero = 0;
			TC.anyother = 0;
			TC.Stretching_exercise = 0;
			TC.drink = 0;
			sendSDO(&Master_Data,SDO_CLIENT,0,Halt);
			break;
		}
		sendSDO(&Master_Data,SDO_CLIENT,0,Read_EPOS_Statusword);
		delay_ms(10);
		flag = MS.Statusword & 0x0400;
	}
	TS.waiting = 1;
	PAout(8) = !PAout(8);
}

//void Action_Halt(void)
//{
//	sendSDO(&Master_Data,SDO_CLIENT,0,PVM_Halt);
//}


