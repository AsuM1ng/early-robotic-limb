#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "main.h"

typedef struct{	
	//以下是MCU向电机设置的参数(上下两者用于实现通信丢帧检测负反馈)
	UNS32 Profile_Velocity_Target_Set;		//电机目标轮廓速度设置
	UNS32 Profile_Acceleration_Set;				//电机轮廓加速度设置值
	UNS32 Profile_Deceleration_Set;				//电机轮廓减速度设置值
	UNS32 Motor_Target_Position_Set;			//电机目标位置设置值
	
	//以下是从电机读来的实际参数
	UNS32 Profile_Velocity;						//电机当前平均轮廓速度
	UNS32 Profile_Velocity_Max;				//电机最大轮廓速度
	UNS32 Profile_Velocity_Target;		//电机目标轮廓速度
	
	UNS32 Profile_Acceleration;				//电机轮廓加速度
	UNS32	Profile_Deceleration;				//电机轮廓减速度
	UNS32 Profile_Acceleration_Max;		//电机最大加速度
	
	UNS32 Motor_Position;							//电机当前位置
	UNS32 Motor_Target_Position;			//电机目标位置					在motor_control.c文件中读取
	UNS32 Motor_Position_limit_Min;		//电机软件限位下限
	UNS32 Motor_Position_limit_Max;		//电机软件限位上限
	
	UNS16 Motor_Torque;								//电机当前转矩
	UNS16	Motor_Torque_Target;				//电机目标转矩
	UNS32 Motor_Torque_Rated;					//电机额定转矩
	
	UNS32 Power_Supply;								//电机供电电压
	unsigned char MotorData[4];				//存放电机指令的控制参数，例如目标速度，目标位置等。
} MotorData;
extern MotorData MD;


typedef struct{
	UNS16 Statusword;									//电机当前状态
	UNS16 Controlword;								//操作指令状态
	UNS16 Error_Code;									//电机当前错误代码
} MotorState;
extern MotorState MS;

//来自主控的命令指令，表明现在要做什么事，宏观调度
typedef struct
{
	u8 ready;										//到预备位置命令标志
	u8 halt;										//暂停当前动作命令标志
	u8 zero;										//回到0点动作命令标志
	
	u8 drink;										//喝水动作命令标志
	u8 Stretching_exercise;			//敬礼动作命令标志
	u8 anyother;								//待完善
	
	u8 halt_2;									//循环动作第二层跳出
} TaskCommand;

extern TaskCommand TC;

typedef struct
{
	u8 operating;								//任务执行中
	u8 waiting;									//等待指令中（对应Halt）
	u8 task_targetIterations;		//任务目标重复次数
	u8 task_currentIterations;	//目标当前已执行次数
} TaskStatus;

extern TaskStatus TS;




void Action_Process(void);
void Action_Ready(void);
void Action_Drink(void);
void Action_Halt(void);
void Action_Posion(void);
void Move2Zero(void);
void target_parameter_set(u8 reduction_ratio, u32 joint_relative_angle, u32 motor_velocity, u32 motor_acceleration, u32 motor_deceleration);
u16 task_time_calculator(u8 reduction_ratio, u32 absolute_angle);
void isMotorTaskFinished(void);
void task_start(void);

extern u16 task_time;
extern u16 task_executed_times;

#endif

