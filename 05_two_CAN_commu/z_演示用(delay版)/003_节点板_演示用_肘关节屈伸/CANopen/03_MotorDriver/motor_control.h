#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "main.h"

typedef struct{
	UNS32 Motor_Speed;								//电机当前平均转速
	UNS32 Motor_Speed_Max;						//电机最大运行速度
	UNS32	Motor_Speed_Target;					//电机目标速度
	
	UNS32 Profile_Velocity;						//电机当前平均轮廓速度
	UNS32 Profile_Velocity_Max;				//电机最大轮廓速度
	UNS32 Profile_Velocity_Target;		//电机目标轮廓速度
	
	UNS32 Profile_Acceleration;				//电机当前轮廓加速度
	UNS32	Profile_Deceleration;				//电机当前轮廓减速度
	
	
	UNS32 Motor_Acceleration;					//电机当前加速度
	UNS32 Motor_Acceleration_Max;			//电机最大加速度
	UNS32 Motor_Acceleration_Target;	//电机目标加速度
	
	int Motor_Position;							//电机当前位置
	int Motor_Position_Target;			//电机目标位置
	int Motor_Position_limit_Min;		//电机软件限位下限
	int Motor_Position_limit_Max;		//电机软件限位上限
	
	UNS16 Motor_Torque;								//电机当前转矩
	UNS16	Motor_Torque_Target;				//电机目标转矩
	UNS32 Motor_Torque_Rated;					//电机额定转矩
	
	UNS32 Power_Supply;								//电机供电电压
} MotorData;
extern MotorData MD;

typedef struct{
	UNS16 Motor_Status;								//电机当前状态
	UNS16 Error_Code;									//电机当前错误代码
} MotorState;
extern MotorData MS;

void Action_Process(void);
void Action_Ready(void);
void Action_Drink(void);
void Action_Halt(void);
void Action_Posion(void);
void Move2Zero(void);


#endif

