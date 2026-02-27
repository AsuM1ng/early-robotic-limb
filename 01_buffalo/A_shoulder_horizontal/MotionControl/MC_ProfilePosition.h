#ifndef _MC_PROFILE_POSITION_H
#define _MC_PROFILE_POSITION_H

#include "string.h"
#include "PID_Regulator.h"
//#include "Motor_Param.h"

typedef struct
{
	/***************** Generator status *****************/
	int8_t	pending_flag;				//控制器挂起标志位
	int8_t	update_flag;				//目标位置切换开关，用于切换目标位置时设定当前的o_pos_ref为起始位置.
	
	/***************** Input *****************/
	int32_t 	i_acc;
	int32_t 	i_dec;
	int32_t		i_vel_limit;		//速度限制值
	int32_t 	i_pos_start;      	//开始位置点
	
	int32_t 	i_pos_end;        	//结束位置点(绝对模式)
	int32_t		i_pos_inc;			//位置增量(增量模式)，有正有负
	
	/************** Sensor Info **************/
	int32_t		i_pos_fdbk;
	int32_t		i_vel_fdbk;
	
	/***************** Param *****************/
	int32_t 	prm_PTP_mode;				//PTP模式：0-->绝对模式；1-->增量模式
	float 		prm_pos_Ts; 	           	//采样时间

	/****************** Middle ****************/
	int32_t 	err_pos;
	int32_t 	pos_radius;				//位置到达半径,这个半径在至少取10，一般取20可以适应
	int32_t 	delta_acc;				//单位时间速度的加速增量
	int32_t		delta_dec;				//单位时间速度的减速增量
	
	/***************** Output *****************/
	int32_t o_pos_ref;					//参考位置: cnt
	int32_t	o_vel_ref;				//参考速度输出: cnt/s

}PTP_Typedef;

void PTP_Init(PTP_Typedef *PTP_struct, float pos_Ts);
void PTP_Clear_Output(PTP_Typedef *PTP_struct);
void PTP_Set_value(PTP_Typedef *PTP_struct, int32_t acc, int32_t dec, int32_t vel_limit,
					int32_t pos_start, int32_t pos_end);
void PTP_Input(PTP_Typedef *PTP_struct, int32_t pos_fdbk, int32_t vel_fdbk);
void PTP_Generator(PTP_Typedef *PTP_struct);

#endif	//end _MC_PROFILE_POSITION_H
