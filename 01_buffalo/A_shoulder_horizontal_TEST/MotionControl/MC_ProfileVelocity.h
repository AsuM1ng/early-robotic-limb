/********************************************************************************************
* @FileName:
* @Date:
* @Version:
* @Author:
* @Summary:
********************************************************************************************/
#ifndef _MC_PROFILE_VELOCITY_H
#define _MC_PROFILE_VELOCITY_H

#include "string.h"
#include "PID_Regulator.h"
#include "Motor_Param.h"

typedef struct
{
	/************ Generator status ************/
	int8_t			pending_flag;		//控制器挂起标志位,也是速度到达标志
	
	/****************** Input *****************/
	int32_t 		i_acc;				//加速度
	int32_t 		i_dec;				//减速度
	int32_t			i_vel_cur;			//起始速度
	int32_t 		i_vel_target;		//目标速度
	int32_t			i_SD;				//Stop Dec
	
	/************** Sensor Info **************/
	int32_t			i_vel_fdbk;			//暂时没用上
	
	/****************** Parameter*************/
	float 			prm_vel_Ts;			//速度采样周期，单位：s
	float 			prm_SF;				//平滑因子
	
	/***************** Middle ****************/
	int32_t 		delta_acc;			//单位时间速度的加速增量
	int32_t			delta_dec;			//单位时间速度的减速增量
	int8_t 			dir;				//方向标志位
	
	/****************** Output *****************/
	int32_t			o_vel_ref;			//速度参考,单位：cnt/s

}PV_Typedef;

void PV_Init(PV_Typedef *PV_struct);
void PV_Set_Value(PV_Typedef *PV_struct, int32_t acc, int32_t dec, int32_t vel_start, int32_t vel_target);
void PV_Input(PV_Typedef *PV_struct, int32_t vel_fdbk);
void PV_Clear_Output(PV_Typedef* PV_struct);
int32_t PV_Generator(PV_Typedef* PV_struct);

#endif

