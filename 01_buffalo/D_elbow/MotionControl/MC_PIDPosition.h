#ifndef _MC_PIDPOSITION_H
#define _MC_PIDPOSITION_H

#include "PID_Regulator.h"
#include "string.h"			//memset函数
typedef struct
{
	/************** Input **************/
	intQ15 	i_pos_ref;				//输入单位：cnt
	intQ15 	i_pos_fdbk;				//输入单位：cnt
	
	/********* Controller *********/
	PID_Struct_TypeDef 	PID_struct;
	
	/************** Output **************/
	intQ15 	o_vel_ref;			//输出单位：cnt/s
	
}PIDPos_Typedef;

void PIDPos_Init(PIDPos_Typedef *PIDPos_struct);
void PIDPos_Input(PIDPos_Typedef *PIDPos_struct, intQ15 pos_ref, intQ15 pos_fdbk);
void PIDPos_Update_CtrlParam(PIDPos_Typedef *PIDPos_struct, intQ15 Kp, intQ15 Ki, intQ15 Kd);
void PIDPos_Update_LimitParam(PIDPos_Typedef *PIDPos_struct, intQ15 Hlimit_integral, intQ15 Llimit_integral, 
								intQ15 Hlimit_output, intQ15 Llimit_output);
void PIDPos_Clear_Output(PIDPos_Typedef *PIDPos_struct);
void PIDPos_Controller(PIDPos_Typedef *PIDPos_struct);

#endif

