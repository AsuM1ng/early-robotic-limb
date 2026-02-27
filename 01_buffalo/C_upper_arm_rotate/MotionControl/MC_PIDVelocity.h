#ifndef _MC_PIDVELOCITY_H
#define _MC_PIDVELOCITY_H

#include "PID_Regulator.h"
#include "string.h"			//memsetº¯Êý

typedef struct
{
	/************** Input **************/
	intQ15 	i_vel_ref;
	intQ15 	i_vel_fdbk;
	
	/********* Position Controller *********/
	PID_Struct_TypeDef 	PID_struct;
	
	/************** Output **************/
	intQ15 	o_torque_ref;
	
}PIDVel_Typedef;

void PIDVel_Init(PIDVel_Typedef *PIDVel_struct);
void PIDVel_Input(PIDVel_Typedef *PIDVel_struct, intQ15 vel_ref, intQ15 vel_fdbk);
void PIDVel_Update_CtrlParam(PIDVel_Typedef *PIDVel_struct, intQ15 Kp, intQ15 Ki, intQ15 Kd);
void PIDVel_Update_LimitParam(PIDVel_Typedef *PIDVel_struct, intQ15 Hlimit_integral, intQ15 Llimit_integral, 
								intQ15 Hlimit_output, intQ15 Llimit_output);
void PIDVel_Clear_Output(PIDVel_Typedef *PIDVel_struct);
void PIDVel_Controller(PIDVel_Typedef *PIDVel_struct);

#endif


