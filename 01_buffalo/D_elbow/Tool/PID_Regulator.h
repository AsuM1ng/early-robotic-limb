#ifndef _PID_REGULATOR_H
#define _PID_REGULATOR_H

#include "stdint.h"
#include "Umath.h"

typedef struct
{
	/******************* PID Input ********************/
	intQ15 i_reference;				//参考输入
	intQ15 i_feedback;				//反馈输入
	
	/******************* PID Param ********************/
	intQ15 prm_Kp;					// P
	intQ15 prm_Ki;					// I
	intQ15 prm_Kd;					// D
	intQ15 prm_max_err;             //最大误差
	intQ15 prm_Hlimit_integral;		//积分饱和值上限
	intQ15 prm_Llimit_integral;		//积分饱和值下限
	intQ15 prm_Hlimit_output;		//输出值下限
	intQ15 prm_Llimit_output;		//输出值下限
	
	intQ15 err;
	intQ15 sum_err;
	intQ15 pre_out;
	/******************* PID Output ********************/
	intQ15 o_pre_err;				//上一次的误差值
	intQ15 o_integral;				//积分项
	intQ15 o_output;				//总输出 

}PID_Struct_TypeDef;

//void PID_Set_Input(PID_Struct_TypeDef* PID_struct, intQ15 ref, intQ15 fedbk);
//void PID_Update_CtrlParam(PID_Struct_TypeDef* PID_Struct, 
//							intQ15 Kp, intQ15 Ki, intQ15 Kd);
//void PID_Update_LimitParam(PID_Struct_TypeDef* PID_Struct, intQ15 Hlimit_integral, intQ15 Llimit_integral, 
//							intQ15 Hlimit_output, intQ15 Llimit_output);
//void PID_Clear_Output(PID_Struct_TypeDef* PID_Structure);

//int32_t Pos_PReg(PID_Struct_TypeDef* PID_struct, int32_t ref, int32_t fdbk);
//intQ15 P_Regulator(PID_Struct_TypeDef* PID_struct, intQ15 reference, intQ15 feedback);
//intQ15 PI_Regulator(PID_Struct_TypeDef* PID_struct, intQ15 reference, intQ15 feedback);
//intQ15 PID_Regulator(PID_Struct_TypeDef* PID_Structure, intQ15 reference, intQ15 feedback);
//intQ15 PI_IntegralSepr_Regulator(PID_Struct_TypeDef* PID_struct, intQ15 reference, 
//										intQ15 feedback, intQ15 separate_error);

#endif
