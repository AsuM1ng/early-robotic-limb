
#include "MC_PIDVelocity.h"
#include "Motor_Param.h"

/********************************************************************************************
* @FileName: MC_PIDVelocity.c
* @Date: 2017_01_21
* @Version: v1.0
* @Author: HYF
* @Summary: PID速度控制器
*			1.PID速度控制器，一般采用PI控制器实现速度伺服。
*			2.PID速度控制器输入为：vel_ref,vel_fdbk. 输入的单位均为：step值
********************************************************************************************/

/**********************************************************
* @FuncName: PIDPos_Init
* @Param: *PIDVel_struct
* @Return: None
* @Brief: PID速度控制器初始化。
* @addtion: ordinary PI: kp = 15k, ki = 0.8k;
			seperate PI: kp = , ki = , w_sep=300rpm
**********************************************************/
void PIDVel_Init(PIDVel_Typedef *PIDVel_struct)
{
	memset(PIDVel_struct, 0, sizeof(PIDVel_Typedef));
	
	//	spd_buff_size=1  Kp=15000  Ki=500
	//	spd_buff_size=3  Kp=13000  Ki=500
	//	spd_buff_size=4  Kp=12000  Ki=400
	//	spd_buff_size=8  Kp=10000  Ki=400
	//Kp=30000, Ki=500
	PIDVel_struct->PID_struct.prm_Kp = 8000;							//20k~40k均可	
	PIDVel_struct->PID_struct.prm_Ki = 100;
	PIDVel_struct->PID_struct.prm_Kd = 0;
	PIDVel_struct->PID_struct.prm_Hlimit_integral = (int32_t)( NORMINAL_CURRENT*32768);			//3A-->Q15(A)=Q15(3)=3*32768=98304
	PIDVel_struct->PID_struct.prm_Llimit_integral = (int32_t)(-NORMINAL_CURRENT*32768);
	PIDVel_struct->PID_struct.prm_Hlimit_output   = (int32_t)( NORMINAL_CURRENT*32768);
	PIDVel_struct->PID_struct.prm_Llimit_output   = (int32_t)(-NORMINAL_CURRENT*32768);
}

/**********************************************************
* @FuncName: PIDVel_Input
* @Param: *PIDVel_struct, vel_ref, vel_fdbk
* @Return: None
* @Brief: 更新PID速度控制器的输入,输入包括vel_ref,vel_fdbk.
* @addtion: vel_ref,vel_fdbk位：step.
*			最大参考输入：
**********************************************************/
void PIDVel_Input(PIDVel_Typedef *PIDVel_struct, intQ15 vel_ref, intQ15 vel_fdbk)
{
	PIDVel_struct->i_vel_ref = vel_ref;
	PIDVel_struct->i_vel_fdbk = vel_fdbk;
}

/**********************************************************
* @FuncName: PIDVel_Update_CtrlParam
* @Param: *PIDVel_struct, Kp, Ki, Kd
* @Return: None
* @Brief: 更新PID速度控制器的控制参数
* @addtion:
**********************************************************/
void PIDVel_Update_CtrlParam(PIDVel_Typedef *PIDVel_struct, intQ15 Kp, intQ15 Ki, intQ15 Kd)
{
	PID_Update_CtrlParam(&PIDVel_struct->PID_struct, Kp, Ki, Kd);
}

/**********************************************************
* @FuncName: PIDVel_Update_LimitParam
* @Param: *PIDVel_struct, Hlimit_integral, Llimit_integral,
*			Hlimit_output，Llimit_output
* @Return: None
* @Brief: 更新PID速度控制器的限制参数
* @addtion:
**********************************************************/
void PIDVel_Update_LimitParam(PIDVel_Typedef *PIDVel_struct, intQ15 Hlimit_integral, intQ15 Llimit_integral, 
								intQ15 Hlimit_output, intQ15 Llimit_output)
{
	PID_Update_LimitParam(&PIDVel_struct->PID_struct, Hlimit_integral, Llimit_integral, 
							Hlimit_output, Llimit_output);
}

/**********************************************************
* @FuncName: PIDVel_Clear_Output
* @Param: *PIDVel_struct
* @Return: None
* @Brief: 清除PID速度控制器的输出
* @addtion:
**********************************************************/
void PIDVel_Clear_Output(PIDVel_Typedef *PIDVel_struct)
{
	PID_Clear_Output(&PIDVel_struct->PID_struct);
	PIDVel_struct->o_torque_ref = 0;
}

/**********************************************************
* @FuncName: PosLoop_Controller
* @Param: *posloop_struct
* @Return: None.
* @Brief: PID速度控制器
* @addtion: 最大参考输入：
**********************************************************/
int32_t tst_sepr_val = 200;
void PIDVel_Controller(PIDVel_Typedef *PIDVel_struct)
{
	PI_IntegralSepr_Regulator(&PIDVel_struct->PID_struct, 
								PIDVel_struct->i_vel_ref, PIDVel_struct->i_vel_fdbk, UM_Q(tst_sepr_val/60));
	
//	PI_Regulator(&PIDVel_struct->PID_struct, PIDVel_struct->i_vel_ref, PIDVel_struct->i_vel_fdbk);
	PIDVel_struct->o_torque_ref = PIDVel_struct->PID_struct.o_output;			//位置控制器输出参考速度，单位：step

}



