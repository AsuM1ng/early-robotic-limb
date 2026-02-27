/********************************************************************************************
* @FileName: MC_PIDPosition.c
* @Date: 2017_08_05
* @Version: v1.0
* @Author: HYF
* @Summary: 1.PID位置控制器，一般采用P控制器实现位置伺服。
*			2.PID位置控制器输入为：pos_ref,pos_fdbk. 输入的单位均为：cnt值
*			将PID的输入单位修改为cnt值!!!
*			编码器一圈为2000pulses
********************************************************************************************/


#include "MC_PIDPosition.h"
#include "Motor_Param.h"
/**********************************************************
* @FuncName: PIDPos_Init
* @Param: *PIDPos_struct
* @Return: None
* @Brief: 初始化PID位置控制器。
* @addtion:
**********************************************************/
void PIDPos_Init(PIDPos_Typedef *PIDPos_struct)
{
	memset(PIDPos_struct, 0, sizeof(PIDPos_Typedef));
	
	PIDPos_struct->PID_struct.prm_Kp = 20;							//50k~100k均可，视刚度而定	
	PIDPos_struct->PID_struct.prm_Ki = 0;
	PIDPos_struct->PID_struct.prm_Kd = 0;
	PIDPos_struct->PID_struct.prm_Hlimit_integral = (int32_t)( NORMINAL_SPEED / 60 * ENCODER_PPR);				//100Hz=100round/s=6000rpm=100*2000cnt/s
	PIDPos_struct->PID_struct.prm_Llimit_integral = (int32_t)(-NORMINAL_SPEED / 60 * ENCODER_PPR);
	PIDPos_struct->PID_struct.prm_Hlimit_output   = (int32_t)( NORMINAL_SPEED / 60 * ENCODER_PPR);
	PIDPos_struct->PID_struct.prm_Llimit_output   = (int32_t)(-NORMINAL_SPEED / 60 * ENCODER_PPR);
}

/**********************************************************
* @FuncName: PIDPos_Input
* @Param: *PIDPos_struct, pos_ref, pos_fdbk
* @Return: None
* @Brief: 更新PID位置控制器的输入,输入包括pos_ref,pos_fdbk.
* @addtion: pos_ref,pos_fdbk单位：cnt
*			最大参考输入：
**********************************************************/
void PIDPos_Input(PIDPos_Typedef *PIDPos_struct, int32_t pos_ref, int32_t pos_fdbk)
{
	PIDPos_struct->i_pos_ref = pos_ref;
	PIDPos_struct->i_pos_fdbk = pos_fdbk;
}

/**********************************************************
* @FuncName: PIDPos_Update_CtrlParam
* @Param: *PIDPos_struct, Kp, Ki, Kd
* @Return: None
* @Brief: 更新PID位置控制器的控制参数
* @addtion:
**********************************************************/
void PIDPos_Update_CtrlParam(PIDPos_Typedef *PIDPos_struct, int32_t Kp, int32_t Ki, int32_t Kd)
{
	PID_Update_CtrlParam(&PIDPos_struct->PID_struct, Kp, Ki, Kd);
}

/**********************************************************
* @FuncName: PIDPos_Update_LimitParam
* @Param: *PIDPos_struct, Hlimit_integral, Llimit_integral,
*			Hlimit_output，Llimit_output
* @Return: None
* @Brief: 更新PID位置控制器的限制参数
* @addtion:
**********************************************************/
void PIDPos_Update_LimitParam(PIDPos_Typedef *PIDPos_struct, int32_t Hlimit_integral, int32_t Llimit_integral, 
								int32_t Hlimit_output, int32_t Llimit_output)
{
	PID_Update_LimitParam(&PIDPos_struct->PID_struct, Hlimit_integral, Llimit_integral, 
							Hlimit_output, Llimit_output);
}

/**********************************************************
* @FuncName: PIDPos_Clear_Output
* @Param: *PIDPos_struct
* @Return: None
* @Brief: 清除PID位置控制器的输出
* @addtion:
**********************************************************/
void PIDPos_Clear_Output(PIDPos_Typedef *PIDPos_struct)
{
	PID_Clear_Output(&PIDPos_struct->PID_struct);
	PIDPos_struct->o_vel_ref = 0;
}

/**********************************************************
* @FuncName: PosLoop_Controller
* @Param: *posloop_struct
* @Return: None.
* @Brief: PID位置控制器
* @addtion: 最大参考输入：cnt
**********************************************************/
void PIDPos_Controller(PIDPos_Typedef *PIDPos_struct)
{
	Pos_PReg(&PIDPos_struct->PID_struct, PIDPos_struct->i_pos_ref, PIDPos_struct->i_pos_fdbk);
	//P_Regulator(&PIDPos_struct->PID_struct, PIDPos_struct->i_pos_ref, PIDPos_struct->i_pos_fdbk);
	
	PIDPos_struct->o_vel_ref = PIDPos_struct->PID_struct.o_output;			//位置控制器输出参考速度，单位：cnt/s

}



