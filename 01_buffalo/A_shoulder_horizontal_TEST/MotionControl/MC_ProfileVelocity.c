#include "MC_ProfileVelocity.h"

/**********************************************************
* @FuncName: PV_Init
* @Param: *PV_struct
* @Return: None
* @Brief: 初始化ProfileVelcity参数
* @addtion:
**********************************************************/
void PV_Init(PV_Typedef *PV_struct)
{
	memset(PV_struct, 0, sizeof(PV_Typedef));					//结构体初始化为0
	
	PV_struct->i_SD = 0;										//Stop Dec（暂时未用上）
	PV_struct->prm_SF = 0;										//平滑因子（暂时未用上）
	PV_struct->prm_vel_Ts = 0.001;								//速度采样周期：1ms
	PV_struct->pending_flag = 1;								//默认状态是：挂起状态
}

/**********************************************************
* @FuncName: PV_Set_Value
* @Param: *PV_struct, acc ,dec , vel_target
* @Return: None
* @Brief: 参数单位：cnt/s
* @addtion: acc: 不低于1000cnt/s^2
**********************************************************/
void PV_Set_Value(PV_Typedef *PV_struct, int32_t acc, int32_t dec, int32_t vel_start, int32_t vel_target)
{
	/* 加减速度限幅 */
	acc = acc > MAX_CTRL_ACC_CNT ? MAX_CTRL_ACC_CNT : acc;
	dec = dec > MAX_CTRL_ACC_CNT ? MAX_CTRL_ACC_CNT : dec;
	
	/* 速度限幅 */
	vel_target = vel_target >  MAX_CTRL_VEL_CNT ?  MAX_CTRL_VEL_CNT : vel_target;
	vel_target = vel_target < -MAX_CTRL_VEL_CNT ? -MAX_CTRL_VEL_CNT : vel_target;
	
	/* 输入参数更新 */
	PV_struct->i_acc = acc;
	PV_struct->i_dec = dec;
	PV_struct->i_vel_cur = vel_start;
	PV_struct->i_vel_target = vel_target;

	/* 计算中间变量 */
	PV_struct->delta_acc = PV_struct->i_acc * PV_struct->prm_vel_Ts;		//速度加速增量
	PV_struct->delta_dec = PV_struct->i_dec * PV_struct->prm_vel_Ts;		//速度减速增量
	
	PV_struct->pending_flag = 0;			//set接口将pending_flag置0，即取消PV挂起 -- 意味着使用PV模式	
	PV_struct->o_vel_ref = vel_start;		//参考速度规划起始点
}

/**********************************************************
* @FuncName: PV_Input
* @Param: *PV_struct, vel_fdbk
* @Return: None
* @Brief: 更新PV的反馈(速度),单位：cnt/s
* @addtion:
**********************************************************/
void PV_Input(PV_Typedef *PV_struct, int32_t vel_fdbk)
{
	PV_struct->i_vel_fdbk = vel_fdbk;
}

/**********************************************************
* @FuncName: PV_Clear_Output
* @Param: *PV_struct
* @Return: None
* @Brief: PV清除输出
* @addtion:
**********************************************************/
void PV_Clear_Output(PV_Typedef* PV_struct)
{
	PV_struct->o_vel_ref = 0;
	
	PV_struct->pending_flag = 1;			//清除后，回到挂起状态
}

/*******************************************************************************
* Function Name   : PV_Generator 相对速度生成器
* Description     : o_vel_ref与i_vel_target进行比较，然后加或者减，目标是将
*					o_vel_ref向i_vel_target靠近！
* @Input          : PV_Typedef*
* @Output         : o_vel_ref
* @Return         : o_vel_ref,单位：cnt/s
* @addtion		  : o_vel_ref与i_vel_target同方向并且abs(o_vel_ref)>abs(i_vel_target)
*					才是减速，其他情况均为加速!
*******************************************************************************/
int32_t PV_Generator(PV_Typedef* PV_struct)
{
	int32_t temp_vel =0;
	temp_vel = PV_struct->o_vel_ref;																			//获取全局变量
	
	if((PV_struct->i_vel_target >= 0) && (temp_vel > PV_struct->i_vel_target))
	{
		temp_vel -= PV_struct->delta_dec;																		//减速
		PV_struct->o_vel_ref = temp_vel < PV_struct->i_vel_target ? PV_struct->i_vel_target : temp_vel;			//限幅
	}
	else if((PV_struct->i_vel_target <= 0) && (temp_vel < PV_struct->i_vel_target))
	{
		temp_vel += PV_struct->delta_dec;																		//减速
		PV_struct->o_vel_ref = temp_vel > PV_struct->i_vel_target ? PV_struct->i_vel_target : temp_vel;			//限幅
	}
	else if(temp_vel != PV_struct->i_vel_target)																//加速
	{
		if(PV_struct->i_vel_target > 0)
		{
			temp_vel += PV_struct->delta_acc;
			PV_struct->o_vel_ref = temp_vel > PV_struct->i_vel_target ? PV_struct->i_vel_target : temp_vel;		//限幅
		}			
		else		//i_vel_target <0
		{
			temp_vel -= PV_struct->delta_acc;
			PV_struct->o_vel_ref = temp_vel < PV_struct->i_vel_target ? PV_struct->i_vel_target : temp_vel;		//限幅
		}
	}
	//else --> 保持速度
	
	return (PV_struct->o_vel_ref);
}


