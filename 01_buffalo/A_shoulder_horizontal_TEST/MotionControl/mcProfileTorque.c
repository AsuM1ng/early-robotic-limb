#include "mcProfileTorque.h"
#include "MC_ProfileTorque.h"
/**********************************************************
* @FuncName: PrflTrq_Init
* @Param:   *PrflTrq_Typedef
* @Return:   None
* @Brief:    初始化ProfileTorque参数
* @addtion:
**********************************************************/
void profileFluxAndTorqueInit(PrflTrq_Typedef *PrflTrq_struct, int32_t velloop_frq)
{
	memset(PrflTrq_struct, 0, sizeof(PrflTrq_Typedef));					//结构体初始化为0
	PrflTrq_struct->prm_velloop_frq = velloop_frq;
    
	PrflTrq_struct->profile_torque_pending_flag = 1;								//默认状态是：挂起状态
}

///**********************************************************
//* @FuncName: PV_Set_Value
//* @Param: *PV_struct, acc ,dec , vel_target
//* @Return: None
//* @Brief: 参数单位：cnt/s
//* @addtion: acc: 不低于1000cnt/s^2
//**********************************************************/
//void PrfFlux_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_ms)
//{
//	that->i_flux_target = flux;
//	that->i_trq_target = trq;
//	
//	that->m_flux_inc = flux / (t_ms * that->prm_velloop_frq / 1000);	//每次周期函数的力矩递增量
//	that->m_trq_inc = 0;
//	
//	that->profile_torque_pending_flag = 0;			//set接口将pending_flag置0，即取消PV挂起 -- 意味着使用PV模式	
//}



//Profile Flux Torque Mode: Flux is profiled, trq is profiled.
void profileFluxAndTorqueSetValue(PrflTrq_Typedef *that ,int32_t flux, int32_t trq, int32_t flux_inc, int32_t trq_inc)
{
	that->i_flux_target = flux;
	that->i_trq_target = trq;
	
	that->m_flux_inc = flux_inc;	//每次周期函数的力矩递增量,有正负
	that->m_trq_inc = trq_inc;		//每次周期函数的力矩递增量,有正负
	
	/* 力矩到达标志位置零 */
	that->o_trq_arv_flag = 0;
	that->o_flux_arv_flag = 0;
    
    that->profile_torque_pending_flag = 0;			//set接口将pending_flag置0，即取消PV挂起 -- 意味着使用PV模式
}
/**********************************************************
* @FuncName: PV_Input
* @Param: *PV_struct, vel_fdbk
* @Return: None
* @Brief: 更新PV的反馈(速度),单位：cnt/s
* @addtion:
**********************************************************/
void profileFluxAndTorqueInput(PrflTrq_Typedef *that, int32_t flux_fdbk, int32_t torque_fdbk)
{
	that->i_flux_fdbk = flux_fdbk;
  	that->i_torque_fdbk = torque_fdbk;
}

/**********************************************************
* @FuncName: PV_Clear_Output
* @Param: *PV_struct
* @Return: None
* @Brief: PV清除输出
* @addtion:
**********************************************************/
void profileFluxAndTorqueClearOutput(PrflTrq_Typedef* that)
{
	that->o_flux_ref = 0;
	that->o_trq_ref = 0;
    
    that->profile_torque_pending_flag = 1;//清除后，回到挂起状态
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
int32_t profileFluxAndTorqueGenerator(PrflTrq_Typedef* that)
{
	int32_t temp_trq =0;
    int32_t temp_flux =0;
    
	temp_trq = that->o_trq_ref;	
    temp_flux = that->o_flux_ref;	   

    /*torque generator*/
	if((that->i_trq_target >= 0) && (temp_trq > that->i_trq_target))
	{
		temp_trq -= that->m_trq_inc;																//减速
		that->o_trq_ref = temp_trq < that->i_trq_target ? that->i_trq_target : temp_trq;			//限幅
	}
	else if((that->i_trq_target <= 0) && (temp_trq < that->i_trq_target))
	{
		temp_trq += that->m_trq_inc;																//减速
		that->o_trq_ref = temp_trq > that->i_trq_target ? that->i_trq_target : temp_trq;			//限幅
	}
	else if(temp_trq != that->i_trq_target)															//加速
	{
		if(that->i_trq_target > 0)
		{
			temp_trq += that->m_trq_inc;
			that->o_trq_ref = temp_trq > that->i_trq_target ? that->i_trq_target : temp_trq;		//限幅
		}
		else//i_vel_target <0
		{
			temp_trq -= that->m_trq_inc;
			that->o_trq_ref = temp_trq < that->i_trq_target ? that->i_trq_target : temp_trq;		//限幅
		}
	}
	//else --> 保持速度
    
    /*flux generator*/   
	if((that->i_flux_target >= 0) && (temp_flux > that->i_flux_target))
	{
		temp_flux -= that->m_flux_inc;																//减速
		that->o_flux_ref = temp_flux < that->i_flux_target ? that->i_flux_target : temp_flux;			//限幅
	}
	else if((that->i_flux_target <= 0) && (temp_flux < that->i_flux_target))
	{
		temp_flux += that->m_flux_inc;																//减速
		that->o_flux_ref = temp_flux > that->i_flux_target ? that->i_flux_target : temp_flux;			//限幅
	}
	else if(temp_flux != that->i_flux_target)															//加速
	{
		if(that->i_flux_target > 0)
		{
			temp_flux += that->m_flux_inc;
			that->o_flux_ref = temp_flux > that->i_flux_target ? that->i_flux_target : temp_flux;		//限幅
		}			
		else//i_vel_target <0
		{
			temp_flux -= that->m_flux_inc;
			that->o_flux_ref = temp_flux < that->i_flux_target ? that->i_flux_target : temp_flux;		//限幅
		}
	}
	//else --> 保持速度    
	
	return (that->o_trq_ref);
}
