/* MC_ProfileTorque: 包含NDNQ, NDPQ, PDNQ, PDPQ四种模式的接口，供MC_TOP进一步封装 */
/* 力矩规划模式现在比较简单，仅仅用到了NDNQ和PDNQ两种模式，并且PDNQ模式仅支持从0力矩
	规划到正力矩.*/
#include "MC_ProfileTorque.h"

void PrflTrq_Init(PrflTrq_Typedef *that , int32_t velloop_frq)		//velloop_frq = 1000Hz
{
	that->prm_velloop_frq = velloop_frq;
}

//Profile Flux Mode: trq = constant value, flux is profiled.
void PrfFlux_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_ms)
{
	that->i_flux_target = flux;
	that->i_trq_target = trq;
	
	that->m_flux_inc = flux / (t_ms * that->prm_velloop_frq / 1000);	//每次周期函数的力矩递增量
	that->m_trq_inc = 0;
}
////Profile Torque Mode: flux = constant value, trq is profiled.
//void PrflTrq_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_ms)
//{
//	that->i_flux_target = flux;
//	that->i_trq_target = trq;
//	that->i_flux_ms = t_ms;
//	
//	that->m_flux_inc = flux / (t_ms * that->prm_velloop_frq / 1000);	//每次周期函数的力矩递增量
//	that->m_trq_inc = trq / (t_ms * that->prm_velloop_frq / 1000);		//每次周期函数的力矩递增量
//	
//	/* 力矩到达标志位置零 */
//	that->o_trq_arv_flag = 0;
//	that->o_flux_arv_flag = 0;
//}

////Profile Flux Torque Mode: Flux is profiled, trq is profiled.
//void PrfFluxTorque_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_flux_ms, int32_t t_trq_ms)
//{
//	that->i_flux_target = flux;
//	that->i_trq_target = trq;
//	that->i_flux_ms = t_flux_ms;
//	that->i_trq_ms = t_trq_ms;
//	
//	that->m_flux_inc = flux / (t_flux_ms * that->prm_velloop_frq / 1000);	//每次周期函数的力矩递增量,有正负
//	that->m_trq_inc = trq / (t_trq_ms * that->prm_velloop_frq / 1000);		//每次周期函数的力矩递增量,有正负
//	
//	/* 力矩到达标志位置零 */
//	that->o_trq_arv_flag = 0;
//	that->o_flux_arv_flag = 0;
//}

void PrfFlux_Velloop(PrflTrq_Typedef *that)
{
	that->o_flux_ref += that->m_flux_inc;
	that->o_trq_ref = 0;
	/* 输出力矩不能超过输入目标力矩 */
	if(that->o_flux_ref >= that->i_flux_target)
	{
		that->o_flux_ref = that->i_flux_target;
		that->o_flux_arv_flag = 1;	//力矩到达
	}
}


//void PrflTrq_Gen_Velloop(PrflTrq_Typedef *that)
//{
//	that->o_flux += that->m_flux_inc;
//	that->o_trq += that->m_trq_inc;
//	if(that->i_flux_target > 0)
//	{
//		/* 输出力矩不能超过输入目标力矩 */
//		if(that->o_flux >= that->i_flux_target)
//		{
//			that->o_flux = that->i_flux_target;
//			that->o_flux_arv_flag = 1;	//力矩到达
//		}
//	}
//	if(that->o_trq >= that->i_trq_target)
//	{
//		that->o_trq = that->i_trq_target;
//		that->o_trq_arv_flag = 1;	//力矩到达
//	}
//}

void PrflTrq_Clear_Output(PrflTrq_Typedef *that)
{
	that->o_flux_ref = 0;
//	that->o_trq_ref = 0;
}

int8_t PrflTrq_Get_ArvFluxFlag(PrflTrq_Typedef *that)
{
	return that->o_flux_arv_flag;
}

int8_t PrflTrq_Get_ArvTrqFlag(PrflTrq_Typedef *that)
{
	return that->o_trq_arv_flag;
}

void PrflTrq_Clear_ArvFluxFlag(PrflTrq_Typedef *that)
{
	that->o_flux_arv_flag = 0;
}

void PrflTrq_Clear_ArvTrqFlag(PrflTrq_Typedef *that)
{
	that->o_trq_arv_flag = 0;
}


