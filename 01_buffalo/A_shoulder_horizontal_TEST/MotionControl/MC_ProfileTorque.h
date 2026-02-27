#ifndef _MC_PROFILE_TORQUE_H
#define _MC_PROFILE_TORQUE_H

#include "tool.h"
typedef struct _PRFL_TRQ
{
//	int32_t i_trq_target;//Q15格式
//	int32_t i_flux_target;//Q15格式
    
//    int32_t i_flux_fdbk;//Q15格式
//    int32_t i_torque_fdbk;//Q15格式
    
//	int32_t i_flux_ms;			//flux到达时间
//	int32_t i_trq_ms;			//trq到达时间
    
    /*CANopen 402 -- profile torque mode*/
    int16_t target_torque;//6071 单位:额定转矩千分之一
    int16_t max_torque;//6072 单位:额定转矩千分之一
    int16_t max_current;//6073 单位:额定电流千分之一
    int16_t torque_demand_value;//6074 转矩控制器的输出，单位:额定转矩千分之一
    uint32_t motor_rate_current;//6075 单位:mA
    uint32_t motor_rate_torque;//6076 单位:mNm
    int16_t torque_actual_value;//6077 单位:额定转矩千分之一
    int16_t current_actual_value;//6078 单位:额定电流千分之一
    uint32_t torque_slope;//6087 单位:额定转矩千分之一每秒
    int16_t torque_profile_type;//6088
		
//	int32_t prm_velloop_frq;	//PrflTrq速度环频率
//	int32_t m_trq_inc;
//	int32_t m_flux_inc;
//	
//	int32_t o_trq_ref;
//	int32_t o_flux_ref;
//	int8_t o_trq_arv_flag;	//trq arrive flag
//	int8_t o_flux_arv_flag;	//trq arrive flag
//    int8_t profile_torque_pending_flag;
}PrflTrq_Typedef;

//void PrflTrq_Init(PrflTrq_Typedef *that , int32_t velloop_frq);		//velloop_frq = 1000Hz
//void PrflTrq_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_ms);
//void PrflTrq_Gen_Velloop(PrflTrq_Typedef *that);
////Profiled Flux Mode
//void PrfFlux_Set_Value(PrflTrq_Typedef *that ,float flux, float trq, int32_t t_ms);
//void PrfFlux_Velloop(PrflTrq_Typedef *that);

//void PrflTrq_Clear_Output(PrflTrq_Typedef *that);

//int8_t PrflTrq_Get_ArvFluxFlag(PrflTrq_Typedef *that);
//int8_t PrflTrq_Get_ArvTrqFlag(PrflTrq_Typedef *that);
//void PrflTrq_Clear_ArvFluxFlag(PrflTrq_Typedef *that);
//void PrflTrq_Clear_ArvTrqFlag(PrflTrq_Typedef *that);
#endif


