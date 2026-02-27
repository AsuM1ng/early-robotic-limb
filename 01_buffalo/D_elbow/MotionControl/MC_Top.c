/* 修改人： Yifeng Han
	修改内容: 将位置伺服的单位修改为cnt，即仅考虑增量编码器位置伺服的情况.设计到P位置控制器，PTP位置控制器.*/

/* MC模块单位制定：
	凡是位置单位，无特别说明单位均为cnt。 
	凡是速度单位，无特别说明单位均为cnt/s,
	注意：速度PID(PID_Velocity)使用step/s；
		  力矩使用step/s。*/
		  
/* MC模块的架构见.h文件 */
#include "MC_Top.h"
#include "global.h"
#include "TestSlave.h"
#include "states.h"
#include "motion_queue.h"
#include "stm32f4xx_it.h"
#include "device_control.h"
#include "factor_group.h"
#include "homing_mode.h"
#include "interpolated_position_mode.h"
#include "position_control_function.h"
#include "profile_position_mode.h"
#include "unprofile_position_mode.h"
#include "profile_velocity_mode.h"
#include "profile_torque_mode.h"
#include "mcProfileTorque.h"


static uint32_t MC_loop_sw = 0;					//三环均不开

int gpvt_tick = 0;

/**********************************************************
* @FuncName: MC_Init
* @Param: *MC_struct
* @Return: None
* @Brief: MotionControl初始化
* @addtion:
**********************************************************/
void MC_Init(MC_Typedef *MC_struct, MC_Mode_Typedef mc_mode)
{
	MC_struct->prm_MC_mode = mc_mode;
	MC_struct->prm_MC_mode_display = MC_struct->prm_MC_mode;
	
	PTP_Init(&MC_struct->PTP_struct, 0.001);		//PTP初始化,采样周期:0.001s
	PV_Init(&MC_struct->PV_struct);					//PV初始化
    PVT_Init(&MC_struct->PVT_struct, 0.001,0);	//Profile Torque 模式初始化
	PIDPos_Init(&MC_struct->PID_pos_struct);		//PIDPos初始化
	PIDVel_Init(&MC_struct->PID_vel_struct);		//PIDVel初始化
	PrflTrq_Init(&MC_struct->PrflTrq_struct, 1000);	//Profile Torque 模式初始化
	
//	MC_struct->trq_limit = 4.0f*32768;
//	MC_struct->flux_limit = 4.0f*32768;
    
    MC_struct->trq_limit = MC_struct->PID_vel_struct.PID_struct.prm_Hlimit_output;
	MC_struct->flux_limit = MC_struct->PID_vel_struct.PID_struct.prm_Hlimit_output;
    
	MC_struct->o_flux_ref = 0;						
	MC_struct->o_torque_ref = 0;
	
}

/*position,velocity and torque monitoring window settings*/
void setMonitorWindow(MC_Typedef *MC_struct)
{
	MC_struct->position_window = 1000;
	MC_struct->position_window_time = 10;
	MC_struct->following_error_window = 20000;
	MC_struct->following_error_timeout = 1000;
    
    MC_struct->velocity_window = 1024;
	MC_struct->velocity_window_time = 10;    
    MC_struct->velocity_threshold = 512;
	MC_struct->velocity_threshold_time = 1000;
    
    MC_struct->torque_window = 10;
    MC_struct->torque_window_time = 10;    
}


//重置MC目标值
void MC_Reset(MC_Typedef *MC_struct, int32_t pos, int32_t vel, int32_t trq, int32_t flux)
{
	MC_struct->i_pos_target = pos;
	MC_struct->i_vel_target = vel;
	MC_struct->i_flux_target = flux;
	MC_struct->i_trq_target = trq;
}

/**
* Clear MC input
**/
void MC_clearInput(MC_Typedef *MC_struct)
{
	MC_struct->i_flux_target = 0;
	MC_struct->i_trq_target = 0;
	
	MC_struct->i_vel_target = 0;
	MC_struct->i_pos_target = 0;
}
/**********************************************************
* @FuncName: MC_Clear_Output
* @Param: *MC_struct
* @Return: None
* @Brief: 清除MC的输出
* @addtion:
**********************************************************/
void MC_Clear_Output(MC_Typedef *MC_struct)
{
	PIDPos_Clear_Output(&MC_struct->PID_pos_struct);
	PIDVel_Clear_Output(&MC_struct->PID_vel_struct);
	PTP_Clear_Output(&MC_struct->PTP_struct);
	PV_Clear_Output(&MC_struct->PV_struct);
	
	MC_struct->o_flux_ref = 0;
	MC_struct->o_torque_ref = 0;
}

//下面的接口得与设置模式配合使用
void MC_Set_Mode(MC_Typedef *MC_struct, MC_Mode_Typedef mc_mode)
{
	MC_struct->prm_MC_mode = mc_mode;
	MC_struct->prm_MC_mode_display = MC_struct->prm_MC_mode;
	
	/* Clear */
	PrflTrq_Clear_Output(&MC_struct->PrflTrq_struct);
//	MC_Clear_Output(MC_struct);			//暴力清零不可取
}
extern int ptp_link_pvt;
void MC_Start_PVT(MC_Typedef *MC_struct)
{
    
	MC_struct->PVT_struct.start = 1;/* MC_PeriodFunc()函数中使用 */
    statusword |= 1<<12;
    gMC.i_pos_target = gMC.PVT_struct.pvt_table_struct[actual_buffer_size].position;

}
/******************************************************************************/
/***********************			位置接口			***********************/
/******************************************************************************/
/**********************************************************
* @FuncName: MC_NP_Set_Value
* @Param: *MC_struct, pos_target
* @Return: None
* @Brief: PID_Position模式的异步输入(异步更新)
* @addtion: 单位：cnt,范围:-2^31 ~ 2^31-1,对于PPR=2000的编码器,
*			运动范围: -1073,741~1073,741 r
**********************************************************/
void MC_NP_Set_Value(MC_Typedef *MC_struct, int32_t pos_target)
{
	MC_struct->i_pos_target = pos_target;		
}
/**********************************************************
* @FuncName: MC_PPM_Set_Value
* @Param: *MC_struct, acc, dec, vel_limit, pos_start, pos_end
* @Return: None
* @Brief: 更新MC_PPM的的输入(异步更新)
* @addtion: 输入量的单位：cnt
**********************************************************/
void MC_PPM_Set_Value(MC_Typedef *MC_struct, int32_t acc, int32_t dec, int32_t vel_limit,
							int32_t pos_start, int32_t pos_end)
{
//	MC_Set_Mode(MC_struct, POS_PTP_MODE);
	MC_struct->i_pos_target = pos_end;
	
	PTP_Set_value(&MC_struct->PTP_struct, acc, dec, vel_limit, pos_start, pos_end);
}


/******************************************************************************/
/***********************			速度接口			***********************/
/******************************************************************************/
/**********************************************************
* @FuncName: MC_NV_Set_Value
* @Param: *MC_struct, vel_target
* @Return: None
* @Brief: PID_Velocity模式的异步输入(异步更新)
* @addtion: 单位：cnt/s,范围:-2^31 ~ 2^31-1,对于PPR=2000的编码器,
*			速度范围: -1073,741Hz~1073,741Hz
**********************************************************/
void MC_NV_Set_Value(MC_Typedef *MC_struct, int32_t vel_target)
{
	MC_struct->i_vel_target = vel_target;	
}

/**********************************************************
* @FuncName: MC_PVM_Set_Value
* @Param: *MC_struct, acc, dec, vel_target
* @Return: None
* @Brief: 更新MC_PVM的异步输入(异步更新)
* @addtion: 单位：cnt/s
**********************************************************/

void MC_PVM_Set_Value(MC_Typedef *MC_struct, int32_t acc, int32_t dec, int32_t vel_target)
{
	MC_struct->i_vel_target = vel_target;
	
	PV_Set_Value(&MC_struct->PV_struct, acc, dec, MC_struct->i_vel_fdbk, vel_target);
}


/******************************************************************************/
/***********************			力矩接口			***********************/
/******************************************************************************/
//Profile Flux No profile Trq //单位Q15(Nm)
void MC_PDNQ_Set_Value(MC_Typedef *MC_struct, float flux, float torque, int32_t t_ms)		//Profile Torque Mode
{
	MC_struct->i_flux_target = flux;
	MC_struct->i_trq_target = torque;
	
	PrfFlux_Set_Value(&MC_struct->PrflTrq_struct, flux, torque, t_ms);
	
}

/**********************************************************
* @FuncName: MC_NDNQ_Set_Value
* @Param: *MC_struct, acc, dec, vel_limit, pos_start, pos_end
* @Return: None
* @Brief: 更新MC_PTM的的输入(异步更新)
* @addtion: 输入量的单位：Q15(A),后边会改成Nm.
**********************************************************/
void MC_NDNQ_Set_Value(MC_Typedef *MC_struct, float flux, float torque)
{
	MC_struct->i_flux_target = flux;
	MC_struct->i_trq_target = torque;
}

/* 标志位set get接口 */
int8_t MC_PrflTrq_Get_ArvFluxFlag(MC_Typedef *MC_struct)
{
	return PrflTrq_Get_ArvFluxFlag(&MC_struct->PrflTrq_struct);
}
int8_t MC_PrflTrq_Get_ArvTrqFlag(MC_Typedef *MC_struct)
{
	return PrflTrq_Get_ArvTrqFlag(&MC_struct->PrflTrq_struct);
}

void MC_PrflTrq_Clear_ArvFluxFlag(MC_Typedef *MC_struct)
{
	PrflTrq_Clear_ArvFluxFlag(&MC_struct->PrflTrq_struct);
}
void MC_PrflTrq_Clear_ArvTrqFlag(MC_Typedef *MC_struct)
{
	PrflTrq_Clear_ArvTrqFlag(&MC_struct->PrflTrq_struct);
}



/**********************************************************
* @FuncName: MC_Update_Feedback
* @Param: *MC_struct, pos_fdbk, spd_fdbk
* @Return: None
* @Brief: 更新MC反馈信息
* @addtion: pos_fdbk和spd_fdbk均以cnt为单位。
**********************************************************/
void MC_Input(MC_Typedef *MC_struct, int32_t pos_fdbk, int32_t vel_fdbk)
{
	MC_struct->i_pos_fdbk = pos_fdbk;
	MC_struct->i_vel_fdbk = vel_fdbk;
}
int32_t MC_Get_i_pos_fdbk(MC_Typedef *MC_struct)
{
	return MC_struct->i_pos_fdbk;
}
int32_t MC_Get_position_demand_value(MC_Typedef *MC_struct)
{
	return MC_struct->position_demand_value;
}
uint32_t MC_Get_following_error_window(MC_Typedef *MC_struct)
{
	return MC_struct->following_error_window;
}
uint32_t MC_Get_position_window(MC_Typedef *MC_struct)
{
	return MC_struct->position_window;
}
/**********************************************************
* @FuncName: MC_PeriodFunc
* @Param: *MC_struct
* @Return: None
* @Brief: MC模块周期功能函数
* @addtion: 该函数要配合反馈信息更新同步使用！
**********************************************************/
void MC_PeriodFunc(MC_Typedef *MC_struct)
{		
	
		PVT_Generator(&MC_struct->PVT_struct);

}

int8_t MC_getPTPPendingflag(MC_Typedef *mc)
{
	return (mc->PTP_struct.pending_flag);
}

