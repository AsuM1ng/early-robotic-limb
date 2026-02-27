/********************************************************************************************
* @FileName: MC_Top.h
* @Date: 2017-08-05
* @Version: v2.0
* @Author: Yifeng Han
* @Summary: 总体架构：将PID位置控制器和PID速度控制器看作基本伺服器，而PTP,PV,PT,PVT看成轨迹规划器.
* 			具体参照word文档: "MC模块详解"
********************************************************************************************/


#ifndef _MC_TOP_H
#define _MC_TOP_H

#include "MC_PositionTime.h"
#include "MC_ProfilePosition.h"
//#include "MC_ProfileVelocity.h"
#include "MC_ProfileTorque.h"
#include "MC_PIDPosition.h"
#include "MC_PIDVelocity.h"
#include "data.h"
#include "motion_queue.h"
#include "MC_PositionVelocityTime.h"
//#define PTP_OUT_POSREF				//PTP生成pos_ref给位置闭环；如果未定义，则PTP生成vel_ref给速度闭环.

/* 伺服环开关 */ /* 与模式匹配着用 */
#define TORQLOOP_MASK		((uint32_t)0x00000001)			//力矩环
#define VELLOOP_MASK		((uint32_t)0x00000002)			//速度PID -- PI
#define POSLOOP_MASK		((uint32_t)0x00000004)			//位置PID -- P
/*******************************************************************************
                             controlword
*******************************************************************************/
//#define ABS_POS             0x1f              //  PPMODE的运行方式(runMode):绝对位置
//#define ABS_POS_IMM         0x3f              //  PPMODE的运行方式(runMode):绝对位置,立即运行
//#define REL_POS             0x5f              //  PPMODE的运行方式(runMode):相对位置
//#define REL_POS_IMM         0x7f              //  PPMODE的运行方式(runMode):相对位置,立即运行
//#define PVM_EXEC            0x00              //  PVMODE的运行方式(runMode):执行PV运动
//#define PTM_EXEC            0x00              //  PTMODE的运行方式(runMode):执行PT运动
//#define NPM_EXEC            0x00              //  PTMODE的运行方式(runMode):执行PT运动


/************* MainMode of MC *************/
/*
typedef enum
{
	POSITION_MODE=0,
	VELOCITY_MODE,
	TORQUE_MODE

}MC_BaseMode_Typedef;
*/


/************* Mode of MC *************/
typedef enum
{
	/******* Position Mode *******/
	POS_NP_MODE=0,						//No Profile Position Mode -- 位置环模式
	POS_PTP_MODE=1,						//Point to Point Mode
	POS_PT_MODE,						//Position Time Mode
	POS_PVT_MODE = 7,						//Position Velocity Time Mode
	
	/******* Speed Mode *******/
	VEL_NV_MODE = 10,					//No Profile Velocity Mode -- 速度环模式
	VEL_PV_MODE = 3,						//Profile Velocity Mode
	
	/******* Torque Mode *******/
//	TRQ_NDNQ = 20,						//D轴No Profile, Q轴NO Profile
//	TRQ_NDPQ,							//D轴No Profile, Q轴Profile
//	TRQ_PDNQ,							//D轴Profile, 	 Q轴NO Profile
	TRQ_PDPQ = 4							//D轴Profile, 	 Q轴Profile
}MC_Mode_Typedef;

typedef struct
{
	/******* MC Mode ******/
	char modes_of_operation;
	MC_Mode_Typedef			prm_MC_mode;				//Mode of Motion Control
	MC_Mode_Typedef			prm_MC_mode_display;		//canopen402 0x6061 modes of operation display
	
	/*********** Input ***********//* 公用的参考值,意思是子模块PTP,PT,PV等都可以用. */
	int32_t					i_pos_target;				//目标位置,单位：cnt
	int32_t					i_vel_target;				//目标速度,单位：cnt 
	int32_t					i_trq_target;				//目标力矩,单位：Q15(A),q轴(用于控制)
    int16_t                 target_torque;              //目标力矩,对象0x6071的影子,单位：额定电流的千分之一(用于显示)
//	int32_t					i_flux_target;				//目标力矩,单位：Q15(A),d轴
	
	/*********** Sensor info ***********/
	int32_t					i_pos_fdbk;					//反馈位置,单位：cnt
	int32_t					i_vel_fdbk;					//反馈速度,单位：cnt/s
	int32_t 				pos_err;					//位置误差,单位: cnt
	int32_t					vel_err;					//速度误差,单位: cnt/s

	/* 轨迹生成器 */
	PTP_Typedef 			PTP_struct;
	PrflTrq_Typedef			PrflTrq_struct;				//Profile Torque mode
	PVT_Typedef             PVT_struct;

	/*********** Output ***********/
	uint32_t following_error_window;//对象0x6065
	uint16_t following_error_timeout;//对象0x6066
	uint32_t position_window;//对象0x6067
	uint16_t position_window_time;//对象0x6068
	int32_t position_demand_value;//对象0x60FC

    int16_t torque_demand_value;
	TIMER_HANDLE target_position_reached_timer;
}MC_Typedef;


/*
extern MotionQueue ptp_motion_queue;
extern Motion ptp_motion;

void MC_Init(MC_Typedef *MC_struct, MC_Mode_Typedef mc_mode);
void setMonitorWindow(MC_Typedef *MC_struct);
void MC_Reset(MC_Typedef *MC_struct, int32_t pos, int32_t vel, int32_t trq, int32_t flux);
void MC_clearInput(MC_Typedef *MC_struct);
void MC_Clear_Output(MC_Typedef *MC_struct);
void MC_Set_Mode(MC_Typedef *MC_struct, MC_Mode_Typedef mc_mode);
*/


/* 位置接口 */
/*
void MC_PPM_Set_Value(MC_Typedef *MC_struct, int32_t acc, int32_t dec, int32_t vel_limit,
							int32_t pos_start, int32_t pos_end);
void MC_NP_Set_Value(MC_Typedef *MC_struct, int32_t pos_target);
*/


/* 速度接口 */
/*
void MC_NV_Set_Value(MC_Typedef *MC_struct, int32_t vel_target);
void MC_PVM_Set_Value(MC_Typedef *MC_struct, int32_t acc, int32_t dec, int32_t vel_target);
*/


/* 力矩接口 */
/*
void MC_PDNQ_Set_Value(MC_Typedef *MC_struct, float flux, float torque, int32_t t_ms);		//Profile Torque Mode;
void MC_NDNQ_Set_Value(MC_Typedef *MC_struct, float flux, float torque);		//Torque Mode
int8_t MC_PrflTrq_Get_ArvFluxFlag(MC_Typedef *MC_struct);
int8_t MC_PrflTrq_Get_ArvTrqFlag(MC_Typedef *MC_struct);
void MC_PrflTrq_Clear_ArvFluxFlag(MC_Typedef *MC_struct);
void MC_PrflTrq_Clear_ArvTrqFlag(MC_Typedef *MC_struct);
int32_t MC_Get_i_pos_fdbk(MC_Typedef *MC_struct);
int32_t MC_Get_position_demand_value(MC_Typedef *MC_struct);
uint32_t MC_Get_following_error_window(MC_Typedef *MC_struct);
uint32_t MC_Get_position_window(MC_Typedef *MC_struct);
*/

/* 周期函数 */
//void MC_Input(MC_Typedef *MC_struct, int32_t pos_fdbk, int32_t vel_fdbk);
void MC_PeriodFunc(MC_Typedef *MC_struct);
//void MC_Start_PVT(MC_Typedef *MC_struct);
//void MC_Stop_PVT(MC_Typedef *MC_struct);


int8_t MC_getPTPPendingflag(MC_Typedef *mc);
#endif


