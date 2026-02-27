/********************************************************************************************
* @FileName: global.h
* @Date: 2017_3_2
* @Version: v1.1
* @Author: Yifeng Han
* @Summary: global.h相当于 main.h，里面包含了所有模块的顶层文件和程序中用到的全局变量；
*			global.h仅仅可以被main.c和stm32f4xx_it.h等耦合层包含！
********************************************************************************************/

/********************************************************************************************
* 		   						Multi-Include-Prevent Section                 
********************************************************************************************/
#ifndef _GLOBAL_H
#define _GLOBAL_H

/********************************************************************************************
* 		   						Debug Switch Section                 
********************************************************************************************/
//#define DEBUG_SW1
//#define DEBUG_SW2

/********************************************************************************************
* 		   						Include File Section                 
********************************************************************************************/

#include "Umath.h"
//#include "MC_Top.h"

//#include "ST_Top.h"
#include "boardHAL.h"

//#include "Global_Config.h"
//#include "JointMotionControl.h"
//#include "Joint_Param.h"
//#include "Motor_Param.h"
//#include "Sensor_Param.h"
//#include "PID_Regulator.h"
//#include "Task.h"
//#include "Ident_Inertia_FRLS.h"
//#include "MVC_Top.h"

//extern MC_Typedef 				gMC;
extern int g_node_id;
extern int g_joint_id;

/********************************************************************************************
* 		   						Macro Define Section                 
********************************************************************************************/
//Nothing
/********************************************************************************************
* 		   						Struct Define Section                 
********************************************************************************************/
/***************** System FSM status *****************/
/*
typedef enum
{
	INIT = 0,			//上电初始化
	CALIB_CURR,			///< Calibrate phase current offset.
	IDLE,				//初始化后
	CALIBRATION,		//标定.
	CLEAR,				//清除数据：PID，MVC.
	SERVO_ENABLE,		//启动伺服环,和位置传感器.
	START,				//电机启动：霍尔不需要启动，编码器需要对齐.
	SERVO,				//旋转状态.
	IDENT,				//辨识状态
	TEST,				///< test servo loop.
	STOP,				//停止状态，也叫紧急制动状态.
	FAULT
}SysState_ET;
*/

/* 错误类型 */
/*
#define FC_NOERR 				((uint32_t)0x00000000)
#define FC_OVERVOLTAGE 			((uint32_t)0x00000001)
#define FC_UNDERVOLTAGE 		((uint32_t)0x00000002)
#define FC_OVERCURRENT 			((uint32_t)0x00000004)
#define FC_OVERTEMP 			((uint32_t)0x00000008)
#define FC_FDBK 				((uint32_t)0x00000010)
*/

/******************** SW *******************/
/*
typedef struct
{
	int8_t force_cal;		//标定开始开关。
	int8_t start;			//启动开关。
	int8_t sw_mode;			//运行模式
	int8_t clear_fault;		//清除错误
}SysSW_Typedef;
*/

/********************************************************************************************
* 		   						Prototype Declare Section                 
********************************************************************************************/
//extern SysState_ET gSys_state;
//extern SysSW_Typedef gSys_ctrl;
//extern uint32_t gsys_fault_code;

//extern uint16_t hTimebase_100us;

//extern MotionFloat_Typedef 	gmotion[16];

//extern int16_t		gServo_tick;	
//extern int16_t		gServo_IncEnc_enable;		//伺服环传感器开启标志位
//extern int16_t		gServo_Hall_enable ;		//速度环hall开启标志位
//extern int16_t		gServo_enable ;				//伺服Generator标志位
//extern int16_t 		gCurloop_enable;			//电流环开启标志位

//extern MVC_Typedef 				gMVC_struct;			//MVC模块全局结构体

//角度模拟
//extern SinGen_Typedef gSinGen;

//extern BHAL_Typedef				gBHAL;	//Board Hardware Abstract Layer module.


/* MC模块调试 */
//extern float 			gSinFrq, gSinAmp;					//sinGen test
//extern int32_t 			gpos_target, gvel_target, gtrq_d, gtrq_q;
//extern int32_t 			gAcc, gDec;
//extern INTEGER32 			gVel;
//extern int32_t 			gVelLimit;
//extern MC_Mode_Typedef 	gMCMode;					//修改MC控制模式

//extern Timer_Typedef gtimer;		///< software timer.

#endif	//end define _GLOBAL_H
