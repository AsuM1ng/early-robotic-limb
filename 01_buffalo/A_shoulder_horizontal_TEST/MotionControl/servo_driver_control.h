
#ifndef _SERVO_DERVER_CONTROL_H__
#define _SERVO_DERVER_CONTROL_H__


#include "stdint.h"

// from main control board

/*******************************************************************************
                      controlword of profile position mode 
*******************************************************************************/
#define ABS_POS             0x001f              //  PPM的运行方式:绝对位置
#define ABS_POS_IMM         0x003f              //  PPM的运行方式:绝对位置,立即运行
#define REL_POS             0x005f              //  PPM的运行方式:相对位置
#define REL_POS_IMM         0x007f              //  PPM的运行方式:相对位置,立即运行

/*******************************************************************************
                             controlword of interpolated position mode
*******************************************************************************/
#define ENABLE_IP_MODE      0x0010       
/*******************************************************************************
                             controlword of profile velocity mode
*******************************************************************************/
#define PVM_EXEC             0x00              //  PVMODE的运行方式(runMode):执行PV运动
/*******************************************************************************
                             controlword of profile torque mode
*******************************************************************************/
#define PTM_EXEC             0x00              //  PTMODE的运行方式(runMode):执行PT运动
/*******************************************************************************
                             controlword of unprofile position mode
*******************************************************************************/
#define NPM_EXEC             0x00              //  NPMODE的运行方式(runMode):执行NP运动



/*******************************************************************************
                             modes_of_operation
*******************************************************************************/
//#define NPM             0x00              //  profile position mode:平滑位置模式，也即点到点运动（PTP：point to point）
#define PPM             0x01              //  profile position mode:平滑位置模式，也即点到点运动（PTP：point to point）
//#define PVM         	0x03              //  profile velocity mode:平滑速度模式
//#define PTM             0x04              //  profile torque mode:平滑力矩模式
#define IPM             0x07              //  interpolated position mode:插值位置模式
#define PVT         	IPM               //  因为IPM模式每一个点都需要提供位置(position)速度(velocity)时间(time)三个信息，所以插值位置模式又俗称PVT模式


//extern volatile int8_t mode_to_driver;
extern volatile uint16_t controlword_to_driver;

extern volatile int sync_motion_flag;
extern volatile int ppm_before_pvt_flag;

extern volatile int sync_motion_stop_flag;
extern volatile int sync_motion_stop_flag_previous;


#endif
