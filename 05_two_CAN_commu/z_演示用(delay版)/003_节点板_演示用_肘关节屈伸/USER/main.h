#ifndef _MAIN_H
#define	_MAIN_H

#include "sys.h"
#include "delay.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "led.h"
#include "key.h"


/*****CANopen 相关头文件********/

#include "dual_can.h"
#include "timer2.h"
#include "timer7_action_timing.h"
#include "data.h"
#include "Master.h"
#include "Master2.h"
#include "Slaver.h"
#include "canfestival.h"

/********  EPOS/电机相关  ********/

#include "EPOS_control_word.h"
#include "motor_control.h"

/********  接口文件  ********/

#include "sys_init.h"

/**********  数学工具  ********/

#include "mathtool.h"

typedef struct
{
	u8 ready;				//准备动作标志
	u8 action;			//运行动作标志
	u8 halt;				//暂停所有动作标志
	u8 zero;				//移动到位置0动作标志
} ActionFlag;

extern ActionFlag actionf;

typedef struct
{
	u8 ready_state;			//准备动作状态，置1进行中，置0当前不在这个状态
	u8 action_state;		//做动作状态
	u8 action_times;		//做动作次数
	u8 hlat_state;			//暂停状态
	u8 hlat_time_state;	//暂停计时状态
	u8 zero_state;			//回到0点动作状态
} MissionProcess;

extern MissionProcess missionp;




//void (*can1RxCallback)(CAN_RxHeaderTypeDef*, uint8_t*);
//void (*can2RxCallback)(CAN_RxHeaderTypeDef*, uint8_t*);
#endif


