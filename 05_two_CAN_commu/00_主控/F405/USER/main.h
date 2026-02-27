#ifndef _MAIN_H
#define	_MAIN_H

#include "init.h"
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
#include "timer3.h"
#include "data.h"
#include "Master.h"
#include "Master2.h"
#include "Slaver.h"
#include "canfestival.h"
#include "master_control_word.h"
#include "key_drive_task.h"

/**********全局变量定义********/

typedef struct
{
	uint8_t key0_flag;			
	uint8_t key1_flag;
	uint8_t wkup_flag;
} PressFlag;
extern PressFlag pressf;

//void (*can1RxCallback)(CAN_RxHeaderTypeDef*, uint8_t*);
//void (*can2RxCallback)(CAN_RxHeaderTypeDef*, uint8_t*);
#endif


