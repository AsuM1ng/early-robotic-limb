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
#include "key_test.h"

/********  EPOS/电机相关  ********/

#include "EPOS_control_word.h"
#include "motor_control.h"
#include "Para_monitor.h"

/********  接口文件  ********/

#include "sys_init.h"

/**********  数学工具  ********/

#include "mathtool.h"

#endif


