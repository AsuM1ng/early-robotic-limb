/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>

#include "stm32f4xx_it.h"
#include "chipHAL_TIM.h"
#include "chipHAL_GPIO.h"
#include "servo_driver_control.h"
#include "global.h"

#include "TestSlave.h"
#include "read_write_SDO.h"
#include "servo_Control.h"
#include "JointControl.h"
#include "BreakControl.h"
#include "MotionPathProcess.h"
#include "can_data_send.h"

#include "AngleSensor.h"


/** @addtogroup Template_Project
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define FIVE_SEC     5000
#define LEN_I		15
#define LEN_W		15
int32_t		QueueI[LEN_I] = {0};
int32_t		QueueWm[LEN_W] = {0};

/* JScope */
#pragma pack(push,1)			//内存对齐方式
struct _Send2Jscope
{
	volatile char watch_count;		//SSI时钟
}gs2j_buf;
#pragma pack(pop)


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

volatile int current_limit_control_counter = 0;

void TIM_PvtIRQHandler(void)
{
	if (TIM_GetITStatus(TIM_PVT, TIM_IT_Update) != RESET)			///////////////modify
	{

        TIM_ClearITPendingBit(TIM_PVT, TIM_IT_Update);				///////////////modify
	}
}




// process 

#define T200MICROSECOND 20
#define TENMILLISECOND  1000
#define ONESECOND       100000
#define TENSECOND       1000000
_Bool LEDLevel  = FALSE;

/*TIM 中断周期为1mS*/
void TIM_GeneralTimerIRQHandler(void)
{
//    static uint8_t ProcessDiv = 0;
    
	if (TIM_GetITStatus(TIM_GENERAL_TIMER, TIM_IT_Update) != RESET)
	{
        SoftTimer.u32MillisecondCount += 1; /*毫秒计时*/
        SoftTimer.u32SecondCount += 1;      /*秒计时*/

        /*10mS 定时*/
        if(SoftTimer.u32MillisecondCount >= SoftTimer.u32TenMilisecond)
        {
            SoftTimer.u32MillisecondCount = 0;

            JointCtrl.bJointProcessStart = TRUE;
            /*异步PDO发送使能*/
            JointCtrl.asynPdoEnable = TRUE;
            
            /*博思特绝对值编码器初始化校准*/
#if (('A' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE))
            if((PositalPara.bHardwareInit)&&(!PositalPara.PositalDataReady))
            {
                GetPositalActualAngle(PositalPara.i32AngleOffset);
            }
#endif
                
            /*10mS 进一次*/
            if(TeachingMode == JointCtrl.JointContrlMode)
            {
                if((FALSE == JointCtrl.MotionPath.bPathRecordFinish) && (TRUE == JointCtrl.MotionPath.bPathRecordEn))
                {
                    MotionPathRecorder();
                }
            }
            
            if(JointCtrl.MotionPath.bReappearStart)
            {
                JointCtrl.synPdoEnable = TRUE;
                MotionPathRestore();
            }
            
            syncSend();
            if(JointCtrl.synPdoEnable)
            {
                cspTargetPositionSend(JointCtrl.Motion.i32JointTargetPosition);
            }
        }

        /*1S 定时*/
        if(SoftTimer.u32SecondCount >= SoftTimer.u32OneSecond)
        {
            SoftTimer.u32SecondCount = 0;
            LEDLevel = !LEDLevel;
            LEDTwinkleProcess(LEDLevel);
        }
        
        /*SDO写操作超时计时*/
//        if(WriteProcess == Sdo_W_Para.Sdo_Status)
//        {
//            Sdo_W_Para.WriteLostCount += 1;
//        }
        
        /*SDO读操作超时计时*/
        if(ReadProcess == Sdo_R_Para.Sdo_Status)
        {
            Sdo_R_Para.ReadLostCount += 1;
        }

		TIM_ClearITPendingBit(TIM_GENERAL_TIMER, TIM_IT_Update);
	}
}


#define BIT_NUM 18
volatile int watch_clock = 0; 
volatile uint8_t clock_counter = 0;

// the cycles must greater or equal than 1
uint8_t ssi_clock_idle_cycles = 2;
uint8_t ssi_clock_start_cycles = 1;
uint8_t ssi_clock_high_cycles = 1;
uint8_t ssi_clock_low_cycles = 1;
uint8_t ssi_clock_flop_cycles = 6;

uint8_t idle_stage_end = 0;
uint8_t start_stage_end = 0;
uint8_t timer_stage_end = 0;
uint8_t flop_stage_end = 0;
uint8_t total_timer_high_low = 0;
uint8_t timer_period = 0;
uint8_t temp_mod_result = 0;
//uint8_t ssi_clock_time_seq[40]={0};
extern volatile uint8_t g_bit_of_absolute_encoder_data;
void getTempDataOfAbsoluteEncoder(void);
