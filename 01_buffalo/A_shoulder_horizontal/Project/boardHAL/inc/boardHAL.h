/********************************************************************************************
* @FileName: boardHAL.h
* @Date: 2017_3_8
* @Version: v1.0
* @Author: Han Yifeng
* @Summary: chipHAL: Chip Hardware Abstraction Layer,主要完成对芯片的抽象，当换芯片时，如果是同一
*				系列芯片，则只需更改CHAL层即可.
*			boardHAL: Board Hardware Abstraction Layer,主要完成对电路板的抽象，当换不同系列的芯片时，
*				需要更改CHAL和BHAL层.但是BHAL的接口不用更改！

********************************************************************************************/
#ifndef _BHAL_H__
#define _BHAL_H__

#include "stdint.h"

/**********************************************************
*
*						系统级模块(NVIC)
*
***********************************************************/
//注意：中断分组为group3，即3位表示抢占优先级，1位表示
//响应优先级。

/**********TIM1 interrupt priority configuration***********/
#define	TIM1_BRK_PRE_PRIORITY		0						//抢占优先级：0
#define TIM1_BRK_SUB_PRIORITY		0						//响应优先级：0

/***********ADC interrupt priority configuration***********/
//转换完成中断(EOC/JEOC)
#define ADC_PRE_PRIORITY			1						//抢占优先级：1
#define ADC_SUB_PRIORITY			0						//响应优先级：0



#define VOICE_MAX 499


//以下为汇编函数
void WFI_SET(void);            //执行WFI指令
void INTX_DISABLE(void);       //关闭所有中断
void INTX_ENABLE(void);        //开启所有中断
void MSR_MSP(uint32_t addr);	        //设置堆栈地址 


void boardHAL_init(void);



///*********EXTI interrupt priority configuration***********/
//#define ENCODER_CHZ_IRQn			EXTI2_IRQn				//z相外部中断
//#define ENCODER_CHZ_IRQHandler		EXTI2_IRQHandler		//z相外部中断
//#define ENCODRE_CHZ_LINE			EXTI_Line2				//EXTI of CHZ GPIO.
//#define ENCODER_CHZ_PRE				1
//#define ENCODER_CHZ_SUB				1

///*************SERVO_TIM priority configuration**********/
//#define SERVO_TIM_IRQn				TIM1_BRK_TIM9_IRQn
//#define SERVO_TIM_IRQHandler		TIM1_BRK_TIM9_IRQHandler
//#define SERVO_PRE_PRIORITY			2
//#define SERVO_SUB_PRIORITY			1



/********************
*
*    RS232模块
*
********************/
//void    BHAL_Rs232Tx( uint8_t  * source , uint8_t num);
//uint8_t BHAL_Rs232Rx( uint8_t * destination ,uint8_t num);
//uint8_t BHAL_Rs232TxLeft(void);
//uint8_t BHAL_Rs232RxNum(void);


/********************
*
*    时间模块
*
********************/
//void     BHAL_DelayUs(uint32_t us);
//uint32_t BHAL_GetSysRunTimeSec(void);
//uint32_t BHAL_GetThisTimeSec(void);
//void BHAL_SetThisTimeSec(uint32_t time);
//uint16_t BHAL_SysTimerMs(uint32_t *timer  ,uint16_t nextTime);


/********************
*
*    CRC模块
*
********************/
//void BHAL_ResetCrc(void);
//uint32_t BHAL_GetCrc(void);
//void  BHAL_InputCrc(uint32_t val);
//void BHAL_Reset(void);


/******************************
*
*    调试与指示灯模块
*
*******************************/
//void BHAL_LedBlueLight(void);
//void BHAL_LedBlueDark(void);
//void BHAL_LedRedLight(void);
//void BHAL_LedRedDark(void);


#endif
